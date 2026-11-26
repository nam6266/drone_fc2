package com.drone.controller

import android.app.Application
import android.content.Context
import android.os.SystemClock
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.drone.controller.control.Gesture
import com.drone.controller.control.GestureDetector
import com.drone.controller.control.Maneuver
import com.drone.controller.control.ManeuverConfig
import com.drone.controller.net.Command
import com.drone.controller.net.DroneLink
import com.drone.controller.net.DroneState
import com.drone.controller.net.Telemetry
import com.drone.controller.net.WifiStatus
import com.drone.controller.sensor.MotionSensor
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlin.math.roundToInt

enum class ControlMode(val label: String) {
    JOYSTICK("Joystick"),
    GESTURE("Gesture"),
}

enum class AutoAction(val label: String) {
    TAKEOFF("Taking off…"),
    LANDING("Landing…"),
}

/** User-adjustable settings, stored on the phone (SharedPreferences). */
data class FlightSettings(
    val hoverThrottlePct: Int = 45, // power that keeps the drone hovering = right stick centre
    val climbRangePct: Int = 15,    // right stick fully up/down adds/removes this much power
    val bigDistanceCm: Int = 30,    // FAST flick
    val smallDistanceCm: Int = 8,   // SLOW flick
    val fastSpeedCmS: Int = 70,     // hand faster than this (cm/s) = FAST flick
)

/**
 * Control state set by the user.
 *
 * Power model (there is no altitude sensor yet):
 *   basePower = 0 while landed, = hover power once airborne (set by TAKE OFF / LAND ramps)
 *   climb     = right stick vertical, springs back to 0 -> centre means "hold altitude"
 *   throttle  = basePower ± climb × climbRangePct   (0 while landed, whatever the stick does)
 */
data class ControlState(
    val mode: ControlMode = ControlMode.JOYSTICK,
    val basePower: Int = 0,      // 0..1000, 0 = landed
    val climb: Int = 0,          // right stick vertical, -1000..1000, + = fly up
    val climbRangePct: Int = 15, // copy of FlightSettings.climbRangePct
    val yaw: Int = 0,            // right stick horizontal, -1000..1000, + = turn right
    val stickRoll: Int = 0,      // left stick, -1000..1000, + = right
    val stickPitch: Int = 0,     // left stick, -1000..1000, + = forward
    val arm: Boolean = false,
    val stop: Boolean = false,
    val autoAction: AutoAction? = null,
    val message: String? = null,
) {
    /** true once TAKE OFF has brought the power up (and until LAND / disarm). */
    val flying: Boolean get() = basePower > 0

    /** Power actually sent to the drone, 0..1000. */
    val throttle: Int
        get() = if (basePower <= 0) 0 else (basePower + climb * climbRangePct / 100).coerceIn(0, POWER_MAX)

    private companion object {
        const val POWER_MAX = 1000
    }
}

/**
 * Ground motor test (works on battery only, no IMU / arming needed).
 * The selected motors spin at [powerPct] only while [running] (the user holds the button).
 */
data class MotorTestState(
    val mask: Int = 0b0001,          // bit0..3 = M1..M4
    val powerPct: Int = 30,
    val running: Boolean = false,
    val minVolts: Float? = null,     // lowest battery voltage seen during this run
)

/** Latest gesture result, for display. */
data class GestureUi(
    val last: Gesture? = null,
    val note: String = "Flick the phone left/right or away/toward you",
    val executing: Boolean = false,
    val count: Int = 0,
)

class DroneViewModel(app: Application) : AndroidViewModel(app) {

    private val prefs = app.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
    private val motion = MotionSensor(app)
    private val detector = GestureDetector()
    private var maneuverConfig = ManeuverConfig()
    private val link = DroneLink(app, viewModelScope) { currentCommand() }

    private val _control = MutableStateFlow(ControlState())
    val control: StateFlow<ControlState> = _control.asStateFlow()

    private val _settings = MutableStateFlow(loadSettings())
    val settings: StateFlow<FlightSettings> = _settings.asStateFlow()

    private val _motorTest = MutableStateFlow(MotorTestState())
    val motorTest: StateFlow<MotorTestState> = _motorTest.asStateFlow()

    private val _gesture = MutableStateFlow(GestureUi())
    val gesture: StateFlow<GestureUi> = _gesture.asStateFlow()

    val telemetry: StateFlow<Telemetry?> = link.telemetry
    val wifiStatus: StateFlow<WifiStatus> = link.wifiStatus

    private val _linkAlive = MutableStateFlow(false)
    /** true while telemetry is "fresh" (< LINK_TIMEOUT_MS). */
    val linkAlive: StateFlow<Boolean> = _linkAlive.asStateFlow()

    val gestureAvailable: Boolean get() = motion.available

    /** Maneuver in progress. Read on the IO thread (DroneLink), written on the main thread. */
    @Volatile private var maneuver: Maneuver? = null
    private var autoJob: Job? = null
    private var armRequestedAt = 0L
    private var started = false

    init {
        applySettings(_settings.value)
        _control.update { it.copy(climbRangePct = _settings.value.climbRangePct) }
        motion.listener = MotionSensor.Listener { right, forward, timeMs -> onHorizontalAccel(right, forward, timeMs) }
        viewModelScope.launch {
            while (true) {
                checkLink()
                checkManeuverDone()
                delay(TICK_MS)
            }
        }
    }

    // ---------- Lifecycle (called from the Activity) ----------
    fun onStart() {
        started = true
        link.start()
        updateMotionSensor()
    }

    fun onStop() {
        // Leaving the app: stop sending -> firmware enters FAILSAFE and descends.
        // Reset arm/power so returning to the app does NOT resend the old flight command.
        started = false
        cancelAuto()
        maneuver = null
        _control.update { it.copy(arm = false, basePower = 0, climb = 0, yaw = 0, stickRoll = 0, stickPitch = 0) }
        stopMotorTest()
        motion.stop()
        link.stop()
    }

    fun setDisplayRotation(rotation: Int) {
        motion.displayRotation = rotation
    }

    override fun onCleared() {
        motion.stop()
        link.stop()
    }

    // ---------- Mode ----------
    fun setMode(mode: ControlMode) {
        if (mode == ControlMode.GESTURE && !motion.available) {
            showMessage("This phone lacks the sensors needed for Gesture mode")
            return
        }
        maneuver = null
        _control.update { it.copy(mode = mode, stickRoll = 0, stickPitch = 0) }
        updateMotionSensor()
    }

    // ---------- Joystick ----------
    /** Left stick: [x], [y] in -1..1 (y + = up = move forward). */
    fun setLeftStick(x: Float, y: Float) {
        _control.update { it.copy(stickRoll = expo(x), stickPitch = expo(y)) }
    }

    /**
     * Right stick: [vertical] -1..1 (+ = fly up), [yaw] -1..1 (+ = turn right). Both spring back to 0.
     * Only works once airborne. Pushing up during LAND cancels the landing.
     */
    fun setRightStick(vertical: Float, yaw: Float) {
        val climb = expo(vertical)
        val c = _control.value
        if (c.autoAction == AutoAction.LANDING && climb > ABORT_LANDING_CLIMB) {
            abortLanding()
        }
        if (!c.flying && c.arm && climb > LANDED_HINT_CLIMB) {
            showMessage("Press TAKE OFF first – the stick controls height once airborne")
        }
        _control.update { it.copy(climb = if (it.flying) climb else 0, yaw = expo(yaw)) }
    }

    // ---------- ARM / STOP ----------
    fun toggleArm() {
        val c = _control.value
        if (c.arm) {
            cancelAuto()
            maneuver = null
            _control.update { it.copy(arm = false, basePower = 0, climb = 0) }
            return
        }
        when {
            !_linkAlive.value -> showMessage("No signal from the drone yet")
            link.telemetry.value?.imuMissing == true ->
                showMessage("Motion sensor (MPU6500) not found – check its wiring. The drone keeps retrying.")
            link.telemetry.value?.imuCalibrating == true ->
                showMessage("Sensor is calibrating – keep the drone still for ~2 s, then press ARM")
            else -> {
                stopMotorTest()
                armRequestedAt = SystemClock.elapsedRealtime()
                _control.update { it.copy(arm = true, stop = false, message = null) }
            }
        }
    }

    // ---------- Motor test ----------
    fun toggleTestMotor(index: Int) {
        _motorTest.update { it.copy(mask = it.mask xor (1 shl index)) }
    }

    fun setTestPower(percent: Int) {
        _motorTest.update { it.copy(powerPct = percent.coerceIn(0, 100)) }
    }

    /** Called when the user presses the HOLD button. Returns false (with a message) if not allowed. */
    fun startMotorTest(): Boolean {
        val reason = when {
            !_linkAlive.value -> "No signal from the drone yet"
            _control.value.arm -> "Disarm first – the motor test only runs while DISARMED"
            _motorTest.value.mask == 0 -> "Select at least one motor"
            else -> null
        }
        if (reason != null) {
            showMessage(reason)
            return false
        }
        _control.update { it.copy(stop = false) }
        _motorTest.update { it.copy(running = true, minVolts = null) }
        return true
    }

    /** Called when the user releases the HOLD button (or leaves the dialog / the app). */
    fun stopMotorTest() {
        _motorTest.update { it.copy(running = false) }
    }

    fun emergencyStop() {
        stopMotorTest()
        cancelAuto()
        maneuver = null
        _control.update {
            it.copy(arm = false, stop = true, basePower = 0, climb = 0, yaw = 0, stickRoll = 0, stickPitch = 0)
        }
    }

    // ---------- Take-off / landing ----------
    fun takeOff() {
        val c = _control.value
        when {
            !c.arm || link.telemetry.value?.state != DroneState.ARMED -> showMessage("Press ARM before take-off")
            c.autoAction != null -> Unit
            c.flying -> showMessage("Already airborne")
            else -> {
                runThrottleProfile(AutoAction.TAKEOFF, listOf(Segment(hoverPower(), TAKEOFF_MS))) {
                    showMessage("Hovering – right stick up/down to change height, release to hold")
                }
            }
        }
    }

    /**
     * Landing (time based, since there is no altitude sensor):
     *   quickly drop to 80 % of hover power -> hold so the drone sinks slowly -> ramp to 0 -> disarm.
     * Best used when the drone is low (< 1 m).
     */
    fun land() {
        val c = _control.value
        if (!c.arm || !c.flying) {
            showMessage("Drone is not airborne")
            return
        }
        val descend = minOf(hoverPower() * LAND_DESCEND_PERCENT / 100, c.basePower)
        val profile = listOf(
            Segment(descend, LAND_DROP_MS),
            Segment(descend, LAND_HOLD_MS),
            Segment(0, LAND_CUT_MS),
        )
        maneuver = null
        runThrottleProfile(AutoAction.LANDING, profile) {
            _control.update { it.copy(arm = false, climb = 0) }
            showMessage("Landed")
        }
    }

    /** Stick pushed up during LAND: stop landing and go back to hover power. */
    private fun abortLanding() {
        runThrottleProfile(AutoAction.TAKEOFF, listOf(Segment(hoverPower(), ABORT_LANDING_MS))) {
            showMessage("Landing cancelled – hovering")
        }
    }

    private fun hoverPower(): Int = _settings.value.hoverThrottlePct * CMD_MAX / 100

    // ---------- Settings ----------
    fun updateSettings(newSettings: FlightSettings) {
        _settings.value = newSettings
        applySettings(newSettings)
        _control.update {
            // While hovering (no ramp running), follow a new hover power right away
            val base = if (it.flying && it.autoAction == null) hoverPower() else it.basePower
            it.copy(basePower = base, climbRangePct = newSettings.climbRangePct)
        }
        prefs.edit()
            .putInt(KEY_HOVER, newSettings.hoverThrottlePct)
            .putInt(KEY_CLIMB, newSettings.climbRangePct)
            .putInt(KEY_BIG, newSettings.bigDistanceCm)
            .putInt(KEY_SMALL, newSettings.smallDistanceCm)
            .putInt(KEY_FAST, newSettings.fastSpeedCmS)
            .apply()
    }

    fun reconnect() {
        link.stop()
        link.start()
    }

    fun clearMessage() = _control.update { it.copy(message = null) }

    private fun showMessage(msg: String) = _control.update { it.copy(message = msg) }

    // ---------- Internals ----------

    /** Called on the IO thread every 20 ms by DroneLink. */
    private fun currentCommand(): Command {
        val c = _control.value
        val t = _motorTest.value
        if (t.running && !c.arm && !c.stop) {
            return Command(throttle = t.powerPct * CMD_MAX / 100, testMask = t.mask)
        }
        var roll = 0
        var pitch = 0
        when (c.mode) {
            ControlMode.JOYSTICK -> {
                roll = c.stickRoll
                pitch = c.stickPitch
            }
            ControlMode.GESTURE -> maneuver?.sticks(SystemClock.elapsedRealtime())?.let {
                roll = it.first
                pitch = it.second
            }
        }
        return Command(arm = c.arm && !c.stop, stop = c.stop, roll = roll, pitch = pitch, yaw = c.yaw, throttle = c.throttle)
    }

    private fun onHorizontalAccel(right: Float, forward: Float, timeMs: Long) {
        val gesture = detector.update(right, forward, timeMs) ?: return
        val c = _control.value
        if (c.mode != ControlMode.GESTURE) return

        val flying = c.arm && _linkAlive.value && link.telemetry.value?.state == DroneState.ARMED
        val reason = when {
            !flying -> "Not armed – preview only, for practice"
            !c.flying -> "Not airborne – press TAKE OFF first"
            c.autoAction != null -> "${c.autoAction.label} ignored"
            maneuver != null -> "Still moving – gesture ignored"
            else -> null
        }
        val note = if (reason == null) {
            val m = Maneuver.plan(gesture, maneuverConfig, SystemClock.elapsedRealtime())
            maneuver = m
            "Moving ${gesture.direction.label} ~${m.distanceCm.roundToInt()} cm"
        } else {
            reason
        }
        _gesture.update { GestureUi(last = gesture, note = note, executing = reason == null, count = it.count + 1) }
    }

    private fun checkManeuverDone() {
        val m = maneuver
        if (m != null && m.isDone(SystemClock.elapsedRealtime())) {
            maneuver = null
        }
        val executing = maneuver != null
        if (_gesture.value.executing != executing) {
            _gesture.update { it.copy(executing = executing) }
        }
    }

    private fun checkLink() {
        val now = SystemClock.elapsedRealtime()
        val tel = link.telemetry.value
        val alive = tel != null && now - link.lastTelemetryAt < LINK_TIMEOUT_MS
        _linkAlive.value = alive

        // Motor test: remember the lowest battery voltage (shows how much the supply sags)
        val t = _motorTest.value
        if (t.running) {
            if (!alive) {
                stopMotorTest()
                showMessage("Motor test stopped: signal lost (did the XIAO reset? Check the power wiring)")
            } else if (tel != null && tel.motorTest) {
                val v = tel.batteryVolts
                if (t.minVolts == null || v < t.minVolts) {
                    _motorTest.update { it.copy(minVolts = v) }
                }
            }
        }

        // App is armed but the drone still reports DISARMED -> it refused or disarmed itself
        // (crash, failsafe...). Clear arm in the app so the next ARM press creates a 0->1 edge.
        val c = _control.value
        if (c.arm && alive && tel?.state == DroneState.DISARMED && now - armRequestedAt > ARM_CONFIRM_MS) {
            cancelAuto()
            maneuver = null
            _control.update {
                it.copy(
                    arm = false,
                    basePower = 0,
                    climb = 0,
                    message = when {
                        tel?.imuMissing == true -> "Drone refused ARM: motion sensor (MPU6500) not found – check its wiring"
                        tel?.imuCalibrating == true -> "Drone refused ARM: sensor still calibrating – keep the drone still"
                        else -> "Drone refused ARM or disarmed itself (see Serial for the reason)"
                    },
                )
            }
        }
    }

    private data class Segment(val targetPower: Int, val durationMs: Long)

    /** Ramps basePower through each segment in turn (linearly), then calls [onDone]. */
    private fun runThrottleProfile(action: AutoAction, segments: List<Segment>, onDone: () -> Unit) {
        autoJob?.cancel()
        _control.update { it.copy(autoAction = action) }
        autoJob = viewModelScope.launch {
            for (seg in segments) {
                val from = _control.value.basePower
                val start = SystemClock.elapsedRealtime()
                while (true) {
                    val k = ((SystemClock.elapsedRealtime() - start).toFloat() / seg.durationMs).coerceIn(0f, 1f)
                    val power = (from + (seg.targetPower - from) * k).roundToInt()
                    _control.update { it.copy(basePower = power) }
                    if (k >= 1f) break
                    delay(RAMP_STEP_MS)
                }
            }
            _control.update { it.copy(autoAction = null) }
            autoJob = null
            onDone()
        }
    }

    private fun cancelAuto() {
        autoJob?.cancel()
        autoJob = null
        if (_control.value.autoAction != null) {
            _control.update { it.copy(autoAction = null) }
        }
    }

    private fun updateMotionSensor() {
        if (started && _control.value.mode == ControlMode.GESTURE) {
            detector.reset()
            motion.start()
        } else {
            motion.stop()
        }
    }

    private fun applySettings(s: FlightSettings) {
        detector.config = detector.config.copy(fastSpeed = s.fastSpeedCmS / 100f)
        maneuverConfig = maneuverConfig.copy(
            bigDistanceCm = s.bigDistanceCm.toFloat(),
            smallDistanceCm = s.smallDistanceCm.toFloat(),
        )
    }

    private fun loadSettings(): FlightSettings {
        val d = FlightSettings()
        return FlightSettings(
            hoverThrottlePct = prefs.getInt(KEY_HOVER, d.hoverThrottlePct),
            climbRangePct = prefs.getInt(KEY_CLIMB, d.climbRangePct),
            bigDistanceCm = prefs.getInt(KEY_BIG, d.bigDistanceCm),
            smallDistanceCm = prefs.getInt(KEY_SMALL, d.smallDistanceCm),
            fastSpeedCmS = prefs.getInt(KEY_FAST, d.fastSpeedCmS),
        )
    }

    /** Soft around centre (fine control), still 100 % at the edge. */
    private fun expo(v: Float): Int {
        val x = v.coerceIn(-1f, 1f)
        return ((x * (1f - EXPO) + x * x * x * EXPO) * CMD_MAX).roundToInt()
    }

    companion object {
        const val CMD_MAX = 1000
        const val EXPO = 0.3f
        const val ABORT_LANDING_CLIMB = 300    // stick 30 % up during LAND cancels the landing
        const val LANDED_HINT_CLIMB = 500      // stick pushed up while landed -> show a hint
        const val ABORT_LANDING_MS = 500L
        const val TAKEOFF_MS = 1500L
        const val LAND_DESCEND_PERCENT = 80     // % of hover power while descending
        const val LAND_DROP_MS = 500L
        const val LAND_HOLD_MS = 2500L
        const val LAND_CUT_MS = 500L
        const val RAMP_STEP_MS = 20L
        const val TICK_MS = 100L
        const val LINK_TIMEOUT_MS = 600L
        const val ARM_CONFIRM_MS = 1000L

        private const val PREFS_NAME = "drone_settings"
        private const val KEY_HOVER = "hover_throttle_pct"
        private const val KEY_CLIMB = "climb_range_pct"
        private const val KEY_BIG = "big_distance_cm"
        private const val KEY_SMALL = "small_distance_cm"
        private const val KEY_FAST = "fast_speed_cm_s"
    }
}
