package com.drone.controller.control

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sign

enum class GestureDirection(val label: String, val roll: Int, val pitch: Int) {
    LEFT("LEFT", -1, 0),
    RIGHT("RIGHT", 1, 0),
    FORWARD("FORWARD", 0, 1),
    BACK("BACK", 0, -1),
}

enum class GestureSize(val label: String) {
    SMALL("SLOW"),
    BIG("FAST"),
}

data class Gesture(
    val direction: GestureDirection,
    val size: GestureSize,
    val peakSpeed: Float, // m/s – estimated peak hand speed
)

data class GestureConfig(
    val startAccel: Float = 1.2f,   // m/s²: start tracking above this acceleration
    val minSpeed: Float = 0.12f,    // m/s: slower -> treated as hand tremor, ignored
    val fastSpeed: Float = 0.7f,    // m/s: at or above this it is a FAST flick
    val dominance: Float = 1.5f,    // main axis must be 1.5× the other (rejects diagonal flicks)
    val maxTrackMs: Long = 400,     // longer pushes are finalised anyway
    val cooldownMs: Long = 600,     // lockout after each gesture (ignores the opposite braking motion)
    val quietAccel: Float = 0.8f,   // m/s²: below this the hand counts as still
    val quietMs: Long = 150,        // must stay still this long before a new gesture
    val maxDtMs: Long = 50,         // cap integration step when sensor samples are missed
)

/**
 * Detects a "flick" of the phone in 4 horizontal directions.
 *
 * Input: acceleration in the world HORIZONTAL plane (gravity removed, aligned to the screen):
 *   rightAccel   > 0 : pushed to the right
 *   forwardAccel > 0 : pushed away from the user
 *
 * How it works:
 *  1. IDLE: wait for acceleration above [GestureConfig.startAccel], pick the main axis and sign.
 *  2. TRACKING: integrate acceleration along that axis -> hand speed. When the acceleration
 *     changes sign (the hand starts braking) the speed has peaked -> finalise the gesture.
 *  3. COOLDOWN: ignore the opposite braking motion, wait for the hand to be still, then IDLE.
 *
 * No Android dependency -> unit testable.
 */
class GestureDetector(var config: GestureConfig = GestureConfig()) {

    private enum class Phase { IDLE, TRACKING, COOLDOWN }

    private var phase = Phase.IDLE
    private var forwardAxis = false
    private var axisSign = 1f
    private var speed = 0f
    private var peakSpeed = 0f
    private var trackStartMs = 0L
    private var cooldownStartMs = 0L
    private var quietSinceMs = NO_TIME
    private var lastMs = NO_TIME

    /** Peak speed of the latest tracking (even if ignored as too slow), for display. */
    var lastPeakSpeed = 0f
        private set

    fun reset() {
        phase = Phase.IDLE
        speed = 0f
        peakSpeed = 0f
        quietSinceMs = NO_TIME
        lastMs = NO_TIME
    }

    /** Call for every sensor sample. Returns a gesture at the moment it is recognised, otherwise null. */
    fun update(rightAccel: Float, forwardAccel: Float, timeMs: Long): Gesture? {
        val dt = if (lastMs == NO_TIME) 0f else (timeMs - lastMs).coerceIn(0, config.maxDtMs) / 1000f
        lastMs = timeMs
        var result: Gesture? = null

        when (phase) {
            Phase.IDLE -> {
                val ar = abs(rightAccel)
                val af = abs(forwardAccel)
                if (max(ar, af) >= config.startAccel) {
                    when {
                        ar >= af * config.dominance -> startTracking(false, sign(rightAccel), timeMs)
                        af >= ar * config.dominance -> startTracking(true, sign(forwardAccel), timeMs)
                        else -> Unit // diagonal, unclear -> wait for the next sample
                    }
                    if (phase == Phase.TRACKING) integrate(rightAccel, forwardAccel, dt)
                }
            }

            Phase.TRACKING -> {
                val along = (if (forwardAxis) forwardAccel else rightAccel) * axisSign
                val timedOut = timeMs - trackStartMs >= config.maxTrackMs
                if (along < 0f || timedOut) {
                    result = finish(timeMs) // hand started braking: speed has peaked
                } else {
                    integrate(rightAccel, forwardAccel, dt)
                }
            }

            Phase.COOLDOWN -> {
                val quiet = max(abs(rightAccel), abs(forwardAccel)) < config.quietAccel
                quietSinceMs = when {
                    !quiet -> NO_TIME
                    quietSinceMs == NO_TIME -> timeMs
                    else -> quietSinceMs
                }
                val cooled = timeMs - cooldownStartMs >= config.cooldownMs
                val calm = quietSinceMs != NO_TIME && timeMs - quietSinceMs >= config.quietMs
                if (cooled && calm) {
                    phase = Phase.IDLE
                }
            }
        }
        return result
    }

    private fun startTracking(forward: Boolean, direction: Float, timeMs: Long) {
        phase = Phase.TRACKING
        forwardAxis = forward
        axisSign = direction
        speed = 0f
        peakSpeed = 0f
        trackStartMs = timeMs
    }

    private fun integrate(rightAccel: Float, forwardAccel: Float, dt: Float) {
        val along = (if (forwardAxis) forwardAccel else rightAccel) * axisSign
        speed += along * dt
        peakSpeed = max(peakSpeed, speed)
    }

    private fun finish(timeMs: Long): Gesture? {
        phase = Phase.COOLDOWN
        cooldownStartMs = timeMs
        quietSinceMs = NO_TIME
        lastPeakSpeed = peakSpeed
        if (peakSpeed < config.minSpeed) return null

        val direction = when {
            forwardAxis && axisSign > 0f -> GestureDirection.FORWARD
            forwardAxis -> GestureDirection.BACK
            axisSign > 0f -> GestureDirection.RIGHT
            else -> GestureDirection.LEFT
        }
        val size = if (peakSpeed >= config.fastSpeed) GestureSize.BIG else GestureSize.SMALL
        return Gesture(direction, size, peakSpeed)
    }

    private companion object {
        const val NO_TIME = Long.MIN_VALUE
    }
}
