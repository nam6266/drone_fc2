package com.drone.controller.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.systemBarsPadding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.controller.ControlMode
import com.drone.controller.DroneViewModel
import com.drone.controller.net.DroneState
import com.drone.controller.net.Telemetry
import com.drone.controller.net.WifiStatus

/**
 * Landscape screen:
 *  ┌ top bar: [MODE] round menu | state, battery, temperature | TAKE OFF, LAND, ⚙ ┐
 *  │ left: steering stick (or gesture panel) │ centre: ARM, STOP │ right: fly up/down + yaw stick │
 */
@Composable
fun ControlScreen(vm: DroneViewModel) {
    val control by vm.control.collectAsState()
    val telemetry by vm.telemetry.collectAsState()
    val wifi by vm.wifiStatus.collectAsState()
    val alive by vm.linkAlive.collectAsState()
    val gesture by vm.gesture.collectAsState()
    val settings by vm.settings.collectAsState()
    val motorTest by vm.motorTest.collectAsState()
    var showSettings by remember { mutableStateOf(false) }
    var showMotorTest by remember { mutableStateOf(false) }

    Box(
        Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background),
    ) {
        Column(
            Modifier
                .fillMaxSize()
                .systemBarsPadding()
                .padding(horizontal = SCREEN_PADDING_H, vertical = SCREEN_PADDING_V),
            verticalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            TopBar(
                wifi = wifi,
                alive = alive,
                tel = telemetry,
                armed = control.arm,
                flying = control.flying,
                busy = control.autoAction != null,
                onReconnect = vm::reconnect,
                onTakeOff = vm::takeOff,
                onLand = vm::land,
                onSettings = { showSettings = true },
                onMotorTest = { showMotorTest = true },
            )

            control.message?.let { MessageBar(it, onDismiss = vm::clearMessage) }

            Row(
                Modifier.weight(1f).fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(12.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                // ---- Left: steering ----
                BoxWithConstraints(Modifier.weight(1f).fillMaxHeight(), contentAlignment = Alignment.Center) {
                    val side = minOf(maxWidth, maxHeight)
                    when (control.mode) {
                        ControlMode.JOYSTICK -> Joystick(
                            x = control.stickRoll / CMD_MAX,
                            y = control.stickPitch / CMD_MAX,
                            onMove = vm::setLeftStick,
                            springX = true,
                            springY = true,
                            circular = true,
                            modifier = Modifier.size(side),
                            labelTop = "FORWARD",
                            labelBottom = "BACK",
                            labelLeft = "LEFT",
                            labelRight = "RIGHT",
                        )
                        ControlMode.GESTURE -> GesturePanel(gesture, settings, Modifier.fillMaxSize())
                    }
                }

                // ---- Giữa: ARM / STOP ----
                Column(
                    Modifier.width(170.dp).fillMaxHeight(),
                    verticalArrangement = Arrangement.spacedBy(10.dp, Alignment.CenterVertically),
                    horizontalAlignment = Alignment.CenterHorizontally,
                ) {
                    Text(
                        when {
                            control.flying -> "Power ${control.throttle / 10} %   Yaw ${control.yaw / 10}"
                            control.arm -> "Landed – press TAKE OFF"
                            else -> "Landed"
                        },
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        fontSize = 13.sp,
                    )
                    control.autoAction?.let { Text(it.label, color = WarnAmber, fontWeight = FontWeight.SemiBold) }
                    Button(
                        onClick = vm::toggleArm,
                        enabled = alive || control.arm,
                        modifier = Modifier.fillMaxWidth().height(64.dp),
                        shape = RoundedCornerShape(16.dp),
                        colors = ButtonDefaults.buttonColors(
                            containerColor = if (control.arm) ArmGreen else MaterialTheme.colorScheme.surfaceVariant,
                            contentColor = Color.White,
                        ),
                    ) {
                        Text(
                            if (control.arm) "ARMED\n(tap to disarm)" else "ARM",
                            fontWeight = FontWeight.Bold,
                            textAlign = TextAlign.Center,
                        )
                    }
                    Button(
                        onClick = vm::emergencyStop,
                        modifier = Modifier.fillMaxWidth().height(72.dp),
                        shape = RoundedCornerShape(16.dp),
                        colors = ButtonDefaults.buttonColors(containerColor = StopRed, contentColor = Color.White),
                    ) {
                        Text("STOP", fontSize = 26.sp, fontWeight = FontWeight.Black)
                    }
                }

                // ---- Right: fly up/down (vertical) + yaw (horizontal), both spring back: centre = hold height ----
                BoxWithConstraints(Modifier.weight(1f).fillMaxHeight(), contentAlignment = Alignment.Center) {
                    val side = minOf(maxWidth, maxHeight)
                    Joystick(
                        x = control.yaw / CMD_MAX,
                        y = control.climb / CMD_MAX,
                        onMove = { x, y -> vm.setRightStick(vertical = y, yaw = x) },
                        springX = true,
                        springY = true,
                        circular = false,
                        modifier = Modifier.size(side),
                        labelTop = "FLY UP ▲",
                        labelBottom = "FLY DOWN ▼",
                        labelLeft = "↺",
                        labelRight = "↻",
                    )
                }
            }
        }

        // Round MODE menu drawn on top of everything (its open buttons overlap the screen),
        // placed exactly over the space the top bar reserves for it.
        ModeMenu(
            current = control.mode,
            onSelect = vm::setMode,
            modifier = Modifier
                .systemBarsPadding()
                .padding(horizontal = SCREEN_PADDING_H, vertical = SCREEN_PADDING_V),
        )
    }

    if (showSettings) {
        SettingsDialog(
            initial = settings,
            onSave = {
                vm.updateSettings(it)
                showSettings = false
            },
            onDismiss = { showSettings = false },
        )
    }

    if (showMotorTest) {
        MotorTestDialog(
            state = motorTest,
            tel = telemetry,
            alive = alive,
            onToggleMotor = vm::toggleTestMotor,
            onPower = vm::setTestPower,
            onStart = vm::startMotorTest,
            onStop = vm::stopMotorTest,
            onDismiss = { showMotorTest = false },
        )
    }
}

private const val CMD_MAX = 1000f
private val SCREEN_PADDING_H = 12.dp
private val SCREEN_PADDING_V = 8.dp

// ---------------------------------------------------------------------------

@Composable
private fun TopBar(
    wifi: WifiStatus,
    alive: Boolean,
    tel: Telemetry?,
    armed: Boolean,
    flying: Boolean,
    busy: Boolean,
    onReconnect: () -> Unit,
    onTakeOff: () -> Unit,
    onLand: () -> Unit,
    onSettings: () -> Unit,
    onMotorTest: () -> Unit,
) {
    Row(
        Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Spacer(Modifier.size(ModeButtonSize)) // the round MODE menu sits here (see ModeMenu)

        // Status
        val (stateText, stateColor) = when {
            !alive || tel == null -> "NO SIGNAL" to WarnAmber
            tel.state == DroneState.ARMED -> tel.state.label to ArmGreen
            tel.state == DroneState.FAILSAFE || tel.state == DroneState.ERROR -> tel.state.label to StopRed
            else -> tel.state.label to Color.White
        }
        Column(Modifier.weight(1f)) {
            Text(stateText, color = stateColor, fontWeight = FontWeight.Bold, fontSize = 15.sp)
            if (tel != null && alive) {
                Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                    Text(
                        "Battery %.2f V · %d%%".format(tel.batteryVolts, tel.batteryPercent) +
                            if (tel.lowBattery) " LOW" else "",
                        color = if (tel.lowBattery) StopRed else ArmGreen,
                        fontSize = 12.sp,
                        fontWeight = FontWeight.SemiBold,
                    )
                    ChipTemperature(tel.chipTempC)
                    ImuStatus(tel)
                }
            } else {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text(wifi.label, color = MaterialTheme.colorScheme.onSurfaceVariant, fontSize = 12.sp)
                    if (wifi == WifiStatus.UNAVAILABLE || wifi == WifiStatus.ERROR || wifi == WifiStatus.IDLE) {
                        TextButton(onClick = onReconnect) { Text("Reconnect", fontSize = 12.sp) }
                    }
                }
            }
        }

        OutlinedButton(onClick = onTakeOff, enabled = armed && !flying && !busy) { Text("TAKE OFF") }
        OutlinedButton(onClick = onLand, enabled = armed && flying) { Text("LAND") }
        TextButton(onClick = onMotorTest, enabled = !armed) { Text("TEST") }
        TextButton(onClick = onSettings) { Text("⚙", fontSize = 22.sp) }
    }
}

/** Shown only when the motion sensor is not ready (the drone refuses ARM until it is). */
@Composable
private fun ImuStatus(tel: Telemetry) {
    when {
        tel.imuMissing -> Text("Sensor NOT FOUND", color = StopRed, fontSize = 12.sp, fontWeight = FontWeight.SemiBold)
        tel.imuCalibrating -> Text("Sensor calibrating – keep still", color = WarnAmber, fontSize = 12.sp, fontWeight = FontWeight.SemiBold)
    }
}

/** Chip temperature: < 60 °C normal, 60–70 °C warning, > 70 °C hot. */
@Composable
private fun ChipTemperature(celsius: Float?) {
    val (text, color) = when {
        celsius == null -> "Chip --" to MaterialTheme.colorScheme.onSurfaceVariant
        celsius >= CHIP_TEMP_HOT_C -> "Chip %.0f °C HOT".format(celsius) to StopRed
        celsius >= CHIP_TEMP_WARN_C -> "Chip %.0f °C".format(celsius) to WarnAmber
        else -> "Chip %.0f °C".format(celsius) to MaterialTheme.colorScheme.onSurfaceVariant
    }
    Text(text, color = color, fontSize = 12.sp, fontWeight = FontWeight.SemiBold)
}

private const val CHIP_TEMP_WARN_C = 60f
private const val CHIP_TEMP_HOT_C = 70f

@Composable
private fun MessageBar(text: String, onDismiss: () -> Unit) {
    Surface(color = WarnAmber.copy(alpha = 0.18f), shape = RoundedCornerShape(12.dp)) {
        Row(Modifier.fillMaxWidth().padding(start = 12.dp), verticalAlignment = Alignment.CenterVertically) {
            Text(text, color = WarnAmber, modifier = Modifier.weight(1f), fontSize = 14.sp)
            Spacer(Modifier.width(8.dp))
            TextButton(onClick = onDismiss) { Text("OK") }
        }
    }
}
