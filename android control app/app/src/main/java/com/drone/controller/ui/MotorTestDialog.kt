package com.drone.controller.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.controller.MotorTestState
import com.drone.controller.net.Telemetry
import kotlin.math.roundToInt

private val MOTOR_NAMES = listOf("M1\nfront-right", "M2\nrear-right", "M3\nrear-left", "M4\nfront-left")

/**
 * Ground motor test: spin the selected motors at a fixed power while the big button is HELD.
 * Works on battery only and without the IMU (no arming, no stabilisation) – the board may move.
 * Shows the battery voltage live and its lowest value during the run (supply sag check).
 */
@Composable
fun MotorTestDialog(
    state: MotorTestState,
    tel: Telemetry?,
    alive: Boolean,
    onToggleMotor: (Int) -> Unit,
    onPower: (Int) -> Unit,
    onStart: () -> Boolean,
    onStop: () -> Unit,
    onDismiss: () -> Unit,
) {
    // Leaving the dialog always stops the motors
    DisposableEffect(Unit) { onDispose { onStop() } }

    AlertDialog(
        onDismissRequest = {
            onStop()
            onDismiss()
        },
        title = { Text("Motor test") },
        text = {
            Column(
                Modifier.verticalScroll(rememberScrollState()),
                verticalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                Text(
                    "Spins the selected motors while you HOLD the button – no sensor, no stabilisation. " +
                        "Remove the props, or hold / clamp the drone firmly.",
                    color = WarnAmber,
                    fontSize = 13.sp,
                )

                Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    MOTOR_NAMES.forEachIndexed { i, name ->
                        FilterChip(
                            selected = ((state.mask shr i) and 1) == 1,
                            onClick = { onToggleMotor(i) },
                            enabled = !state.running,
                            label = { Text(name, fontSize = 11.sp, textAlign = TextAlign.Center, lineHeight = 12.sp) },
                        )
                    }
                }

                Column {
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                        Text("Power", fontWeight = FontWeight.SemiBold)
                        Text("${state.powerPct} %", color = MaterialTheme.colorScheme.primary, fontWeight = FontWeight.SemiBold)
                    }
                    Slider(
                        value = state.powerPct.toFloat(),
                        onValueChange = { onPower((it / POWER_STEP).roundToInt() * POWER_STEP) },
                        valueRange = 0f..100f,
                        steps = 100 / POWER_STEP - 1,
                    )
                }

                VoltageReadout(state, tel, alive)

                HoldButton(running = state.running, confirmed = tel?.motorTest == true, onStart = onStart, onStop = onStop)
            }
        },
        confirmButton = {
            TextButton(onClick = {
                onStop()
                onDismiss()
            }) { Text("Close") }
        },
    )
}

@Composable
private fun VoltageReadout(state: MotorTestState, tel: Telemetry?, alive: Boolean) {
    val now = if (alive && tel != null) "%.2f V".format(tel.batteryVolts) else "– (no signal)"
    val lowest = state.minVolts
    val min = lowest?.let { "%.2f V".format(it) } ?: "–"
    val minColor = when {
        lowest == null -> MaterialTheme.colorScheme.onSurfaceVariant
        lowest < SAG_DANGER_V -> StopRed
        lowest < SAG_WARN_V -> WarnAmber
        else -> ArmGreen
    }
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text("Battery now: $now", fontSize = 13.sp)
        Text("Lowest in test: $min", color = minColor, fontSize = 13.sp, fontWeight = FontWeight.SemiBold)
    }
}

/** Big press-and-hold button: motors run only while a finger is on it. */
@Composable
private fun HoldButton(running: Boolean, confirmed: Boolean, onStart: () -> Boolean, onStop: () -> Unit) {
    val color = when {
        running && confirmed -> ArmGreen
        running -> WarnAmber
        else -> MaterialTheme.colorScheme.surfaceVariant
    }
    Box(
        Modifier
            .fillMaxWidth()
            .height(72.dp)
            .clip(RoundedCornerShape(16.dp))
            .background(color)
            .pointerInput(Unit) {
                detectTapGestures(
                    onPress = {
                        if (onStart()) {
                            try {
                                tryAwaitRelease()
                            } finally {
                                onStop()
                            }
                        }
                    },
                )
            },
        contentAlignment = Alignment.Center,
    ) {
        Text(
            when {
                running && confirmed -> "SPINNING – release to stop"
                running -> "Starting…"
                else -> "HOLD TO SPIN"
            },
            color = Color.White,
            fontWeight = FontWeight.Bold,
            fontSize = 18.sp,
        )
    }
}

private const val POWER_STEP = 5
private const val SAG_WARN_V = 3.6f   // below this under load: the battery / wiring is struggling
private const val SAG_DANGER_V = 3.4f // below this the XIAO is close to resetting
