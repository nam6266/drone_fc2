package com.drone.controller.ui

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.controller.FlightSettings
import kotlin.math.roundToInt

@Composable
fun SettingsDialog(initial: FlightSettings, onSave: (FlightSettings) -> Unit, onDismiss: () -> Unit) {
    var s by remember { mutableStateOf(initial) }

    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Flight settings") },
        text = {
            Column(
                Modifier.verticalScroll(rememberScrollState()),
                verticalArrangement = Arrangement.spacedBy(4.dp),
            ) {
                SettingSlider(
                    "Hover power", "${s.hoverThrottlePct} %",
                    "Power that keeps the drone hovering = right stick centre. TAKE OFF ramps up to this level.",
                    s.hoverThrottlePct, 20..80, 1,
                ) { s = s.copy(hoverThrottlePct = it) }
                SettingSlider(
                    "Up/Down strength", "± ${s.climbRangePct} %",
                    "Right stick fully up/down adds/removes this much power around hover power.",
                    s.climbRangePct, 5..30, 1,
                ) { s = s.copy(climbRangePct = it) }
                SettingSlider(
                    "FAST flick", "${s.bigDistanceCm} cm", "Estimated distance. Measure in a test flight, then adjust.",
                    s.bigDistanceCm, 10..80, 5,
                ) { s = s.copy(bigDistanceCm = it) }
                SettingSlider(
                    "SLOW flick", "${s.smallDistanceCm} cm", "",
                    s.smallDistanceCm, 3..20, 1,
                ) { s = s.copy(smallDistanceCm = it) }
                SettingSlider(
                    "FAST threshold", "%.2f m/s".format(s.fastSpeedCmS / 100f),
                    "Hand faster than this = FAST flick. See your measured hand speed on the gesture panel.",
                    s.fastSpeedCmS, 30..200, 10,
                ) { s = s.copy(fastSpeedCmS = it) }
            }
        },
        confirmButton = {
            TextButton(onClick = { onSave(s) }) { Text("Save") }
        },
        dismissButton = {
            Row {
                TextButton(onClick = { s = FlightSettings() }) { Text("Defaults") }
                TextButton(onClick = onDismiss) { Text("Cancel") }
            }
        },
    )
}

@Composable
private fun SettingSlider(
    title: String,
    valueText: String,
    hint: String,
    value: Int,
    range: IntRange,
    step: Int,
    onChange: (Int) -> Unit,
) {
    Column(Modifier.fillMaxWidth()) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(title, fontWeight = FontWeight.SemiBold)
            Text(valueText, color = MaterialTheme.colorScheme.primary, fontWeight = FontWeight.SemiBold)
        }
        Slider(
            value = value.toFloat(),
            onValueChange = { v ->
                val snapped = ((v - range.first) / step).roundToInt() * step + range.first
                onChange(snapped.coerceIn(range))
            },
            valueRange = range.first.toFloat()..range.last.toFloat(),
            steps = (range.last - range.first) / step - 1,
        )
        if (hint.isNotEmpty()) {
            Text(hint, fontSize = 12.sp, color = MaterialTheme.colorScheme.onSurfaceVariant)
        }
    }
}
