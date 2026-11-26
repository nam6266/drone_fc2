package com.drone.controller.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.controller.FlightSettings
import com.drone.controller.GestureUi
import com.drone.controller.control.GestureDirection

/** Left panel in Gesture mode: arrow of the latest gesture + measured hand speed. */
@Composable
fun GesturePanel(gesture: GestureUi, settings: FlightSettings, modifier: Modifier = Modifier) {
    val last = gesture.last
    val arrowColor = when {
        gesture.executing -> MaterialTheme.colorScheme.primary
        last != null -> MaterialTheme.colorScheme.onSurfaceVariant
        else -> MaterialTheme.colorScheme.outline
    }
    Column(
        modifier
            .clip(RoundedCornerShape(20.dp))
            .background(MaterialTheme.colorScheme.surfaceVariant)
            .padding(12.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(6.dp, Alignment.CenterVertically),
    ) {
        Text(
            last?.direction?.arrow() ?: "✋",
            fontSize = 64.sp,
            color = arrowColor,
            fontWeight = FontWeight.Bold,
        )
        if (last != null) {
            Text(
                "${last.direction.label} · ${last.size.label} · %.2f m/s".format(last.peakSpeed),
                color = Color.White,
                fontWeight = FontWeight.SemiBold,
            )
        }
        Text(gesture.note, color = if (gesture.executing) ArmGreen else WarnAmber, fontSize = 13.sp, textAlign = TextAlign.Center)
        Text(
            "Slow ≈ ${settings.smallDistanceCm} cm · Fast ≈ ${settings.bigDistanceCm} cm" +
                " (fast when > %.2f m/s)".format(settings.fastSpeedCmS / 100f),
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            fontSize = 12.sp,
            textAlign = TextAlign.Center,
        )
        Text(
            "Push away = FORWARD · Pull toward you = BACK",
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            fontSize = 12.sp,
        )
    }
}

private fun GestureDirection.arrow(): String = when (this) {
    GestureDirection.LEFT -> "←"
    GestureDirection.RIGHT -> "→"
    GestureDirection.FORWARD -> "↑"
    GestureDirection.BACK -> "↓"
}
