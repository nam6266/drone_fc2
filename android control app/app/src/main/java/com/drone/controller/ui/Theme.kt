package com.drone.controller.ui

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

val ArmGreen = Color(0xFF2E9E5B)
val StopRed = Color(0xFFD32F2F)
val WarnAmber = Color(0xFFF2A93B)

private val Colors = darkColorScheme(
    primary = Color(0xFF4FC3F7),
    onPrimary = Color(0xFF00222E),
    background = Color(0xFF101214),
    surface = Color(0xFF181B1F),
    surfaceVariant = Color(0xFF23272D),
    onSurfaceVariant = Color(0xFFB8C0C8),
    outline = Color(0xFF55606B),
)

@Composable
fun DroneTheme(content: @Composable () -> Unit) {
    MaterialTheme(colorScheme = Colors, content = content)
}
