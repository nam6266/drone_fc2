package com.drone.controller.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.onSizeChanged
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Virtual joystick. Values [x], [y] in -1..1 (y + = UP).
 *
 * - RELATIVE drag: the knob starts from its current position, it does not jump to the finger.
 * - [springX]/[springY] = true: that axis springs back to centre on release.
 * - [circular] = true: knob moves within a circle (steering); false: 2 independent axes (power).
 * - When not touched, the knob follows the external [x], [y] (e.g. power ramping during take-off).
 * - The caller sets a square size (e.g. Modifier.size(...)).
 */
@Composable
fun Joystick(
    x: Float,
    y: Float,
    onMove: (x: Float, y: Float) -> Unit,
    springX: Boolean,
    springY: Boolean,
    circular: Boolean,
    modifier: Modifier = Modifier,
    labelTop: String = "",
    labelBottom: String = "",
    labelLeft: String = "",
    labelRight: String = "",
) {
    var sizePx by remember { mutableFloatStateOf(1f) }
    var knobX by remember { mutableFloatStateOf(x) }
    var knobY by remember { mutableFloatStateOf(y) }
    var dragging by remember { mutableStateOf(false) }
    val onMoveLatest by rememberUpdatedState(onMove)

    // When not touched, the knob follows the external values
    LaunchedEffect(x, y, dragging) {
        if (!dragging) {
            knobX = x
            knobY = y
        }
    }

    val ring = MaterialTheme.colorScheme.outline
    val pad = MaterialTheme.colorScheme.surfaceVariant
    val knobColor = MaterialTheme.colorScheme.primary

    Box(
        modifier
            .onSizeChanged { sizePx = min(it.width, it.height).toFloat().coerceAtLeast(1f) }
            .pointerInput(springX, springY, circular) {
                fun release() {
                    if (springX) knobX = 0f
                    if (springY) knobY = 0f
                    dragging = false
                    onMoveLatest(knobX, knobY)
                }
                detectDragGestures(
                    onDragStart = { dragging = true },
                    onDragEnd = { release() },
                    onDragCancel = { release() },
                ) { change, drag ->
                    change.consume()
                    val travel = sizePx / 2f * TRAVEL
                    var nx = knobX + drag.x / travel
                    var ny = knobY - drag.y / travel
                    if (circular) {
                        val len = sqrt(nx * nx + ny * ny)
                        if (len > 1f) {
                            nx /= len
                            ny /= len
                        }
                    } else {
                        nx = nx.coerceIn(-1f, 1f)
                        ny = ny.coerceIn(-1f, 1f)
                    }
                    knobX = nx
                    knobY = ny
                    onMoveLatest(nx, ny)
                }
            },
        contentAlignment = Alignment.Center,
    ) {
        Canvas(Modifier.fillMaxSize()) {
            val r = size.minDimension / 2f
            val c = center
            val stroke = Stroke(2.dp.toPx())
            if (circular) {
                drawCircle(pad, radius = r)
                drawCircle(ring, radius = r - stroke.width, style = stroke)
            } else {
                val corner = CornerRadius(24.dp.toPx())
                drawRoundRect(pad, cornerRadius = corner)
                drawRoundRect(ring, cornerRadius = corner, style = stroke, size = Size(size.width, size.height))
            }
            drawLine(ring.copy(alpha = 0.4f), Offset(c.x - r, c.y), Offset(c.x + r, c.y))
            drawLine(ring.copy(alpha = 0.4f), Offset(c.x, c.y - r), Offset(c.x, c.y + r))
            val travel = r * TRAVEL
            val knob = Offset(c.x + knobX * travel, c.y - knobY * travel)
            drawCircle(knobColor.copy(alpha = 0.85f), radius = r * KNOB_RATIO, center = knob)
        }
        val labelColor = MaterialTheme.colorScheme.onSurfaceVariant
        Text(labelTop, color = labelColor, fontSize = 11.sp, modifier = Modifier.align(Alignment.TopCenter).padding(6.dp))
        Text(labelBottom, color = labelColor, fontSize = 11.sp, modifier = Modifier.align(Alignment.BottomCenter).padding(6.dp))
        Text(labelLeft, color = labelColor, fontSize = 11.sp, modifier = Modifier.align(Alignment.CenterStart).padding(6.dp))
        Text(labelRight, color = labelColor, fontSize = 11.sp, modifier = Modifier.align(Alignment.CenterEnd).padding(6.dp))
    }
}

private const val TRAVEL = 0.72f     // knob travels at most 72 % of the radius
private const val KNOB_RATIO = 0.2f  // knob radius = 20 % of the joystick radius
