package com.drone.controller.ui

import androidx.compose.animation.core.FastOutSlowInEasing
import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.tween
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.DpOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.drone.controller.ControlMode
import kotlin.math.cos
import kotlin.math.sin

/** Size of the round "Mode" button (the top bar reserves this much space for it). */
val ModeButtonSize: Dp = 56.dp

/**
 * Round "Mode" button. Tapping it plays an animation that fans out one round button per mode
 * (Joystick, Gesture) on an arc below/right of it. Choosing a mode – or tapping "Mode" again – closes it.
 *
 * On Android 12+ a blurred, alpha-thresholded copy of the circles is drawn underneath
 * (see [rememberGooeyRenderEffect]), so the buttons stretch out of the main button like liquid
 * drops. On older Android the same movement runs without the liquid look.
 *
 * Place it at the top-left corner; the box is larger than the button to make room for the open menu.
 */
@Composable
fun ModeMenu(current: ControlMode, onSelect: (ControlMode) -> Unit, modifier: Modifier = Modifier) {
    var expanded by rememberSaveable { mutableStateOf(false) }
    val progress by animateFloatAsState(
        targetValue = if (expanded) 1f else 0f,
        animationSpec = tween(durationMillis = ANIMATION_MS, easing = LinearEasing),
        label = "modeMenu",
    )
    val gooey = rememberGooeyRenderEffect()
    val modes = ControlMode.values()
    val offsets = itemOffsets(modes.size)

    // Animation timeline (progress 0..1):
    val spread = FastOutSlowInEasing.transform(0f, 0.8f, progress)  // items fly out
    val labelAlpha = LinearEasing.transform(0.3f, 0.8f, progress)   // item labels fade in
    val swap = FastOutSlowInEasing.transform(0.35f, 0.65f, progress) // "Mode" -> "✕"
    val open = progress > OPEN_THRESHOLD

    val blobColor = MaterialTheme.colorScheme.primary

    Box(modifier.size(MENU_AREA)) {
        // ---- Liquid layer: plain circles only, no text, no touch (Android 12+) ----
        if (gooey != null) {
            Box(Modifier.matchParentSize().graphicsLayer { renderEffect = gooey }) {
                offsets.forEach { target ->
                    Box(
                        Modifier
                            .offset(target.x * spread, target.y * spread)
                            .size(ITEM_SIZE)
                            .background(blobColor, CircleShape)
                    )
                }
                Box(Modifier.size(ModeButtonSize).background(blobColor, CircleShape))
            }
        }

        // ---- Crisp layer: labels + clicks ----
        modes.forEachIndexed { index, mode ->
            val target = offsets[index]
            val selected = mode == current
            Box(
                Modifier
                    .offset(target.x * spread, target.y * spread)
                    .size(ITEM_SIZE)
                    .clip(CircleShape)
                    .background(blobColor)
                    .then(if (selected) Modifier.border(3.dp, Color.White, CircleShape) else Modifier)
                    .clickable(enabled = open) {
                        onSelect(mode)
                        expanded = false
                    },
                contentAlignment = Alignment.Center,
            ) {
                Text(
                    if (selected) "✓\n${mode.label}" else mode.label,
                    color = Color.White,
                    fontSize = 11.sp,
                    fontWeight = FontWeight.Bold,
                    textAlign = TextAlign.Center,
                    lineHeight = 12.sp,
                    modifier = Modifier.alpha(labelAlpha),
                )
            }
        }

        // Main button on top. Transparent over the liquid blob, solid when there is no blob.
        Box(
            Modifier
                .size(ModeButtonSize)
                .clip(CircleShape)
                .background(if (gooey != null) Color.Transparent else blobColor)
                .clickable { expanded = !expanded },
            contentAlignment = Alignment.Center,
        ) {
            Column(Modifier.alpha(1f - swap), horizontalAlignment = Alignment.CenterHorizontally) {
                Text("MODE", color = Color.White, fontSize = 12.sp, fontWeight = FontWeight.Black)
                Text(current.label, color = Color.White.copy(alpha = 0.8f), fontSize = 9.sp)
            }
            Text(
                "✕",
                color = Color.White,
                fontSize = 22.sp,
                fontWeight = FontWeight.Bold,
                modifier = Modifier.alpha(swap).rotate(90f * swap),
            )
        }
    }
}

/**
 * Item positions (top-left of each item, relative to the main button's top-left),
 * spread evenly on a quarter arc from "right" to "down".
 */
private fun itemOffsets(count: Int): List<DpOffset> {
    val centerShift = (ModeButtonSize - ITEM_SIZE) / 2
    return List(count) { i ->
        val t = if (count == 1) 0.5f else i / (count - 1f)
        val angle = Math.toRadians((ARC_START_DEG + (ARC_END_DEG - ARC_START_DEG) * t).toDouble())
        DpOffset(
            centerShift + ARC_RADIUS * cos(angle).toFloat(),
            centerShift + ARC_RADIUS * sin(angle).toFloat(), // Compose y grows downward
        )
    }
}

private const val ANIMATION_MS = 650
private const val OPEN_THRESHOLD = 0.9f
private const val ARC_START_DEG = 10.0   // first item: almost straight to the right
private const val ARC_END_DEG = 80.0     // last item: almost straight down
private val ITEM_SIZE = 60.dp
private val ARC_RADIUS = 92.dp
private val MENU_AREA = 170.dp           // room for the open menu (radius + item size + margin)
