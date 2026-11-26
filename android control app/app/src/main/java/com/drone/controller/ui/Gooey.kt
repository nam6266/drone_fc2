package com.drone.controller.ui

import android.graphics.ColorMatrixColorFilter
import android.graphics.Shader
import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.animation.core.Easing
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.graphics.RenderEffect
import androidx.compose.ui.graphics.asComposeRenderEffect
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp

/**
 * Maps [value] from the window [from]..[to] to 0..1 (clamped), then applies this easing.
 * Lets several parts of one animation start and finish at different moments.
 */
fun Easing.transform(from: Float, to: Float, value: Float): Float =
    transform(((value - from) / (to - from)).coerceIn(0f, 1f))

/**
 * "Gooey" (metaball) effect: shapes drawn in a layer with this effect melt together like liquid
 * when they are close and split apart when they move away.
 *   1. blur          – overlapping shapes add their soft edges together
 *   2. alpha cut-off – alpha' = 50·alpha − 5000 (0..255 scale): hard edge at alpha ≈ 100
 * Returns null below Android 12 (API 31), where RenderEffect does not exist: callers then
 * simply skip the liquid layer and keep the rest of the animation.
 */
@Composable
fun rememberGooeyRenderEffect(blurRadius: Dp = 18.dp): RenderEffect? {
    if (Build.VERSION.SDK_INT < Build.VERSION_CODES.S) return null
    val radiusPx = with(LocalDensity.current) { blurRadius.toPx() }
    return remember(radiusPx) { createGooeyEffect(radiusPx) }
}

@RequiresApi(Build.VERSION_CODES.S)
private fun createGooeyEffect(radiusPx: Float): RenderEffect {
    // DECAL: outside the layer counts as transparent -> no smeared edges
    val blur = android.graphics.RenderEffect.createBlurEffect(radiusPx, radiusPx, Shader.TileMode.DECAL)
    val threshold = android.graphics.RenderEffect.createColorFilterEffect(
        ColorMatrixColorFilter(
            android.graphics.ColorMatrix(
                floatArrayOf(
                    1f, 0f, 0f, 0f, 0f,
                    0f, 1f, 0f, 0f, 0f,
                    0f, 0f, 1f, 0f, 0f,
                    0f, 0f, 0f, ALPHA_SCALE, ALPHA_OFFSET,
                )
            )
        )
    )
    // createChainEffect(outer, inner): blur first, then the alpha cut-off
    return android.graphics.RenderEffect.createChainEffect(threshold, blur).asComposeRenderEffect()
}

private const val ALPHA_SCALE = 50f
private const val ALPHA_OFFSET = -5000f
