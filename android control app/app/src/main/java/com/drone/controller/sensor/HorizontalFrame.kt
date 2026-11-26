package com.drone.controller.sensor

import kotlin.math.sqrt

/**
 * Converts acceleration from PHONE axes to the world HORIZONTAL plane, relative to the screen.
 * Pure math (no Android dependency) -> unit testable.
 *
 * Phone axes (Android): x to the right, y toward the top edge, z out of the screen (portrait).
 * [rotation]: a Surface.ROTATION_* value (0, 1, 2, 3 = 0°, 90°, 180°, 270°).
 */
object HorizontalFrame {

    data class Accel(val right: Float, val forward: Float)

    private const val MIN_LENGTH = 0.2f

    /**
     * [linear]  : acceleration without gravity (TYPE_LINEAR_ACCELERATION), m/s²
     * [gravity] : gravity vector (TYPE_GRAVITY) – Android reports it pointing UP, lying flat = (0, 0, +9.8)
     * Returns null if the directions cannot be determined.
     */
    fun toHorizontal(linear: FloatArray, gravity: FloatArray, rotation: Int): Accel? {
        val up = normalize(gravity) ?: return null
        val (screenX, screenY) = screenAxes(rotation)
        // "Away from the user" = top edge of the screen + back of the phone (-z):
        //   held upright -> the back is horizontal, pointing away;  held flat -> the top edge points away
        val away = floatArrayOf(screenY[0], screenY[1], screenY[2] - 1f)
        val rightDir = normalize(horizontal(screenX, up)) ?: return null
        val forwardDir = normalize(horizontal(away, up)) ?: return null
        return Accel(dot(linear, rightDir), dot(linear, forwardDir))
    }

    /** SCREEN right (X) and up (Y) axes, expressed in phone axes. */
    fun screenAxes(rotation: Int): Pair<FloatArray, FloatArray> = when (rotation) {
        1 -> floatArrayOf(0f, -1f, 0f) to floatArrayOf(1f, 0f, 0f)   // ROTATION_90
        2 -> floatArrayOf(-1f, 0f, 0f) to floatArrayOf(0f, -1f, 0f)  // ROTATION_180
        3 -> floatArrayOf(0f, 1f, 0f) to floatArrayOf(-1f, 0f, 0f)   // ROTATION_270
        else -> floatArrayOf(1f, 0f, 0f) to floatArrayOf(0f, 1f, 0f) // ROTATION_0
    }

    private fun dot(a: FloatArray, b: FloatArray) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    /** Removes the vertical component (along [up]) of v. */
    private fun horizontal(v: FloatArray, up: FloatArray): FloatArray {
        val k = dot(v, up)
        return floatArrayOf(v[0] - k * up[0], v[1] - k * up[1], v[2] - k * up[2])
    }

    /** null if the vector is too short (undefined direction, e.g. that axis is vertical). */
    private fun normalize(v: FloatArray): FloatArray? {
        val len = sqrt(dot(v, v))
        if (len < MIN_LENGTH) return null
        return floatArrayOf(v[0] / len, v[1] / len, v[2] / len)
    }
}
