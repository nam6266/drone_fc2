package com.drone.controller.sensor

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Test
import kotlin.math.cos
import kotlin.math.sin

/**
 * Landscape ROTATION_90: the phone's top edge is rotated to the LEFT.
 *   phone x axis points UP (when held upright), screen-right = phone -y.
 */
class HorizontalFrameTest {

    private val rot90 = 1
    private val eps = 0.01f

    private fun check(linear: FloatArray, gravity: FloatArray, right: Float, forward: Float, rotation: Int = rot90) {
        val a = HorizontalFrame.toHorizontal(linear, gravity, rotation)
        assertNotNull(a)
        assertEquals("right", right, a!!.right, eps)
        assertEquals("forward", forward, a.forward, eps)
    }

    @Test
    fun upright_facingUser() {
        val g = floatArrayOf(9.81f, 0f, 0f)                  // upright, screen facing the user
        check(floatArrayOf(0f, -2f, 0f), g, right = 2f, forward = 0f)   // push toward screen-right
        check(floatArrayOf(0f, 0f, -2f), g, right = 0f, forward = 2f)   // push away (out of the back)
        check(floatArrayOf(2f, 0f, 0f), g, right = 0f, forward = 0f)    // lift up: ignored
    }

    @Test
    fun flat_screenUp() {
        val g = floatArrayOf(0f, 0f, 9.81f)                  // lying flat, screen up
        check(floatArrayOf(0f, -2f, 0f), g, right = 2f, forward = 0f)
        check(floatArrayOf(2f, 0f, 0f), g, right = 0f, forward = 2f)    // screen top edge = away
        check(floatArrayOf(0f, 0f, 2f), g, right = 0f, forward = 0f)    // lift up: ignored
    }

    @Test
    fun tilted45deg() {
        val c = cos(Math.toRadians(45.0)).toFloat()
        val s = sin(Math.toRadians(45.0)).toFloat()
        val g = floatArrayOf(9.81f * s, 0f, 9.81f * c)       // tilted 45° toward the user
        // Horizontal 'away' = (cos45 on phone x, -sin45 on phone z)
        check(floatArrayOf(2f * c, 0f, -2f * s), g, right = 0f, forward = 2f)
    }

    @Test
    fun rotation270_isMirrored() {
        val g = floatArrayOf(-9.81f, 0f, 0f)                 // top edge rotated to the RIGHT
        check(floatArrayOf(0f, 2f, 0f), g, right = 2f, forward = 0f, rotation = 3)
        check(floatArrayOf(0f, 0f, -2f), g, right = 0f, forward = 2f, rotation = 3)
    }
}
