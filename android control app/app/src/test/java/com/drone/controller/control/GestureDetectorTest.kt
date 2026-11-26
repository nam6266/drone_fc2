package com.drone.controller.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.sin

/**
 * The hand moves with a "hill" speed profile: v(t) = V·sin²(πt/T), 0 ≤ t ≤ T
 *  -> acceleration a(t) = V·π/T·sin(2πt/T): first half pushes, second half brakes (opposite sign).
 */
class GestureDetectorTest {

    private val stepMs = 10L

    /** Acceleration of one hand stroke, sampled every 10 ms, followed by [quietMs] of stillness. */
    private fun stroke(peakSpeed: Float, durationS: Float, quietMs: Long = 800): List<Float> {
        val samples = mutableListOf<Float>()
        val n = (durationS * 1000 / stepMs).toInt()
        for (i in 0..n) {
            val t = i * stepMs / 1000f
            samples += (peakSpeed * PI / durationS * sin(2 * PI * t / durationS)).toFloat()
        }
        repeat((quietMs / stepMs).toInt()) { samples += 0f }
        return samples
    }

    private fun run(
        detector: GestureDetector,
        right: List<Float>,
        forward: List<Float> = List(right.size) { 0f },
        startMs: Long = 1000,
    ): List<Gesture> {
        val found = mutableListOf<Gesture>()
        for (i in right.indices) {
            detector.update(right[i], forward[i], startMs + i * stepMs)?.let { found += it }
        }
        return found
    }

    @Test
    fun fastLeft_isOneBigLeft_andBrakeIsIgnored() {
        val g = run(GestureDetector(), stroke(1.2f, 0.3f).map { -it })
        assertEquals(1, g.size) // the opposite braking must NOT become a RIGHT gesture
        assertEquals(GestureDirection.LEFT, g[0].direction)
        assertEquals(GestureSize.BIG, g[0].size)
        assertTrue("peak=${g[0].peakSpeed}", g[0].peakSpeed in 0.9f..1.25f)
    }

    @Test
    fun slowRight_isOneSmallRight() {
        val g = run(GestureDetector(), stroke(0.3f, 0.5f))
        assertEquals(1, g.size)
        assertEquals(GestureDirection.RIGHT, g[0].direction)
        assertEquals(GestureSize.SMALL, g[0].size)
    }

    @Test
    fun forwardAndBack_useForwardAxis() {
        val push = stroke(1.0f, 0.3f)
        val g1 = run(GestureDetector(), List(push.size) { 0f }, push)
        assertEquals(listOf(GestureDirection.FORWARD), g1.map { it.direction })
        val g2 = run(GestureDetector(), List(push.size) { 0f }, push.map { -it })
        assertEquals(listOf(GestureDirection.BACK), g2.map { it.direction })
    }

    @Test
    fun twoSeparateStrokes_areBothDetected() {
        val g = run(GestureDetector(), stroke(1.0f, 0.3f) + stroke(1.0f, 0.3f).map { -it })
        assertEquals(listOf(GestureDirection.RIGHT, GestureDirection.LEFT), g.map { it.direction })
    }

    @Test
    fun handTremor_isIgnored() {
        // small fast hand tremor: 0.6 m/s² amplitude, 8 Hz for 3 seconds
        val tremor = List(300) { i -> (0.6 * sin(2 * PI * 8 * i * stepMs / 1000.0)).toFloat() }
        assertEquals(0, run(GestureDetector(), tremor, tremor.map { it * 0.5f }).size)
    }

    @Test
    fun diagonalStroke_isIgnored() {
        val s = stroke(1.0f, 0.3f)
        assertEquals(0, run(GestureDetector(), s, s).size) // 45°: unclear whether left/right or forward/back
    }

    @Test
    fun tinyMove_belowMinSpeed_isIgnored_butPeakIsReported() {
        val d = GestureDetector()
        // exceeds the acceleration threshold but with tiny speed (short jerk)
        assertEquals(0, run(d, stroke(0.08f, 0.15f)).size)
        assertTrue(d.lastPeakSpeed > 0f && d.lastPeakSpeed < 0.12f)
    }
}
