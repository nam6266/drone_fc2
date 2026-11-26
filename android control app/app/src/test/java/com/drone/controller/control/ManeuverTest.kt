package com.drone.controller.control

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test
import kotlin.math.PI
import kotlin.math.tan

class ManeuverTest {

    private val config = ManeuverConfig()

    @Test
    fun bigLeft_tiltsLeftThenBrakesRight_thenStops() {
        val m = Maneuver.plan(Gesture(GestureDirection.LEFT, GestureSize.BIG, 1f), config, nowMs = 0)
        assertEquals(Pair(-500, 0), m.sticks(10))               // tilt left at 50 % stick
        assertEquals(Pair(500, 0), m.sticks(m.phaseMs + 10))     // brake: tilt right
        assertNull(m.sticks(m.totalMs))                          // finished
    }

    @Test
    fun forward_usesPitch() {
        val m = Maneuver.plan(Gesture(GestureDirection.FORWARD, GestureSize.SMALL, 0.3f), config, nowMs = 0)
        assertEquals(Pair(0, 300), m.sticks(1))
    }

    /** Ideal point-mass simulation: the travelled distance must equal the configured cm. */
    @Test
    fun idealDistance_matchesTarget() {
        for (size in GestureSize.values()) {
            val m = Maneuver.plan(Gesture(GestureDirection.RIGHT, size, 1f), config, nowMs = 0)
            var v = 0.0
            var x = 0.0
            val dtMs = 1L
            var t = 0L
            while (true) {
                val s = m.sticks(t) ?: break
                val angle = s.first / 1000.0 * config.maxAngleDeg * PI / 180
                v += 9.81 * tan(angle) * dtMs / 1000.0
                x += v * dtMs / 1000.0
                t += dtMs
            }
            val target = if (size == GestureSize.BIG) config.bigDistanceCm else config.smallDistanceCm
            assertEquals("size=$size", target.toDouble(), x * 100, 0.5)  // error < 0.5 cm
            assertEquals(0.0, v, 0.01)                                   // fully stopped after braking
        }
    }
}
