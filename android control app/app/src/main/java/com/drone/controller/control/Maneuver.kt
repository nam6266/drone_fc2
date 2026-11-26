package com.drone.controller.control

import kotlin.math.roundToInt
import kotlin.math.sqrt
import kotlin.math.tan

data class ManeuverConfig(
    val bigDistanceCm: Float = 30f,     // FAST flick
    val smallDistanceCm: Float = 8f,    // SLOW flick
    val bigTiltFraction: Float = 0.5f,  // stick fraction: 0.5 × 20° = 10° tilt
    val smallTiltFraction: Float = 0.3f, // 0.3 × 20° = 6° tilt
    // MUST match cfg::MAX_ANGLE_DEG in the firmware (tilt at full stick)
    val maxAngleDeg: Float = 20f,
)

/**
 * One "move a short distance" maneuver – open loop, there is NO position sensor:
 *   phase 1: tilt toward the target direction for [phaseMs]  -> drone accelerates
 *   phase 2: tilt the opposite way for [phaseMs]              -> drone brakes
 *   then:    sticks back to 0
 * Real distance varies a lot (battery, drafts, drone lag) -> tune the cm values in Settings.
 */
class Maneuver(
    val direction: GestureDirection,
    val distanceCm: Float,
    val tiltFraction: Float,
    val phaseMs: Long,
    val startMs: Long,
) {
    val totalMs: Long get() = 2 * phaseMs

    fun isDone(nowMs: Long): Boolean = nowMs - startMs >= totalMs

    /** Roll/pitch sticks (-1000..1000) at [nowMs], null when finished. */
    fun sticks(nowMs: Long): Pair<Int, Int>? {
        val t = nowMs - startMs
        val phaseSign = when {
            t < 0 -> 0
            t < phaseMs -> 1   // accelerate
            t < totalMs -> -1  // brake
            else -> return null
        }
        val amount = (tiltFraction * STICK_MAX * phaseSign).roundToInt()
        return Pair(direction.roll * amount, direction.pitch * amount)
    }

    companion object {
        const val STICK_MAX = 1000f
        private const val G = 9.81f
        private const val DEG_TO_RAD = (Math.PI / 180.0).toFloat()

        /**
         * Tilt θ -> horizontal acceleration a = g·tanθ. Accelerate for t seconds, then brake for t seconds:
         *   distance d = ½·a·t² + ½·a·t² = a·t²   ->   t = √(d / a)
         */
        fun plan(gesture: Gesture, config: ManeuverConfig, nowMs: Long): Maneuver {
            val big = gesture.size == GestureSize.BIG
            val distanceCm = if (big) config.bigDistanceCm else config.smallDistanceCm
            val fraction = if (big) config.bigTiltFraction else config.smallTiltFraction
            val accel = G * tan(fraction * config.maxAngleDeg * DEG_TO_RAD)
            val seconds = sqrt((distanceCm / 100f) / accel)
            return Maneuver(gesture.direction, distanceCm, fraction, (seconds * 1000f).roundToInt().toLong(), nowMs)
        }
    }
}
