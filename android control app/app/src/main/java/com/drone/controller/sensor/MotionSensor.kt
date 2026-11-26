package com.drone.controller.sensor

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.view.Surface

/**
 * Reads the hand's horizontal acceleration (see [HorizontalFrame]) and reports it to [listener].
 *
 * Uses two Android "virtual" sensors (fused by the OS from accelerometer + gyroscope):
 *   TYPE_GRAVITY             – gravity direction (which way is vertical)
 *   TYPE_LINEAR_ACCELERATION – acceleration with gravity removed
 * Only active in Gesture mode (saves battery).
 */
class MotionSensor(context: Context) : SensorEventListener {

    fun interface Listener {
        fun onHorizontalAccel(right: Float, forward: Float, timeMs: Long)
    }

    private val sm = context.getSystemService(SensorManager::class.java)
    private val gravitySensor: Sensor? = sm.getDefaultSensor(Sensor.TYPE_GRAVITY)
    private val linearSensor: Sensor? = sm.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)

    val available: Boolean get() = gravitySensor != null && linearSensor != null

    /** Screen Surface.ROTATION_*, updated by the Activity on every rotation. */
    @Volatile var displayRotation: Int = Surface.ROTATION_90

    var listener: Listener? = null

    private val gravity = FloatArray(3)
    private var hasGravity = false
    private var running = false

    fun start() {
        if (running || !available) return
        sm.registerListener(this, gravitySensor, SAMPLING_US)
        sm.registerListener(this, linearSensor, SAMPLING_US)
        running = true
    }

    fun stop() {
        sm.unregisterListener(this)
        running = false
        hasGravity = false
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_GRAVITY -> {
                event.values.copyInto(gravity, endIndex = 3)
                hasGravity = true
            }
            Sensor.TYPE_LINEAR_ACCELERATION -> if (hasGravity) {
                HorizontalFrame.toHorizontal(event.values, gravity, displayRotation)?.let {
                    listener?.onHorizontalAccel(it.right, it.forward, event.timestamp / NANOS_PER_MS)
                }
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    private companion object {
        const val SAMPLING_US = 10_000 // 100 Hz
        const val NANOS_PER_MS = 1_000_000L
    }
}
