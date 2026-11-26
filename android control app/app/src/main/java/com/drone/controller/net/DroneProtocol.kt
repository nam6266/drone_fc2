package com.drone.controller.net

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * UDP protocol – MUST match the firmware (Protocol.h, config.h).
 * Little-endian, no padding.
 */
object DroneProtocol {
    const val WIFI_SSID = "XIAO-Drone"
    const val WIFI_PASS = "drone1234"
    const val DRONE_IP = "192.168.4.1"
    const val DRONE_PORT = 4210

    const val CMD_MAGIC = 0xA5
    const val TEL_MAGIC = 0x5A
    const val FLAG_ARM = 0x01
    const val FLAG_STOP = 0x02
    const val FLAG_MOTOR_TEST = 0x04            // ground motor test (no IMU / arming needed)
    const val TEST_MASK_SHIFT = 4               // bits 4..7 = motors M1..M4 to spin in the test
    const val TEL_FLAG_LOW_BAT = 0x01
    const val TEL_FLAG_IMU_MISSING = 0x02      // MPU6500 not found (drone refuses ARM)
    const val TEL_FLAG_IMU_CALIBRATING = 0x04  // gyro calibrating (drone refuses ARM)
    const val TEL_FLAG_MOTOR_TEST = 0x08       // drone confirms the motor test is running

    const val CMD_SIZE = 13
    const val TEL_SIZE = 14
    const val TEL_TEMP_UNKNOWN: Short = Short.MIN_VALUE  // firmware has not read the temperature yet

    /** Writes [cmd] into [out] (13 bytes). Returns [out]. */
    fun encodeCommand(cmd: Command, seq: Int, out: ByteArray = ByteArray(CMD_SIZE)): ByteArray {
        require(out.size >= CMD_SIZE)
        var flags = 0
        if (cmd.arm) flags = flags or FLAG_ARM
        if (cmd.stop) flags = flags or FLAG_STOP
        if ((cmd.testMask and 0x0F) != 0) {
            flags = flags or FLAG_MOTOR_TEST or ((cmd.testMask and 0x0F) shl TEST_MASK_SHIFT)
        }
        ByteBuffer.wrap(out).order(ByteOrder.LITTLE_ENDIAN).apply {
            put(CMD_MAGIC.toByte())
            put(flags.toByte())
            putShort(seq.toShort())
            putShort(cmd.roll.coerceIn(-1000, 1000).toShort())
            putShort(cmd.pitch.coerceIn(-1000, 1000).toShort())
            putShort(cmd.yaw.coerceIn(-1000, 1000).toShort())
            putShort(cmd.throttle.coerceIn(0, 1000).toShort())
        }
        out[CMD_SIZE - 1] = xor(out, CMD_SIZE - 1)
        return out
    }

    /** Decodes a 14-byte telemetry packet; null on wrong size / magic / checksum. */
    fun decodeTelemetry(data: ByteArray, length: Int): Telemetry? {
        if (length != TEL_SIZE) return null
        if ((data[0].toInt() and 0xFF) != TEL_MAGIC) return null
        if (xor(data, TEL_SIZE - 1) != data[TEL_SIZE - 1]) return null
        val b = ByteBuffer.wrap(data, 0, length).order(ByteOrder.LITTLE_ENDIAN)
        b.get() // magic
        val state = b.get().toInt() and 0xFF
        val seq = b.short.toInt() and 0xFFFF
        val roll = b.short / 10f
        val pitch = b.short / 10f
        val vbat = (b.short.toInt() and 0xFFFF) / 1000f
        val flags = b.get().toInt() and 0xFF
        val temp10 = b.short
        return Telemetry(
            state = DroneState.fromCode(state),
            seq = seq,
            rollDeg = roll,
            pitchDeg = pitch,
            batteryVolts = vbat,
            lowBattery = (flags and TEL_FLAG_LOW_BAT) != 0,
            imuMissing = (flags and TEL_FLAG_IMU_MISSING) != 0,
            imuCalibrating = (flags and TEL_FLAG_IMU_CALIBRATING) != 0,
            motorTest = (flags and TEL_FLAG_MOTOR_TEST) != 0,
            chipTempC = if (temp10 == TEL_TEMP_UNKNOWN) null else temp10 / 10f,
        )
    }

    fun xor(data: ByteArray, count: Int): Byte {
        var s = 0
        for (i in 0 until count) s = s xor data[i].toInt()
        return s.toByte()
    }
}

/** Command sent to the drone. Sticks: -1000..1000, throttle (power): 0..1000. */
data class Command(
    val arm: Boolean = false,
    val stop: Boolean = false,
    val roll: Int = 0,      // + = move right
    val pitch: Int = 0,     // + = move forward
    val yaw: Int = 0,       // + = turn right
    val throttle: Int = 0,
    /** Motor test: bit0..3 = M1..M4 to spin at [throttle] power. 0 = normal command. */
    val testMask: Int = 0,
)

enum class DroneState(val label: String) {
    DISARMED("DISARMED"),
    ARMED("ARMED"),
    FAILSAFE("FAILSAFE"),
    ERROR("ERROR"),
    UNKNOWN("?");

    companion object {
        fun fromCode(code: Int) = when (code) {
            0 -> DISARMED
            1 -> ARMED
            2 -> FAILSAFE
            3 -> ERROR
            else -> UNKNOWN
        }
    }
}

data class Telemetry(
    val state: DroneState,
    val seq: Int,
    val rollDeg: Float,
    val pitchDeg: Float,
    val batteryVolts: Float,
    val lowBattery: Boolean,
    val chipTempC: Float?,   // ESP32 chip temperature (°C), null = unknown
    val imuMissing: Boolean = false,      // MPU6500 not found
    val imuCalibrating: Boolean = false,  // gyro calibrating, keep the drone still
    val motorTest: Boolean = false,       // motor test running on the drone
) {
    /** The drone only accepts ARM when its motion sensor is found and calibrated. */
    val imuReady: Boolean
        get() = !imuMissing && !imuCalibrating

    /** Rough 1S LiPo % (linear 3.3 V..4.2 V – approximate, voltage sags under load). */
    val batteryPercent: Int
        get() = (((batteryVolts - 3.3f) / (4.2f - 3.3f)) * 100f).toInt().coerceIn(0, 100)
}
