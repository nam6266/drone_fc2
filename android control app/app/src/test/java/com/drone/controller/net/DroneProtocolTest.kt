package com.drone.controller.net

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test

/** Expected bytes were generated with Python struct.pack – the same format the firmware reads. */
class DroneProtocolTest {

    private fun hex(s: String) = s.chunked(2).map { it.toInt(16).toByte() }.toByteArray()

    @Test
    fun encodeCommand_matchesFirmwareLayout() {
        val cmd = Command(arm = true, roll = -500, pitch = 250, yaw = -1000, throttle = 300)
        val bytes = DroneProtocol.encodeCommand(cmd, seq = 7)
        assertArrayEquals(hex("a50107000cfefa0018fc2c0162"), bytes)
    }

    @Test
    fun encodeCommand_motorTest_matchesFirmware() {
        // M1 + M3 at 30 % – same bytes as the firmware host test
        val bytes = DroneProtocol.encodeCommand(Command(throttle = 300, testMask = 0b0101), seq = 9)
        assertArrayEquals(hex("a55409000000000000002c01d5"), bytes)
    }

    @Test
    fun decodeTelemetry_motorTestFlag() {
        val t = DroneProtocol.decodeTelemetry(hex("5a002a000000000036100bc40198"), 14)!!
        assertEquals(true, t.motorTest)
        assertEquals(true, t.imuMissing)
        assertEquals(false, t.imuCalibrating)
        assertEquals(4.150f, t.batteryVolts, 0.001f)
    }

    @Test
    fun encodeCommand_clampsValues() {
        val bytes = DroneProtocol.encodeCommand(Command(roll = 5000, throttle = -3), seq = 0)
        assertEquals(1000, (bytes[4].toInt() and 0xFF) or (bytes[5].toInt() shl 8))
        assertEquals(0, (bytes[10].toInt() and 0xFF) or (bytes[11].toInt() shl 8))
    }

    @Test
    fun decodeTelemetry_valid() {
        val t = DroneProtocol.decodeTelemetry(hex("5a012a0085ffc8011e0f01c40117"), 14)
        assertNotNull(t)
        t!!
        assertEquals(DroneState.ARMED, t.state)
        assertEquals(42, t.seq)
        assertEquals(-12.3f, t.rollDeg, 0.001f)
        assertEquals(45.6f, t.pitchDeg, 0.001f)
        assertEquals(3.870f, t.batteryVolts, 0.001f)
        assertEquals(true, t.lowBattery)
        assertEquals(45.2f, t.chipTempC!!, 0.001f)
    }

    @Test
    fun decodeTelemetry_imuFlags() {
        val ready = DroneProtocol.decodeTelemetry(hex("5a012a0085ffc8011e0f01c40117"), 14)!!
        assertEquals(false, ready.imuMissing)
        assertEquals(false, ready.imuCalibrating)
        assertEquals(true, ready.imuReady)

        val notReady = DroneProtocol.decodeTelemetry(hex("5a012a0085ffc8011e0f07c40111"), 14)!!
        assertEquals(true, notReady.lowBattery)
        assertEquals(true, notReady.imuMissing)
        assertEquals(true, notReady.imuCalibrating)
        assertEquals(false, notReady.imuReady)
    }

    @Test
    fun decodeTelemetry_unknownTemperature() {
        val t = DroneProtocol.decodeTelemetry(hex("5a012a0085ffc8011e0f01008052"), 14)
        assertNotNull(t)
        assertNull(t!!.chipTempC)
    }

    @Test
    fun decodeTelemetry_rejectsBadChecksumOrSize() {
        val good = hex("5a012a0085ffc8011e0f01c40117")
        val bad = good.copyOf().also { it[13] = 0 }
        assertNull(DroneProtocol.decodeTelemetry(bad, 14))
        assertNull(DroneProtocol.decodeTelemetry(good, 13))
    }
}
