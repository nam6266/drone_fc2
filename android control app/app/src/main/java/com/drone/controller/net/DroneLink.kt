package com.drone.controller.net

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.net.wifi.WifiNetworkSpecifier
import android.os.SystemClock
import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetSocketAddress
import java.net.SocketTimeoutException

enum class WifiStatus(val label: String) {
    IDLE("Not connected"),
    REQUESTING("Requesting WiFi ${DroneProtocol.WIFI_SSID}…"),
    CONNECTED("Connected to ${DroneProtocol.WIFI_SSID}"),
    UNAVAILABLE("Cannot connect (rejected or drone not found)"),
    LOST("Drone WiFi lost – waiting to reconnect"),
    ERROR("Socket error"),
}

/**
 * Connection to the drone:
 *  1. Ask Android to join the XIAO WiFi (WifiNetworkSpecifier – the system shows a dialog).
 *  2. Open one DatagramSocket bound to that network (the drone network has no Internet;
 *     without binding, Android may route packets over mobile data).
 *  3. Send Command 50 times/s, receive Telemetry on the same socket.
 *
 * [commandProvider] is called on the IO thread every 20 ms to get the latest command.
 */
class DroneLink(
    context: Context,
    private val scope: CoroutineScope,
    private val commandProvider: () -> Command,
) {
    private val cm = context.getSystemService(ConnectivityManager::class.java)

    private val _wifiStatus = MutableStateFlow(WifiStatus.IDLE)
    val wifiStatus: StateFlow<WifiStatus> = _wifiStatus.asStateFlow()

    private val _telemetry = MutableStateFlow<Telemetry?>(null)
    val telemetry: StateFlow<Telemetry?> = _telemetry.asStateFlow()

    /** Time (elapsedRealtime, ms) of the latest telemetry. */
    @Volatile var lastTelemetryAt = 0L
        private set

    private var callback: ConnectivityManager.NetworkCallback? = null
    @Volatile private var socketJob: Job? = null
    @Volatile private var socket: DatagramSocket? = null

    fun start() {
        if (callback != null) return
        val specifier = WifiNetworkSpecifier.Builder()
            .setSsid(DroneProtocol.WIFI_SSID)
            .setWpa2Passphrase(DroneProtocol.WIFI_PASS)
            .build()
        val request = NetworkRequest.Builder()
            .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
            .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
            .setNetworkSpecifier(specifier)
            .build()

        val cb = object : ConnectivityManager.NetworkCallback() {
            override fun onAvailable(network: Network) {
                _wifiStatus.value = WifiStatus.CONNECTED
                openSocket(network)
            }

            override fun onLost(network: Network) {
                _wifiStatus.value = WifiStatus.LOST
                closeSocket()
            }

            override fun onUnavailable() {
                // User pressed Cancel or the network was not found. The request is already released.
                _wifiStatus.value = WifiStatus.UNAVAILABLE
                callback = null
            }
        }
        callback = cb
        _wifiStatus.value = WifiStatus.REQUESTING
        cm.requestNetwork(request, cb)
    }

    fun stop() {
        closeSocket()
        callback?.let { runCatching { cm.unregisterNetworkCallback(it) } }
        callback = null
        _wifiStatus.value = WifiStatus.IDLE
    }

    private fun openSocket(network: Network) {
        closeSocket()
        socketJob = scope.launch(Dispatchers.IO) {
            val s = try {
                DatagramSocket().also {
                    network.bindSocket(it)
                    it.soTimeout = 200
                }
            } catch (e: IOException) {
                Log.e(TAG, "Cannot open socket", e)
                _wifiStatus.value = WifiStatus.ERROR
                return@launch
            }
            socket = s
            val target = InetSocketAddress(DroneProtocol.DRONE_IP, DroneProtocol.DRONE_PORT)

            // Telemetry receive loop
            launch {
                val buf = ByteArray(64)
                val packet = DatagramPacket(buf, buf.size)
                while (isActive && !s.isClosed) {
                    try {
                        packet.length = buf.size
                        s.receive(packet)
                        DroneProtocol.decodeTelemetry(buf, packet.length)?.let {
                            lastTelemetryAt = SystemClock.elapsedRealtime()
                            _telemetry.value = it
                        }
                    } catch (e: SocketTimeoutException) {
                        // normal: no packet yet, loop to re-check isActive
                    } catch (e: IOException) {
                        if (s.isClosed) break
                    }
                }
            }

            // 50 Hz command loop with steady pacing
            val buf = ByteArray(DroneProtocol.CMD_SIZE)
            val packet = DatagramPacket(buf, buf.size, target)
            var seq = 0
            var next = SystemClock.elapsedRealtime()
            while (isActive && !s.isClosed) {
                DroneProtocol.encodeCommand(commandProvider(), seq++, buf)
                try {
                    s.send(packet)
                } catch (e: IOException) {
                    if (s.isClosed) break   // WiFi is changing state: skip this packet
                }
                next += SEND_PERIOD_MS
                val wait = next - SystemClock.elapsedRealtime()
                if (wait > 0) delay(wait) else next = SystemClock.elapsedRealtime()
            }
        }
    }

    private fun closeSocket() {
        socketJob?.cancel()
        socketJob = null
        socket?.close()
        socket = null
    }

    companion object {
        private const val TAG = "DroneLink"
        const val SEND_PERIOD_MS = 20L   // 50 Hz
    }
}
