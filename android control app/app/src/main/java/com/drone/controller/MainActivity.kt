package com.drone.controller

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.Surface
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import com.drone.controller.ui.ControlScreen
import com.drone.controller.ui.DroneTheme

class MainActivity : ComponentActivity() {

    private val vm: DroneViewModel by viewModels()

    // Permission the system needs to find the drone's WiFi
    private val wifiPermission =
        if (Build.VERSION.SDK_INT >= 33) Manifest.permission.NEARBY_WIFI_DEVICES
        else Manifest.permission.ACCESS_FINE_LOCATION

    // Try to connect whether or not it is granted (many phones do not require it)
    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { vm.onStart() }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)   // keep the screen on while flying
        setContent {
            DroneTheme { ControlScreen(vm) }
        }
    }

    override fun onStart() {
        super.onStart()
        // Landscape has two orientations (rotated left / right): tell the gesture sensor
        // so "right" always means right on screen. Rotating recreates the Activity -> onStart again.
        vm.setDisplayRotation(currentRotation())
        if (checkSelfPermission(wifiPermission) == PackageManager.PERMISSION_GRANTED) vm.onStart()
        else permissionLauncher.launch(wifiPermission)
    }

    override fun onStop() {
        vm.onStop()
        super.onStop()
    }

    private fun currentRotation(): Int =
        if (Build.VERSION.SDK_INT >= 30) {
            display?.rotation ?: Surface.ROTATION_90
        } else {
            @Suppress("DEPRECATION")
            windowManager.defaultDisplay.rotation
        }
}
