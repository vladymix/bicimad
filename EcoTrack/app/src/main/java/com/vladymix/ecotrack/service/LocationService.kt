package com.vladymix.ecotrack.service

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.content.Context
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import androidx.activity.result.ActivityResultLauncher
import androidx.core.app.ActivityCompat


class LocationService(private val activity: Activity) {

    private val oldLocation = Location("False")
    var listener: Result? = null


    inner class MyLocationListener : LocationListener {
        override fun onLocationChanged(p0: Location) {
            listener?.updateLocation(oldLocation, p0)

        }
    }

    companion object {
        private const val MIN_TIME_BETWEEN_UPDATES: Long = 5000 // 5 segundos
        private const val MIN_DISTANCE_CHANGE_FOR_UPDATES = 3.0f // 10 metros
    }

    interface Result {
        fun permissionsGranted()
        fun permissionsDenied()
        fun updateLocation(lastLocation: Location?, currentLocation: Location)
    }


    lateinit var locationPermissionRequest: ActivityResultLauncher<Array<String>>

    private val locationListener = MyLocationListener()

    fun hasPermissions(): Boolean {
        return (ActivityCompat.checkSelfPermission(
            this.activity,
            Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED)
    }

    fun startService() {
        if (hasPermissions()) {
            startListener()
        } else {

/*            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                ActivityCompat.requestPermissions(
                    this.activity,
                    arrayOf(
                        Manifest.permission.ACCESS_BACKGROUND_LOCATION
                    ),
                    1024
                )
            }*/
            locationPermissionRequest.launch(
                arrayOf(
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_COARSE_LOCATION
                )
            )
        }


    }

    @SuppressLint("MissingPermission")
    fun managerResultPermissions(permissions: Map<String, Boolean>) {
        val fineLocation = permissions[Manifest.permission.ACCESS_FINE_LOCATION] ?: false
        // Only approximate location access granted.
        val coarseLocation = permissions[Manifest.permission.ACCESS_COARSE_LOCATION] ?: false

        if (fineLocation || coarseLocation) {
            listener?.permissionsGranted()
            startListener()

        } else {
            listener?.permissionsDenied()
        }
    }

    @SuppressLint("MissingPermission")
    fun startListener() {
        val locationManager = this.activity.getSystemService(Context.LOCATION_SERVICE) as LocationManager
        locationManager.requestLocationUpdates(
            LocationManager.GPS_PROVIDER,
            MIN_TIME_BETWEEN_UPDATES,
            MIN_DISTANCE_CHANGE_FOR_UPDATES,
            locationListener
        )
    }


    fun stopService() {
        try {
            val locationManager = this.activity.getSystemService(Context.LOCATION_SERVICE) as LocationManager
            locationManager.removeUpdates(locationListener)
        } catch (ex: Exception) {
            ex.printStackTrace()
        }
    }
}