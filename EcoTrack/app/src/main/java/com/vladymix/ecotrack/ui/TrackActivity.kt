package com.vladymix.ecotrack.ui

import android.location.Location
import android.os.Bundle
import android.view.View
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.isVisible
import com.vladymix.ecotrack.R
import com.vladymix.ecotrack.service.LocationService
import java.util.Calendar

class TrackActivity : AppCompatActivity() {
    private lateinit var textView: TextView
    private lateinit var locationService: LocationService

    private var viewModel = TrackViewModel()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_track)
        textView = findViewById(R.id.textView)
        locationService = LocationService(this)

        locationService.listener = object : LocationService.Result {
            override fun permissionsGranted() {
                Toast.makeText(baseContext, "Permissions ready", Toast.LENGTH_LONG).show()
            }

            override fun permissionsDenied() {
                Toast.makeText(baseContext, "No has permission", Toast.LENGTH_LONG).show()
            }

            override fun updateLocation(lastLocation: Location?, currentLocation: Location) {
                Toast.makeText(baseContext, "Location changed", Toast.LENGTH_LONG).show()
                val buffer = StringBuffer(textView.text)
                buffer.append("\n")
                var date = ""
                Calendar.getInstance().let {
                    date += (it.get(Calendar.HOUR_OF_DAY))
                    date += (":")
                    date += (it.get(Calendar.MINUTE))
                    date += (":")
                    date += (it.get(Calendar.SECOND))
                }
                buffer.append("$date lat:${currentLocation.latitude} lon:${currentLocation.longitude}")
                textView.text = buffer.toString()
                viewModel.sendDataSbc(currentLocation.latitude, currentLocation.longitude)
            }

        }

        locationService.locationPermissionRequest =
            registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
                locationService.managerResultPermissions(permissions)
            }

        findViewById<View>(R.id.btnGoto).setOnClickListener {
            locationService.startService()
            it.isVisible = false
        }

        viewModel.sendDataSbc(40.34, -3.18)
    }

    override fun onDestroy() {
        super.onDestroy()
        locationService.stopService()
    }
}