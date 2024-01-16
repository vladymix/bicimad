package com.vladymix.ecotrack

import android.content.Intent
import android.location.Location
import android.os.Bundle
import android.view.View
import android.widget.EditText
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import com.vladymix.ecotrack.service.LocationService
import com.vladymix.ecotrack.ui.MapsActivity
import com.vladymix.ecotrack.ui.TrackActivity
import java.util.Calendar

class MainActivity : BaseActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        findViewById<View>(R.id.btnGoto).setOnClickListener {

            findViewById<EditText>(R.id.editTextDevice).text.let {
                if (it.toString().isEmpty()) {
                    Toast.makeText(this, "Device can´t empty", Toast.LENGTH_LONG).show()
                } else {
                    vApi.setDevice(findViewById<EditText>(R.id.editTextDevice).text.toString())
                    startActivity(Intent(this, MapsActivity::class.java))
                }
            }
        }
        getPermissions()
    }

    private fun getPermissions() {
        val locationService = LocationService(this)
        locationService.locationPermissionRequest =
            registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { permissions ->
                locationService.managerResultPermissions(permissions)
            }

        locationService.listener = object : LocationService.Result {
            override fun permissionsGranted() {
                Toast.makeText(baseContext, "Permissions ready", Toast.LENGTH_LONG).show()
            }

            override fun permissionsDenied() {
                Toast.makeText(baseContext, "No has permission", Toast.LENGTH_LONG).show()
            }

            override fun updateLocation(lastLocation: Location?, currentLocation: Location) {

            }

        }
        locationService.startService()
    }
}