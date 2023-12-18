package com.vladymix.ecotrack

import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.EditText
import android.widget.Toast
import com.vladymix.ecotrack.ui.TrackActivity

class MainActivity : BaseActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        findViewById<View>(R.id.btnGoto).setOnClickListener {

            findViewById<EditText>(R.id.editTextDevice).text.let {
                if (it.toString().isEmpty()) {
                    Toast.makeText(this, "Device can´t empty", Toast.LENGTH_LONG).show()
                } else {
                    api.setDevice(findViewById<EditText>(R.id.editTextDevice).text.toString())
                    startActivity(Intent(this, TrackActivity::class.java))
                }
            }
        }
    }
}