package com.vladymix.ecotrack

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.vladymix.ecotrack.service.Api


open class BaseActivity : AppCompatActivity() {
    lateinit var api: Api

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        api = Api.getInstance()
    }
}