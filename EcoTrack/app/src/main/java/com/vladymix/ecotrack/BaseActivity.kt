package com.vladymix.ecotrack

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.vladymix.ecotrack.service.Api
import com.vladymix.ecotrack.service.VladyApi


open class BaseActivity : AppCompatActivity() {
    lateinit var api: Api
    lateinit var vApi: VladyApi

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        api = Api.getInstance()
        vApi = VladyApi.getInstance()
    }
}