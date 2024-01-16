package com.vladymix.ecotrack.service

import android.util.Log
import com.google.gson.GsonBuilder
import com.vladymix.ecotrack.service.models.VLocationRequest
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import okhttp3.OkHttpClient
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory


class VladyApi {

    private val retrofit: Retrofit
    private val service: VApiService

    private lateinit var device: String

    companion object {
        private val BASE_URL = "https://apiservice.vladymix.es"
        private var instance: VladyApi? = null
        fun getInstance(): VladyApi {
            if (instance == null) {
                instance = VladyApi()
            }
            return instance!!
        }
    }

    init {
        val client = OkHttpClient.Builder().build()
        val converter = GsonConverterFactory.create(GsonBuilder().create())

        retrofit = Retrofit.Builder()
            .baseUrl(BASE_URL)
            .addConverterFactory(converter)
            .client(client)
            .build()
        service = retrofit.create(VApiService::class.java)
    }

    fun setDevice(device: String) {
        this.device = device
    }

    suspend fun sendLocation(latitude: Double, longitude: Double) {
        withContext(Dispatchers.IO) {
            try {
                val response = service.setLastLocation(VLocationRequest(device,latitude, longitude))
                if (response.isSuccessful) {
                    Log.i("API","Send data successful")
                }
            } catch (ex: Exception) {
                ex.printStackTrace()
            }
        }

    }
}