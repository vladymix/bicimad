package com.vladymix.ecotrack.service.models


import com.google.gson.annotations.SerializedName

data class Device(
    @SerializedName("display_mode")
    val displayMode: Int,
    val humidity: Double,
    val latitude: Double,
    @SerializedName("log_time")
    val logTime: Int,
    val longitude: Double,
    val lux: Int,
    val noise: Int,
    val pressure: Double,
    val temperature: Double
)