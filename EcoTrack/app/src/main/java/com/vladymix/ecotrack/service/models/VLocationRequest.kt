package com.vladymix.ecotrack.service.models

import com.google.gson.annotations.SerializedName


data class VLocationRequest(
    @SerializedName("session_id")
    val sessionId: String,
    val lat: Double,
    val lon: Double
)