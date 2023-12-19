package com.vladymix.ecotrack.ui

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.vladymix.ecotrack.service.Api
import kotlinx.coroutines.launch


class TrackViewModel : ViewModel() {
    private val api = Api.getInstance()

    fun sendData(latitude: Double, longitude: Double) {
        viewModelScope.launch {
            api.sendLocation(latitude, longitude)
        }
    }

    fun sendDataSbc(latitude: Double, longitude: Double) {
        viewModelScope.launch {
            api.sendLocationSbc(latitude, longitude)
        }
    }
}