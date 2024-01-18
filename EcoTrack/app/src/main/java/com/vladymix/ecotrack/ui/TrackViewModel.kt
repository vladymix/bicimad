package com.vladymix.ecotrack.ui

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.vladymix.ecotrack.service.Api
import com.vladymix.ecotrack.service.VladyApi
import com.vladymix.ecotrack.service.models.Device
import kotlinx.coroutines.launch


class TrackViewModel : ViewModel() {
    private val api = Api.getInstance()
    private val vApi = VladyApi.getInstance()
    private val _dataDevice  = MutableLiveData<Device>()
    val dataDevice : LiveData<Device> get() = _dataDevice

    fun sendData(latitude: Double, longitude: Double) {
        viewModelScope.launch {
            api.sendLocation(latitude, longitude)
        }
    }

    fun getData(){
        viewModelScope.launch {
            try {
                api.getData()?.let {
                    _dataDevice.postValue(it)
                }
            }catch (ex:Exception){
                ex.printStackTrace()
            }
        }

    }

    fun sendDataSbc(latitude: Double, longitude: Double) {
        viewModelScope.launch {
            api.sendLocationSbc(latitude, longitude)
        }
    }

    fun sendDataServer(latitude: Double, longitude: Double) {
        viewModelScope.launch {
            vApi.sendLocation(latitude, longitude)
        }
    }
}