package com.vladymix.ecotrack.service


import com.vladymix.ecotrack.service.models.VLocationRequest
import retrofit2.Response
import retrofit2.http.Body
import retrofit2.http.POST


interface VApiService {
    @POST("/locations/gps/")
    suspend fun setLastLocation(@Body paymentRequest: VLocationRequest): Response<Void>

}