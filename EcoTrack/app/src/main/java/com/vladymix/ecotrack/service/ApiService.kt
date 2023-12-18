package com.vladymix.ecotrack.service

import com.vladymix.ecotrack.service.models.LocationRequest
import retrofit2.Call
import retrofit2.Response
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Path


interface ApiService {


    @GET("/api/v1/{clientId}/attributes?clientKeys=noise,pressure&temperature=shared1,shared2")
    fun getData(@Path("clientId") client: String): Call<String>

    @POST("/api/v1/{clientId}/telemetry")
    suspend fun postData(@Path("clientId") client: String, @Body paymentRequest: LocationRequest): Response<Void>

}