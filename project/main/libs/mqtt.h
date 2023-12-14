#include <stdio.h>

#include "esp_log.h"
#include "mqtt_client.h"
#include <esp_event.h>

/*! Buffer to save a received MQTT message */
static char mqtt_msg[512];

typedef struct 
{
    /* data */
    esp_mqtt_client_config_t config;
    esp_mqtt_client_handle_t client;
    esp_event_handler_t event_handler;
} Device;

void startMqtt(Device device);

void setMqttConfig(Device *device,char *url, int port, char *clientId);
