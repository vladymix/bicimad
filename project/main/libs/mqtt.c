#include "mqtt.h"
#define TAG "lib/MQTT"

void startMqtt(Device device)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&device.config);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, device.event_handler, NULL);
    esp_mqtt_client_start(client);
}

void setMqttConfig(Device *device, char *url, int port, char *clientId)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = url,
        .broker.address.port = port
        }; // FUTURE USE MAC .credentials.username = clientId

    device->config = mqtt_cfg;
}