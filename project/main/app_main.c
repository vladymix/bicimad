/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <sys/param.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <esp_event_loop.h>
#include <esp_event.h>
#include "esp_wifi.h"
#include "esp_system.h"
// #include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"

#include "cJSON.h"
#include "libs/sbc.h"
#include "libs/mqtt.h"

#include <esp_timer.h>

#include "esp_ota_ops.h"

#include "esp_https_ota.h"
#include "esp_crt_bundle.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include <sys/socket.h>
#include "esp_sleep.h"

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

/*! Saves bit values used in application */
static EventGroupHandle_t event_group;
static const char *TAG = "App Main";

// *******   DEVICES  ***********
AnalogicDevice lux;
AnalogicDevice noise;
OLed oled;
TouchButton button;
Sensor sensor;
struct bme280_t bmp;
Device device;

/*! Saves OTA config received from ThingsBoard*/
static struct shared_keys
{
    char targetFwServerUrl[256];
    char targetFwVer[128];
} shared_attributes;

/*! Factory partiton label */
#define FACTORY_PARTITION_LABEL "factory"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

uint8_t esp32_mac[6];

static int s_retry_num = 0;

void notify_wifi_connected()
{
    xEventGroupClearBits(event_group, WIFI_DISCONNECTED_EVENT);
    xEventGroupSetBits(event_group, WIFI_CONNECTED_EVENT);
}

void notify_wifi_disconnected()
{
    xEventGroupClearBits(event_group, WIFI_CONNECTED_EVENT);
    xEventGroupSetBits(event_group, WIFI_DISCONNECTED_EVENT);
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            notify_wifi_connected();
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool fw_versions_are_equal(const char *current_ver, const char *target_ver)
{
    assert(current_ver != NULL && target_ver != NULL);

    if (strcmp(current_ver, target_ver) == 0)
    {
        ESP_LOGW(TAG, "Skipping OTA, firmware versions are equal - current: %s, target: %s", FIRMWARE_VERSION, shared_attributes.targetFwVer);
        return true;
    }
    return false;
}

static bool ota_params_are_specified(struct shared_keys ota_config)
{
    if (strlen(ota_config.targetFwServerUrl) == 0)
    {
        ESP_LOGW(TAG, "Firmware URL is not specified");
        return false;
    }

    if (strlen(ota_config.targetFwVer) == 0)
    {
        ESP_LOGW(TAG, "Target firmware version is not specified");
        return false;
    }

    return true;
}

static void logOlded(char *log)
{
    // Clear
    oled_display_clear(&oled, 7);
    // Print
    oled_display_text(&oled, 7, log, false);
}

void setMacToMqtt()
{
    char macAddress[30];
    sprintf(macAddress, "%02X:%02X:%02X:%02X:%02X:%02X", esp32_mac[0], esp32_mac[1], esp32_mac[2], esp32_mac[3], esp32_mac[4], esp32_mac[5]);
    logOlded(macAddress);
    //device.config.credentials.client_id = macAddress;
}

void wifi_init_sta(const char *running_partition_label)
{
    assert(running_partition_label != NULL);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    APP_ABORT_ON_ERROR(esp_wifi_init(&cfg));
    APP_ABORT_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    wifi_config_t wifi_config = {};
    APP_ABORT_ON_ERROR(esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config));

    /*  if (wifi_config.sta.ssid[0] == '\0' || wifi_config.sta.password[0] == '\0')
      {
          ESP_LOGW(TAG, "Flash memory doesn't contain any Wi-Fi credentials");
          if (strcmp(FACTORY_PARTITION_LABEL, running_partition_label) == 0)
          {
              ESP_LOGW(TAG, "Factory partition is running, Wi-Fi credentials from config are used and will be saved to the flash memory");
              wifi_sta_config_t wifi_sta_config = {
                  .ssid = CONFIG_EXAMPLE_WIFI_SSID,
                  .password = CONFIG_EXAMPLE_WIFI_PASSWORD,
              };

              wifi_config.sta = wifi_sta_config;
          }
          else
          {
              ESP_LOGE(TAG, "Wi-Fi credentials were not found, running partition is not '%s'", FACTORY_PARTITION_LABEL);
              APP_ABORT_ON_ERROR(ESP_FAIL);
          }
      }
      else
      {
          ESP_LOGI(TAG, "Wi-Fi credentials from flash memory: %s, %s", wifi_config.sta.ssid, wifi_config.sta.password);
      }*/

    /* wifi_sta_config_t wifi_sta_config = {
         .ssid = "CONFIG_EXAMPLE_WIFI_SSID",
         .password = "CONFIG_EXAMPLE_WIFI_PASSWORD",
     };*/

    wifi_sta_config_t wifi_sta_config = {
        .ssid = "SKYNET",
        .password = "volvere1990",
    };

    /* wifi_sta_config_t wifi_sta_config = {
         .ssid = "SBC",
         .password = "SBCwifi$",
     };
     */

    wifi_config.sta = wifi_sta_config;

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    APP_ABORT_ON_ERROR(esp_wifi_get_mac(ESP_IF_WIFI_STA, esp32_mac))
    ESP_LOGI(TAG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X", esp32_mac[0], esp32_mac[1], esp32_mac[2], esp32_mac[3], esp32_mac[4], esp32_mac[5]);
    setMacToMqtt();
    APP_ABORT_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA));

    APP_ABORT_ON_ERROR(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    APP_ABORT_ON_ERROR(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to WIFI");
        notify_wifi_connected();
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", "CONFIG_EXAMPLE_WIFI_SSID", "CONFIG_EXAMPLE_WIFI_PASSWORD");
        notify_wifi_disconnected();
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static enum state connection_state(BaseType_t actual_event, const char *current_state_name)
{
    assert(current_state_name != NULL);

    if (actual_event & WIFI_DISCONNECTED_EVENT)
    {
        ESP_LOGE(TAG, "%s state, Wi-Fi not connected, wait for the connect", current_state_name);
        return STATE_WAIT_WIFI;
    }

    if (actual_event & MQTT_DISCONNECTED_EVENT)
    {
        ESP_LOGW(TAG, "%s state, MQTT not connected, wait for the connect", current_state_name);
        return STATE_WAIT_MQTT;
    }

    return STATE_CONNECTION_IS_OK;
}

static void parse_ota_config(const cJSON *object)
{
    if (object != NULL)
    {
        cJSON *server_url_response = cJSON_GetObjectItem(object, TB_SHARED_ATTR_FIELD_TARGET_FW_URL);
        if (cJSON_IsString(server_url_response) && (server_url_response->valuestring != NULL) && strlen(server_url_response->valuestring) < sizeof(shared_attributes.targetFwServerUrl))
        {
            memcpy(shared_attributes.targetFwServerUrl, server_url_response->valuestring, strlen(server_url_response->valuestring));
            shared_attributes.targetFwServerUrl[sizeof(shared_attributes.targetFwServerUrl) - 1] = 0;
            ESP_LOGI(TAG, "Received firmware URL: %s", shared_attributes.targetFwServerUrl);
        }

        cJSON *target_fw_ver_response = cJSON_GetObjectItem(object, TB_SHARED_ATTR_FIELD_TARGET_FW_VER);
        if (cJSON_IsString(target_fw_ver_response) && (target_fw_ver_response->valuestring != NULL) && strlen(target_fw_ver_response->valuestring) < sizeof(shared_attributes.targetFwVer))
        {
            memcpy(shared_attributes.targetFwVer, target_fw_ver_response->valuestring, strlen(target_fw_ver_response->valuestring));
            shared_attributes.targetFwVer[sizeof(shared_attributes.targetFwVer) - 1] = 0;
            ESP_LOGI(TAG, "Received firmware version: %s", shared_attributes.targetFwVer);
        }
    }
}

static void sendData(esp_mqtt_client_handle_t client, Sensor sensor)
{
    // Crear json que se quiere enviar al ThingsBoard
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "lux", sensor.lux);
    cJSON_AddNumberToObject(root, "humidity", sensor.humidity);
    cJSON_AddNumberToObject(root, "temperature", sensor.temperature);
    cJSON_AddNumberToObject(root, "noise", sensor.noise);
    cJSON_AddNumberToObject(root, "pressure", sensor.pressure);
    cJSON_AddNumberToObject(root, "display_mode", sensor.mode);
    cJSON_AddNumberToObject(root, "time", esp_timer_get_time());

    char *post_data = cJSON_PrintUnformatted(root);
    // Enviar los datos
    esp_mqtt_client_publish(client, "v1/devices/me/telemetry", post_data, 0, 1, 0);
    // v1/  devices / me / telemetry sale de la MQTT Device API Reference de ThingsBoard cJSON_Delete(root);
    // Free is intentional, it's client responsibility to free the result of cJSON_Print
    free(post_data);
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        xEventGroupClearBits(event_group, MQTT_DISCONNECTED_EVENT);
        xEventGroupSetBits(event_group, MQTT_CONNECTED_EVENT);

        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);

        device.client = client;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        device.client = client = NULL;
        xEventGroupClearBits(event_group, MQTT_CONNECTED_EVENT);
        xEventGroupSetBits(event_group, MQTT_DISCONNECTED_EVENT);
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGE(TAG, "MQTT_EVENT_DATA, msg_id=%d, %s", event->msg_id, event->topic);
        if (event->data_len >= (sizeof(mqtt_msg) - 1))
        {
            ESP_LOGE(TAG, "Received MQTT message size [%d] more than expected [%d]", event->data_len, (sizeof(mqtt_msg) - 1));
            break;
        }

        if (strcmp(TB_ATTRIBUTES_RESPONSE_TOPIC, event->topic) == 0)
        {
            memcpy(mqtt_msg, event->data, event->data_len);
            mqtt_msg[event->data_len] = 0;
            ESP_LOGE(TAG, " DATA if mqtt_msg: %s", mqtt_msg);
            cJSON *attributes = cJSON_Parse(mqtt_msg);
            if (attributes != NULL)
            {
                cJSON *shared = cJSON_GetObjectItem(attributes, "shared");
                parse_ota_config(shared);
            }

            char *attributes_string = cJSON_Print(attributes);
            cJSON_Delete(attributes);
            ESP_LOGD(TAG, "Shared attributes response: %s", attributes_string);
            // Free is intentional, it's client responsibility to free the result of cJSON_Print
            free(attributes_string);
            xEventGroupSetBits(event_group, OTA_CONFIG_FETCHED_EVENT);
        }
        else if (strcmp(TB_ATTRIBUTES_TOPIC, event->topic) == 0)
        {
            memcpy(mqtt_msg, event->data, MIN(event->data_len, sizeof(mqtt_msg)));
            ESP_LOGE(TAG, " DATA else mqtt_msg: %s", mqtt_msg);
            mqtt_msg[event->data_len] = 0;
            cJSON *attributes = cJSON_Parse(mqtt_msg);
            parse_ota_config(attributes);
            char *attributes_string = cJSON_Print(attributes);
            cJSON_Delete(attributes);
            ESP_LOGD(TAG, "Shared attributes were updated on ThingsBoard: %s", attributes_string);
            // Free is intentional, it's client responsibility to free the result of cJSON_Print
            free(attributes_string);
            xEventGroupSetBits(event_group, OTA_CONFIG_UPDATED_EVENT);
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s %s", label, hash_print);
}

esp_err_t _ota_http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "OTA HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "OTA HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "OTA HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "OTA HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "OTA HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "OTA HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "OTA HTTP_EVENT_DISCONNECTED");
        break;
    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "OTA HTTP_EVENT_REDIRECT");
        break;
    }
    return ESP_OK;
}

static void start_ota(const char *current_ver, struct shared_keys ota_config)
{
    assert(current_ver != NULL);

    if (!fw_versions_are_equal(current_ver, ota_config.targetFwVer) && ota_params_are_specified(ota_config))
    {
        ESP_LOGW(TAG, "Starting OTA, firmware versions are different - current: %s, target: %s", current_ver, ota_config.targetFwVer);
        ESP_LOGI(TAG, "Target firmware version: %s", ota_config.targetFwVer);
        ESP_LOGI(TAG, "Firmware URL: %s", ota_config.targetFwServerUrl);
        esp_http_client_config_t config = {
            .url = ota_config.targetFwServerUrl,
            .cert_pem = (char *)server_cert_pem_start,
            .event_handler = _ota_http_event_handler,
        };
        logOlded("Download...");

        esp_err_t ret = esp_https_ota(&config);
        if (ret == ESP_OK)
        {
            logOlded("Update completed");
            delayms(2000);
            ESP_LOGW(TAG, "Firmware Updated");
            esp_restart();
        }
        else
        {
            logOlded("Update ERROR");
            ESP_LOGE(TAG, "Firmware  Updated Failed %d", ret);
        }
    }
}

void readLux()
{
    sensor.lux = readAdc1Value(&lux);
    ESP_LOGW(TAG, "Read Lux %d.\n", sensor.lux);
}

void readNoise()
{
    sensor.noise = readAdc1Value(&noise);
    ESP_LOGW(TAG, "Read Noise %d.\n", sensor.noise);
}

void readBmp()
{
    readDataBmp(bmp, &sensor);
}

void displayData()
{
    oled_display_clear(&oled, 1);
    oled_display_clear(&oled, 2);
    switch (sensor.mode)
    {

    case DISPLAY_NOISE:
        /* code */
        char dataNoise[14];
        sprintf(dataNoise, "%d", sensor.noise);
        oled_display_text(&oled, 1, "Noise is: ", false);
        oled_display_text(&oled, 2, dataNoise, false);
        break;

    case DISPLAY_LUX:
        char data[14];
        sprintf(data, "%d", sensor.lux);
        oled_display_text(&oled, 1, "Lux is: ", false);
        oled_display_text(&oled, 2, data, false);
        break;
    case DISPLAY_HUMIDITY:
        /* code */
        char dataHumidity[14];
        sprintf(dataHumidity, "%f", sensor.humidity);
        oled_display_text(&oled, 1, "Humidity is: ", false);
        oled_display_text(&oled, 2, dataHumidity, false);
        break;

    case DISPLAY_PRESSURE:
        /* code */
        char dataPressure[14];
        sprintf(dataPressure, "%f", sensor.pressure);
        oled_display_text(&oled, 1, "Pressure is: ", false);
        oled_display_text(&oled, 2, dataPressure, false);
        break;

    case DISPLAY_TEMPERATURE:
        /* code */
        char dataTemp[14];
        sprintf(dataTemp, "%f", sensor.temperature);
        oled_display_text(&oled, 1, "Temperature is: ", false);
        oled_display_text(&oled, 2, dataTemp, false);
        break;

    default:
        break;
    }

    // oled_display_text(&oled, 4, "Updated to", false);
    oled_display_text(&oled, 5, FIRMWARE_VERSION, false);
}

void logicSensor()
{
    if (device.client)
    {
        readLux();
        readBmp();
        readNoise();
        displayData();
        sendData(device.client, sensor);
        logOlded("Reading sensors");
    }
    else
    {
        logOlded("Client no set");
    }
}

void ota_task(void *pvParameter)
{
    enum state current_connection_state = STATE_CONNECTION_IS_OK;
    enum state state = STATE_INITIAL;
    BaseType_t ota_events;
    BaseType_t actual_event = 0x00;
    char running_partition_label[sizeof(((esp_partition_t *)0)->label)];

    while (1)
    {
        if (state != STATE_INITIAL && state != STATE_APP_LOOP)
        {
            if (state != STATE_APP_LOOP)
            {
                xEventGroupClearBits(event_group, OTA_TASK_IN_NORMAL_STATE_EVENT);
            }

            actual_event = xEventGroupWaitBits(event_group,
                                               WIFI_CONNECTED_EVENT | WIFI_DISCONNECTED_EVENT | MQTT_CONNECTED_EVENT | MQTT_DISCONNECTED_EVENT | OTA_CONFIG_FETCHED_EVENT,
                                               false, false, portMAX_DELAY);
        }

        switch (state)
        {
        case STATE_RETRY_CONECTED:
        {
            logOlded("Retry conect");
            ESP_LOGE(TAG, "Retry conect");
            // state = STATE_INITIAL;
            break;
        }
        case STATE_INITIAL:
        {
            logOlded("Initializing");
            // Initialize NVS.
            esp_err_t err = nvs_flash_init();
            if (err == ESP_ERR_NVS_NO_FREE_PAGES)
            {
                // OTA app partition table has a smaller NVS partition size than the non-OTA
                // partition table. This size mismatch may cause NVS initialization to fail.
                // If this happens, we erase NVS partition and initialize NVS again.
                APP_ABORT_ON_ERROR(nvs_flash_erase());
                err = nvs_flash_init();
            }
            APP_ABORT_ON_ERROR(err);

            const esp_partition_t *running_partition = esp_ota_get_running_partition();
            strncpy(running_partition_label, running_partition->label, sizeof(running_partition_label));
            ESP_LOGI(TAG, "Running partition: %s", running_partition_label);

            wifi_init_sta(running_partition);
            state = STATE_WAIT_WIFI;

            break;
        }
        case STATE_WAIT_WIFI:
        {
            logOlded("wait wifi");
            // state = STATE_WAIT_MQTT;
            // actual_event = WIFI_CONNECTED_EVENT;
            if (actual_event & WIFI_DISCONNECTED_EVENT)
            {
                ESP_LOGW(TAG, "WAIT_WIFI state, Wi-Fi not connected, wait for the connect");
                state = STATE_WAIT_WIFI;
                break;
            }

            if (actual_event & WIFI_CONNECTED_EVENT)
            {
                ESP_LOGD(TAG, "********  Start MQTT *************");
                startMqtt(device);

                state = STATE_WAIT_MQTT;
                break;
            }

            ESP_LOGE(TAG, "WAIT_WIFI state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        case STATE_WAIT_MQTT:
        {
            logOlded("WAIT MQTT");
            current_connection_state = connection_state(actual_event, "WAIT_MQTT");
            if (current_connection_state != STATE_CONNECTION_IS_OK)
            {
                state = current_connection_state;
                break;
            }

            if (actual_event & (WIFI_CONNECTED_EVENT | MQTT_CONNECTED_EVENT))
            {

                logOlded("Sending version");
                ESP_LOGI(TAG, "Send current version %s", FIRMWARE_VERSION);
                // Send the current firmware version to ThingsBoard
                cJSON *current_fw = cJSON_CreateObject();
                cJSON_AddStringToObject(current_fw, TB_CLIENT_ATTR_FIELD_CURRENT_FW, FIRMWARE_VERSION);
                char *current_fw_attribute = cJSON_PrintUnformatted(current_fw);
                cJSON_Delete(current_fw);
                esp_mqtt_client_publish(device.client, TB_ATTRIBUTES_TOPIC, current_fw_attribute, 0, 1, 0);
                // Free is intentional, it's client responsibility to free the result of cJSON_Print
                free(current_fw_attribute);

                // Send the shared attributes keys to receive their values
                esp_mqtt_client_subscribe(device.client, TB_ATTRIBUTES_SUBSCRIBE_TO_RESPONSE_TOPIC, 1);
                esp_mqtt_client_publish(device.client, TB_ATTRIBUTES_REQUEST_TOPIC, TB_SHARED_ATTR_KEYS_REQUEST, 0, 1, 0);
                ESP_LOGI(TAG, "Waiting for shared attributes response");
                logOlded("Waiting config");
                state = STATE_WAIT_OTA_CONFIG_FETCHED;
                break;
            }

            ESP_LOGE(TAG, "WAIT_MQTT state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        case STATE_WAIT_OTA_CONFIG_FETCHED:
        {
            logOlded("OTA Fetched");
            current_connection_state = connection_state(actual_event, "WAIT_OTA_CONFIG_FETCHED");
            if (current_connection_state != STATE_CONNECTION_IS_OK)
            {
                state = current_connection_state;
                break;
            }

            if (actual_event & (WIFI_CONNECTED_EVENT | MQTT_CONNECTED_EVENT))
            {
                if (actual_event & OTA_CONFIG_FETCHED_EVENT)
                {
                    ESP_LOGI(TAG, "Shared attributes were fetched from ThingsBoard");
                    xEventGroupClearBits(event_group, OTA_CONFIG_FETCHED_EVENT);
                    state = STATE_OTA_CONFIG_FETCHED;
                    break;
                }

                state = STATE_WAIT_OTA_CONFIG_FETCHED;
                break;
            }

            ESP_LOGE(TAG, "WAIT_OTA_CONFIG_FETCHED state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        case STATE_OTA_CONFIG_FETCHED:
        {
            logOlded("OTA Config fetched");
            current_connection_state = connection_state(actual_event, "OTA_CONFIG_FETCHED");
            if (current_connection_state != STATE_CONNECTION_IS_OK)
            {
                ESP_LOGE(TAG, "WAIT STATE_OTA_CONFIG_FETCHED actual event%d", actual_event);
                state = current_connection_state;
                break;
            }

            if (actual_event & (WIFI_CONNECTED_EVENT | MQTT_CONNECTED_EVENT))
            {
                ESP_LOGE(TAG, "WAIT WIFI_CONNECTED_EVENT | MQTT_CONNECTED_EVENT actual event%d", actual_event);
                start_ota(FIRMWARE_VERSION, shared_attributes);
                ESP_LOGE(TAG, "Parsed data actual event%d", actual_event);
                esp_mqtt_client_subscribe(device.client, TB_ATTRIBUTES_TOPIC, 1);
                ESP_LOGI(TAG, "Subscribed to shared attributes updates");
                state = STATE_APP_LOOP;
                break;
            }
            ESP_LOGE(TAG, "OTA_CONFIG_FETCHED state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        case STATE_APP_LOOP:
        {
            current_connection_state = connection_state(actual_event, "APP_LOOP");
            if (current_connection_state != STATE_CONNECTION_IS_OK)
            {
                state = current_connection_state;
                break;
            }

            logicSensor();
            delayms(30000);

            if (actual_event & (WIFI_CONNECTED_EVENT | MQTT_CONNECTED_EVENT))
            {
                ota_events = xEventGroupWaitBits(event_group, OTA_CONFIG_UPDATED_EVENT, false, true, 0);
                if ((ota_events & OTA_CONFIG_UPDATED_EVENT))
                {
                    start_ota(FIRMWARE_VERSION, shared_attributes);
                }
                xEventGroupClearBits(event_group, OTA_CONFIG_UPDATED_EVENT);
                xEventGroupSetBits(event_group, OTA_TASK_IN_NORMAL_STATE_EVENT);
                state = STATE_APP_LOOP;
                break;
            }

            ESP_LOGE(TAG, "APP_LOOP state, unexpected event received: %d", actual_event);
            state = STATE_INITIAL;
            break;
        }
        default:
        {
            ESP_LOGE(TAG, "Unexpected state");
            state = STATE_INITIAL;
            break;
        }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void button_handler(TouchButton button)
{
    sensor.mode++;
    sensor.mode = sensor.mode % 5;
    ESP_LOGE("Button", "button_handler status:%d mode:%d\n", button.status, sensor.mode);
    displayData();
}

static void button_handler_task(void *arg)
{
    while (1)
    {
        StateTouch lastState;
        lastState = button.status;
        int level = gpio_get_level(button.gpio);
        if (level == 1)
        {
            button.status = BUTTON_STATE_TOUCH;
        }
        else
        {
            button.status = BUTTON_STATE_RELEASE;
        }

        if (lastState == BUTTON_STATE_TOUCH && button.status == BUTTON_STATE_RELEASE)
        {

            button_handler(button);
        }
        /* code */
        delayms(button.sensitivity);
    }
}

// RTC_IO to wakeup thus requires RTC peripherals
// Only RTC IO can be used as a source for external wake
// source. They are pins: 0,2,4,12-15,25-27,32-39.
static void manager_sleep(void *arg)
{

    while (1)
    {
        /* code */
        delayms(60 * 1000);
        if (device.client == NULL)
        {
            esp_sleep_enable_ext0_wakeup(button.gpio, 1);
            logOlded("Enter in sleep");
            delayms(5 * 1000);
            oled_clear_screen(&oled, false);
            esp_deep_sleep_start();
        }
    }
}

void initButton(TouchButton *button)
{
    button->status = BUTTON_STATE_RELEASE;
    button->time = 0;
    gpio_reset_pin(button->gpio);
    gpio_set_direction(button->gpio, GPIO_MODE_INPUT);
}

void app_main(void)
{
    //{clientId:"ckawzufasqcuwqy7i7gf"} sbc
    //{clientId:"ab2xshew87rhk9md6c0i"} Bici map
    // h6s7vg0nliofvy0c4lfk // sbc
    // 8plu6opoxckrvvw9gjx7 thisnger io mario
    device.event_handler = mqtt_event_handler;
    setMqttConfig(&device, "mqtt://mqtt.thingsboard.cloud", 1883, "5nru3umt4lw1g4xdkzgg");

    sensor.mode = DISPLAY_TEMPERATURE;
    // Initialize OLED
    oled._sda = CONFIG_SDA_GPIO;
    oled._slc = CONFIG_SCL_GPIO;
    oled._reset = CONFIG_RESET_GPIO;
    initOled(&oled);

    // Initialize LDR
    lux.channel = ADC1_CHANNEL_4;
    lux.adc_atten = ADC_ATTEN_DB_11;
    lux.adc_bits_width_t = ADC_WIDTH_BIT_12;
    initAdc1(&lux);

    // Initialize touch button
    // GPIO_NUM_21
    button.gpio = GPIO_NUM_27;
    button.sensitivity = 100;
    initButton(&button);

    // Initialize BMP280
    bmp._sda = GPIO_NUM_22;
    bmp._slc = GPIO_NUM_23;
    bmp.dev_addr = 0x76;
    initBMP(&bmp);
    // readBmp();

    // Initialize noise
    noise.adc_atten = ADC_ATTEN_DB_11;
    noise.adc_bits_width_t = ADC_WIDTH_BIT_12;
    noise.channel = ADC1_CHANNEL_7;
    initAdc1(&noise);

    oled_clear_screen(&oled, false);
    logOlded(FIRMWARE_VERSION);

    event_group = xEventGroupCreate();
    xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
    xTaskCreate(&button_handler_task, "button_handler_task", 4 * 1024, NULL, 5, NULL);
    xTaskCreate(&manager_sleep, "manager_sleep", 4 * 1023, NULL, 6, NULL);
}
