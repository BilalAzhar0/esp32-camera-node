#include <string.h>
#include <stdlib.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "../../data.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include <time.h>


#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#include "esp_camera.h"

// #if CONFIG_ESP_WIFI_AUTH_OPEN
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
// #elif CONFIG_ESP_WIFI_AUTH_WEP
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
// #elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK 
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
// #endif


#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


static EventGroupHandle_t s_wifi_event_group;

static const char *wifi_TAG = "wifi station";
static const char *http_TAG = "HTTP client";
static const char *sntp_TAG = "SNTP";
static const char *cam_TAG = "camera";

static int s_retry_num = 0;

//********************************** CAMERA PIN CONFIGURE **********************************//
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, 
    .frame_size = FRAMESIZE_UXGA,  

    .jpeg_quality = 10, 
    .fb_count = 1,    
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};
static esp_err_t init_camera(void)
{
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(wifi_TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}
#endif

//********************************** WIFI EVENT HANDLER **********************************//
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(wifi_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(wifi_TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(wifi_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
//********************************** WIFI CONFIGURATION METHOD **********************************//
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&event_handler,NULL,&instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&event_handler,NULL,&instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(wifi_TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(wifi_TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(wifi_TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(wifi_TAG, "UNEXPECTED EVENT");
    }
}

//********************************** HTTP EVENT HANDLER **********************************//
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer; 
    static int output_len;       
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(http_TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(http_TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(http_TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(http_TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(http_TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                if (evt->user_data) {
                    memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                } else {
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(esp_http_client_get_content_length(evt->client));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(http_TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    memcpy(output_buffer + output_len, evt->data, evt->data_len);
                }
                output_len += evt->data_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(http_TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                ESP_LOG_BUFFER_HEX(http_TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(http_TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(http_TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(http_TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(http_TAG, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
    }
    return ESP_OK;
}



static void http_image_post(camera_fb_t *pic,char* header)
{
    esp_http_client_config_t config = {
        .url =  flask_server,
        .method = HTTP_METHOD_POST,
        // .host = "192.168.7.179",
        // .path = "/file",
        // .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .event_handler = _http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client,"Filename",header);
    esp_http_client_set_post_field(client,(char*)pic->buf,pic->len);    
    
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(http_TAG, "HTTP POST Status = %d, content_length = %lld",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(http_TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }
    esp_camera_fb_return(pic);
    esp_http_client_cleanup(client);
}

char* getCurrentTime() {
    time_t now;
    time(&now);

    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    static char time_string[20];
    strftime(time_string, sizeof(time_string), "%Y-%m-%d-%H-%M-%S", &timeinfo);
    return time_string;
}
char* getNodeID(){
    uint8_t MAC_ADDRESS[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, MAC_ADDRESS);

    char* sum_string = (char*)malloc(7); 
    sprintf(sum_string, "%02X%02X%02X", (int)MAC_ADDRESS[3], (int)MAC_ADDRESS[4], (int)MAC_ADDRESS[5]);

    return sum_string;
}
void initializeSntp(){

    setenv("TZ", "PKT-5", 1);
    tzset();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI(sntp_TAG, "Waiting for system time synchronization... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    if (retry >= retry_count) {
        ESP_LOGE(sntp_TAG, "Time synchronization failed");
    } else {
        char time_string[20];
        strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", &timeinfo);
        ESP_LOGI(sntp_TAG, "Current time: %s", time_string);
    }

}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(wifi_TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    initializeSntp();
    
    char* NODE_ID = getNodeID();
    free(getNodeID());

    #if ESP_CAMERA_SUPPORTED
    if(ESP_OK != init_camera()) {
        return;
    }

    while (1)
    {   
        char node_time_stamp[27];
        strcpy(node_time_stamp,NODE_ID);
        strcat(node_time_stamp, "-");
        strcat(node_time_stamp,getCurrentTime()); 

        ESP_LOGI("MAC","time IS :%s",node_time_stamp);

        ESP_LOGI(cam_TAG, "Taking picture...");
        camera_fb_t *pic = NULL; 
        pic = esp_camera_fb_get();
        ESP_LOGI(cam_TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        
        http_image_post(pic,node_time_stamp);

        vTaskDelay(5000 / portTICK_RATE_MS);


    }
#else
    ESP_LOGE(TAG, "Camera support is not available for this chip");
    return;
#endif

}
