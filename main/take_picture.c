/**
 * This example takes a picture every 5s and print its size on serial monitor.
 */

// =============================== SETUP ======================================

// 1. Board setup (Uncomment):
// #define BOARD_WROVER_KIT
// #define BOARD_ESP32CAM_AITHINKER
// #define BOARD_ESP32S3_WROOM

/**
 * 2. Kconfig setup
 *
 * If you have a Kconfig file, copy the content from
 *  https://github.com/espressif/esp32-camera/blob/master/Kconfig into it.
 * In case you haven't, copy and paste this Kconfig file inside the src directory.
 * This Kconfig file has definitions that allows more control over the camera and
 * how it will be initialized.
 */

/**
 * 3. Enable PSRAM on sdkconfig:
 *
 * CONFIG_ESP32_SPIRAM_SUPPORT=y
 *
 * More info on
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html#config-esp32-spiram-support
 */

// ================================ CODE ======================================

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"

// WIFI
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "nvs.h"
#include "protocol_examples_common.h"
#include "example_common_private.h"
#include "esp_sntp.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "esp_tls.h"
#include "sdkconfig.h"
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE 
#include "esp_crt_bundle.h"
#endif
#include "time_sync.h"

// #include "test_images.h"
// #include "task_stats.c"

#include "message.h"

#include "esp_http_client.h"

#define BOARD_ESP32S3_WROOM 1
// ESP32S3 (WROOM) PIN Map
#ifdef BOARD_ESP32S3_WROOM
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 15
#define CAM_PIN_XCLK -1 
#define CAM_PIN_SIOD -1
#define CAM_PIN_SIOC -1
#define CAM_PIN_D0 14
#define CAM_PIN_D1 13
#define CAM_PIN_D2 12
#define CAM_PIN_D3 11
#define CAM_PIN_D4 10
#define CAM_PIN_D5 9
#define CAM_PIN_D6 46
#define CAM_PIN_D7 3
#endif
static const char *TAG = "example:take_picture";
static const char *TAG_WIFI = "wifi_request_task";

/* Constants that aren't configurable in menuconfig */
// #define RECEIVE_SERVER "www.receive-image-vvjuxrz3lq-uc.a.run.app"
// #define RECEIVE_URL "https://receive-image-vvjuxrz3lq-uc.a.run.app"
// #define RECEIVE_SERVER "www.172.20.10.8:6000"
// #define RECEIVE_URL "http://172.20.10.8:6000/send"
// #define RECEIVE_SERVER "www.192.168.122.249:5000"
// #define RECEIVE_URL "http://192.168.122.249:5000/send" 
// #define RECEIVE_SERVER "www.192.168.219.249:5000"
// #define RECEIVE_URL "http://192.168.219.249:5000/send" 
#define RECEIVE_SERVER "www.192.168.0.100:5000"
#define RECEIVE_URL "http://192.168.0.100:5000/send" 
// #define RECEIVE_SERVER "www.192.168.1.20:5000"
// #define RECEIVE_URL "http://192.168.1.20:5000/send" 
#define SIG_COMPLETE_SERVER "www.receive-image-vvjuxrz3lq-uc.a.run.app"
#define SIG_COMPLETE_URL "https://receive-image-vvjuxrz3lq-uc.a.run.app"

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048

#define SERVER_URL_MAX_SZ 256

/* Timer interval once every day (24 Hours) */
#define TIME_PERIOD (86400000000ULL)

// static const char HOWSMYSSL_REQUEST[] = "POST " WEB_URL " HTTP/1.1\r\n"
//                              "Host: "WEB_SERVER"\r\n"
//                              "User-Agent: esp-idf/1.0 esp32\r\n"
//                              "Content-Type: image/jpeg\r\n"
//                              "folder: camera0/19343\r\n"
//                              "Content-Length: ";
// extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
// extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");

// extern const uint8_t local_server_cert_pem_start[] asm("_binary_local_server_cert_pem_start");
// extern const uint8_t local_server_cert_pem_end[]   asm("_binary_local_server_cert_pem_end");
// #if CONFIG_EXAMPLE_USING_ESP_TLS_MBEDTLS
// static const int server_supported_ciphersuites[] = {MBEDTLS_TLS_RSA_WITH_AES_256_GCM_SHA384, MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256, 0};
// static const int server_unsupported_ciphersuites[] = {MBEDTLS_TLS_ECDHE_RSA_WITH_ARIA_128_CBC_SHA256, 0};
// #endif
#ifdef CONFIG_EXAMPLE_CLIENT_SESSION_TICKETS
static esp_tls_client_session_t *tls_client_session = NULL;
static bool save_client_session = false;
#endif

#define LED_PIN 0
#define MOTION_DETECTED_PIN 47
#define SEND_DONE_PIN 21 

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

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 24000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    
    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_HD,   // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 4, // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 10,      // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static void http_rest_with_url(void *pvparameters);
static void capture_image();

#define FRAMES_TO_SEND 55

// Globals
static QueueHandle_t x_frame_queue;
esp_timer_handle_t periodic_timer;
volatile int frame_num = 0;

static esp_err_t init_camera(void)
{
    // initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

static esp_err_t init_timer() {
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &capture_image,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic"
    };

    
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    
    // xSemaphore = xSemaphoreCreateBinary();
    // xTaskCreate(&timer_task, "timer task", 8192, NULL, configMAX_PRIORITIES - 3, NULL);
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 200000)); // timer once every 0.2s
    ESP_LOGI(TAG,"Timer initialized");
    return ESP_OK;
}

static void capture_image(void * arg) {
    static uint64_t seq_num = 0;
    static uint64_t time;
    static struct timeval tv_now;
    
    // no motion detected
    if(frame_num == 0) {
        return;
    }
    // first frame
    else if(frame_num == FRAMES_TO_SEND) {
        gettimeofday(&tv_now, NULL);
        time = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    }
    
    camera_fb_t * pic = esp_camera_fb_get();
    // ESP_LOGI(TAG, "Capture");
        if(pic != NULL) {
            frame_t image;
            image.time = time;
            image.seq = seq_num++;
            image.pic = pic;
            xQueueSendToBack( x_frame_queue, ( void * ) &image, pdMS_TO_TICKS(10000000));
            frame_num--;
            if(frame_num == 0) {
                gpio_set_level(LED_PIN, gpio_get_level(LED_PIN) ^ 0x1);
                
                gpio_set_level(SEND_DONE_PIN, 1);
                ESP_LOGI(TAG, "Finished Capturing %d frames!", FRAMES_TO_SEND);
            }
        }
        else {
            ESP_LOGE(TAG, "Invalid image.");
        }
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            // Clean the buffer in case of a new request
            if (output_len == 0 && evt->user_data) {
                // we are just starting to copy the output data into the use
                memset(evt->user_data, 0, MAX_HTTP_OUTPUT_BUFFER);
            }
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                int copy_len = 0;
                if (evt->user_data) {
                    // The last byte in evt->user_data is kept for the NULL character in case of out-of-bound access.
                    copy_len = MIN(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - output_len));
                    if (copy_len) {
                        memcpy(evt->user_data + output_len, evt->data, copy_len);
                    }
                } else {
                    int content_len = esp_http_client_get_content_length(evt->client);
                    if (output_buffer == NULL) {
                        // We initialize output_buffer with 0 because it is used by strlen() and similar functions therefore should be null terminated.
                        output_buffer = (char *) calloc(content_len + 1, sizeof(char));
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    copy_len = MIN(evt->data_len, (content_len - output_len));
                    if (copy_len) {
                        memcpy(output_buffer + output_len, evt->data, copy_len);
                    }
                }
                output_len += copy_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
#if CONFIG_EXAMPLE_ENABLE_RESPONSE_BUFFER_DUMP
                ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
#endif
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
    }
    return ESP_OK;
}

static esp_err_t init_wifi(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_wifi_connect());
    
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&example_wifi_shutdown));

    if (esp_reset_reason() == ESP_RST_POWERON) {
        ESP_LOGI(TAG, "Updating time from NVS");
        ESP_ERROR_CHECK(update_time_from_nvs());
    }

    const esp_timer_create_args_t nvs_update_timer_args = {
            .callback = (void *)&fetch_and_store_time_in_nvs,
    };

    esp_timer_handle_t nvs_update_timer;
    ESP_ERROR_CHECK(esp_timer_create(&nvs_update_timer_args, &nvs_update_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(nvs_update_timer, TIME_PERIOD));
    // configMAX_PRIORITIES - 3 lower than camera
    xTaskCreate(&http_rest_with_url, "https_post_task0", 18192, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(&http_rest_with_url, "https_post_task1", 18192, NULL, configMAX_PRIORITIES - 2, NULL);
    // xTaskCreatePinnedToCore(&http_rest_with_url, "https_post_task", 18192, NULL, configMAX_PRIORITIES - 3, NULL, 1);
    ESP_LOGI(TAG, "WIFI task created");
    return ESP_OK;
}

static void http_rest_with_url(void *pvparameters)
{
    // Declare local_response_buffer with size (MAX_HTTP_OUTPUT_BUFFER + 1) to prevent out of bound access when
    // it is used by functions like strlen(). The buffer should only be used upto size MAX_HTTP_OUTPUT_BUFFER
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER + 1] = {0};
    /**
     * NOTE: All the configuration parameters for http_client must be specified either in URL or as host and path parameters.
     * If host and path parameters are not set, query parameter will be ignored. In such cases,
     * query parameter should be specified in URL.
     *
     * If URL as well as host and path parameters are specified, values of host and path will be considered.
     */
    esp_http_client_config_t config = {
        .host = RECEIVE_SERVER,
        .path = "/",
        // .query = "esp",
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer,        // Pass address of local buffer to get response
        .disable_auto_redirect = true,
        // .crt_bundle_attach = esp_crt_bundle_attach
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    // POST
    esp_http_client_set_url(client, RECEIVE_URL);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "image/jpeg");

    esp_err_t err;
    frame_t frame;
    char length_str[100];
    char time_str[100];
    char frame_number_str[100];
    struct timeval tv_now;
    int64_t time_us;
    int64_t time2_us;
    int queue_messages;
    while(1) {
        gettimeofday(&tv_now, NULL);
        time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        while(!xQueueReceive( x_frame_queue, &frame, pdMS_TO_TICKS(10000) )){
            ESP_LOGI(TAG_WIFI, "Waiting for picture...");
        };

        sprintf(length_str, "%d", frame.pic->len);
        sprintf(time_str, "camera0/%lld", frame.time);
        sprintf(frame_number_str, "%lld", frame.seq);
        esp_http_client_set_header(client, "Content-Length", length_str);
        esp_http_client_set_header(client, "folder", time_str);
        esp_http_client_set_header(client, "frame-number", frame_number_str);
        // ESP_LOGI(TAG_WIFI, "time (microsec): %lld",  frame->time);
        esp_http_client_set_post_field(client, (char *) frame.pic->buf, frame.pic->len);
        err = esp_http_client_perform(client);
        // time_us = frame.time;
        esp_camera_fb_return(frame.pic); // return memory ownership to camera driver
        if (err == ESP_OK) {
            gpio_set_level(LED_PIN, gpio_get_level(LED_PIN) ^ 0x1);
            gettimeofday(&tv_now, NULL);
            time2_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
            queue_messages = uxQueueMessagesWaiting(x_frame_queue);
            ESP_LOGI(TAG, "Queue size: %d, Diff (microsec): %lld", queue_messages, time2_us - time_us);
        } else {
            ESP_LOGE(TAG_WIFI, "HTTP POST request failed: %s", esp_err_to_name(err));
        }
    }
    
    esp_http_client_cleanup(client);
    vTaskDelete(NULL);
}

void IRAM_ATTR motion_detection_interrupt()
{
    if(frame_num == 0){
        frame_num = FRAMES_TO_SEND;
        // gpio_set_level(SEND_DONE_PIN, 0);
        gpio_set_level(SEND_DONE_PIN, 0);
    }
}

void app_main(void)
{
#if ESP_CAMERA_SUPPORTED
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(MOTION_DETECTED_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(SEND_DONE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(SEND_DONE_PIN, 0);

    x_frame_queue  = xQueueCreate( 50, sizeof( frame_t ) );
    init_wifi();
    if (ESP_OK != init_camera())
    {
        return;
    }
    ESP_LOGI(TAG, "Camera init done!");
    init_timer();

    gpio_set_intr_type(MOTION_DETECTED_PIN, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(MOTION_DETECTED_PIN, motion_detection_interrupt, NULL);
    


#else
    ESP_LOGE(TAG, "Camera support is not available for this chip");
    return;
#endif
}
