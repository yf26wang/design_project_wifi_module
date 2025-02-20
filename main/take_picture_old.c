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

#include "test_images.h"

#define BOARD_ESP32S3_WROOM 1
// ESP32S3 (WROOM) PIN Map
#ifdef BOARD_ESP32S3_WROOM
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 8
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
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
#define WEB_SERVER "www.receive-image-vvjuxrz3lq-uc.a.run.app"
#define WEB_PORT "443"
#define WEB_URL "https://receive-image-vvjuxrz3lq-uc.a.run.app"

#define SERVER_URL_MAX_SZ 256

/* Timer interval once every day (24 Hours) */
#define TIME_PERIOD (86400000000ULL)

static const char HOWSMYSSL_REQUEST[] = "POST " WEB_URL " HTTP/1.1\r\n"
                             "Host: "WEB_SERVER"\r\n"
                             "User-Agent: esp-idf/1.0 esp32\r\n"
                             "Content-Type: text/plain\r\n"
                             "folder: camera0/19343\r\n"
                             "Content-Length: ";
extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");

extern const uint8_t local_server_cert_pem_start[] asm("_binary_local_server_cert_pem_start");
extern const uint8_t local_server_cert_pem_end[]   asm("_binary_local_server_cert_pem_end");
// #if CONFIG_EXAMPLE_USING_ESP_TLS_MBEDTLS
// static const int server_supported_ciphersuites[] = {MBEDTLS_TLS_RSA_WITH_AES_256_GCM_SHA384, MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256, 0};
// static const int server_unsupported_ciphersuites[] = {MBEDTLS_TLS_ECDHE_RSA_WITH_ARIA_128_CBC_SHA256, 0};
// #endif
#ifdef CONFIG_EXAMPLE_CLIENT_SESSION_TICKETS
static esp_tls_client_session_t *tls_client_session = NULL;
static bool save_client_session = false;
#endif

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
    .frame_size = FRAMESIZE_QVGA,   // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12, // 0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2,      // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
};

static void https_get_request2(esp_tls_cfg_t cfg, const char *WEB_SERVER_URL, const char *REQUEST, int size);
static void https_get_request2(esp_tls_cfg_t cfg, const char *WEB_SERVER_URL, const char *REQUEST, int size);
static void https_get_request_using_crt_bundle(void);
static void wifi_request_task(void *pvparameters);

// Globals
static QueueHandle_t x_frame_queue;

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
    xTaskCreate(&wifi_request_task, "https_get_task", 18192, NULL, 0, NULL);
    ESP_LOGI(TAG, "WIFI task created");
    return ESP_OK;
}

static void wifi_request_task(void *pvparameters)
{
    ESP_LOGI(TAG_WIFI, "Start wifi_request_task");

#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE 
    https_get_request_using_crt_bundle();
#elif
    
#endif
    ESP_LOGI(TAG_WIFI, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
    // https_get_request_using_cacert_buf();
    // https_get_request_using_global_ca_store();
    // https_get_request_using_specified_ciphersuites();
    ESP_LOGI(TAG_WIFI, "Finish https_request example");
    vTaskDelete(NULL);
}

static void https_get_request_using_crt_bundle(void)
{
    ESP_LOGI(TAG_WIFI, "https_request using crt bundle");
    esp_tls_cfg_t cfg = {
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    
    // struct test_image image;
    // get_next_image(&image);
    while(1) {
        camera_fb_t * frame = NULL;
        while(!xQueueReceive( x_frame_queue, &frame, ( TickType_t ) portTICK_PERIOD_MS * 10000 )){
            ESP_LOGI(TAG_WIFI, "Waiting...");
        };
        char header[300];
        sprintf(header, "%s%d\r\n\r\n", HOWSMYSSL_REQUEST, frame->len);
        // ESP_LOGI(TAG_WIFI, "header: %s", header);
        int request_size = strlen(header) + frame->len;
        char request[request_size];
        // memset(request,0,request_size);
        strcpy(request,header);
        memcpy(request + strlen(header),frame->buf,frame->len);
        esp_camera_fb_return(frame);
        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        ESP_LOGI(TAG_WIFI, "https_request using crt bundle, start: %lld", time_us);
        https_get_request2(cfg, WEB_URL, request, request_size);
        gettimeofday(&tv_now, NULL);
        time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        ESP_LOGI(TAG_WIFI, "https_request using crt bundle, end: %lld", time_us);
    }
}

static void https_get_request2(esp_tls_cfg_t cfg, const char *WEB_SERVER_URL, const char *REQUEST, int size)
{
    char buf[512];
    int ret, len;

    esp_tls_t *tls = esp_tls_init();
    if (!tls) {
        ESP_LOGE(TAG_WIFI, "Failed to allocate esp_tls handle!");
        // cleanup
        for (int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG_WIFI, "%d...", countdown);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    }

    if (esp_tls_conn_http_new_sync(WEB_SERVER_URL, &cfg, tls) == 1) {
        ESP_LOGI(TAG_WIFI, "Connection established...");
    } else {
        ESP_LOGE(TAG_WIFI, "Connection failed...");
        int esp_tls_code = 0, esp_tls_flags = 0;
        esp_tls_error_handle_t tls_e = NULL;
        esp_tls_get_error_handle(tls, &tls_e);
        /* Try to get TLS stack level error and certificate failure flags, if any */
        ret = esp_tls_get_and_clear_last_error(tls_e, &esp_tls_code, &esp_tls_flags);
        if (ret == ESP_OK) {
            ESP_LOGE(TAG_WIFI, "TLS error = -0x%x, TLS flags = -0x%x", esp_tls_code, esp_tls_flags);
        }
        esp_tls_conn_destroy(tls); // cleanup
    }

#ifdef CONFIG_EXAMPLE_CLIENT_SESSION_TICKETS
    /* The TLS session is successfully established, now saving the session ctx for reuse */
    if (save_client_session) {
        esp_tls_free_client_session(tls_client_session);
        tls_client_session = esp_tls_get_client_session(tls);
    }
#endif

    size_t written_bytes = 0;
    ESP_LOGI(TAG_WIFI, "%d request size", size);
    ESP_LOGI(TAG_WIFI, "request: %s", REQUEST);
    do {
        ret = esp_tls_conn_write(tls,
                                 REQUEST + written_bytes,
                                 size - written_bytes);
        if (ret >= 0) {
            ESP_LOGI(TAG_WIFI, "%d bytes written", ret);
            written_bytes += ret;
        } else if (ret != ESP_TLS_ERR_SSL_WANT_READ  && ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
            ESP_LOGE(TAG_WIFI, "esp_tls_conn_write  returned: [0x%02X](%s)", ret, esp_err_to_name(ret));
            esp_tls_conn_destroy(tls); // cleanup
        }
    } while (written_bytes < size);

    ESP_LOGI(TAG_WIFI, "Reading HTTP response...");
    do {
        len = sizeof(buf) - 1;
        memset(buf, 0x00, sizeof(buf));
        ret = esp_tls_conn_read(tls, (char *)buf, len);

        if (ret == ESP_TLS_ERR_SSL_WANT_WRITE  || ret == ESP_TLS_ERR_SSL_WANT_READ) {
            break;
        } else if (ret < 0) {
            ESP_LOGE(TAG_WIFI, "esp_tls_conn_read  returned [-0x%02X](%s)", -ret, esp_err_to_name(ret));
            break;
        } else if (ret == 0) {
            ESP_LOGI(TAG_WIFI, "connection closed");
            break;
        }

        len = ret;
        ESP_LOGD(TAG_WIFI, "%d bytes read", len);
        /* Print response directly to stdout as it is read */
        for (int i = 0; i < len; i++) {
            putchar(buf[i]);
        }
        putchar('\n'); // JSON output doesn't have a newline at end
    } while (len > 0);

}

void app_main(void)
{
#if ESP_CAMERA_SUPPORTED
    x_frame_queue  = xQueueCreate( 10, sizeof( struct camera_fb_t * ) );
    init_wifi();
    if (ESP_OK != init_camera())
    {
        return;
    }
    int frame_num = 10;
    while (frame_num > 0)
    {
        ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *pic = esp_camera_fb_get();


        // use pic->buf to access the image
        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        ESP_LOGI(TAG, "Picture format: %zu", pic->format);
        ESP_LOGI(TAG, "Picture width, height: (%zu, %zu)", pic->width, pic->height);
        // ESP_LOGI(TAG, "Data:");
        // for(int i = 0; i < pic->len; i++) {
        //     printf("0x%x ", pic->buf[i]);
        // }
        // printf("\n\r");
        ESP_LOGI(TAG, "Sending Picture...");
        xQueueGenericSend( x_frame_queue, ( void * ) &pic, ( TickType_t ) 100 * portTICK_PERIOD_MS, queueSEND_TO_BACK );
        ESP_LOGI(TAG, "Sending Done!");
        // esp_camera_fb_return(pic);
        // break;
        vTaskDelay(5000 / portTICK_RATE_MS);
        frame_num--;
    }
    
#else
    ESP_LOGE(TAG, "Camera support is not available for this chip");
    return;
#endif
}
