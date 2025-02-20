idf_component_register(SRCS take_picture.c time_sync.c wifi_connect.c
                        # https_request_example_main.c test_images.c
                        INCLUDE_DIRS include
                        REQUIRES esp_wifi
                        REQUIRES esp_timer
                        REQUIRES esp-tls
                        REQUIRES esp_http_client
                        PRIV_INCLUDE_DIRS .
                        PRIV_REQUIRES nvs_flash esp_psram
                        # EMBED_TXTFILES server_root_cert.pem local_server_cert.pem
                        ) 
