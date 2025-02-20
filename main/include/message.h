#include "esp_camera.h"

typedef struct {
    camera_fb_t * pic;
    int64_t time;
    int seq;
} frame_t;