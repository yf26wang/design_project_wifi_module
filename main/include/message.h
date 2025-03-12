#include "esp_camera.h"

typedef struct {
    camera_fb_t * pic;
    uint64_t time;
    uint64_t seq;
} frame_t;