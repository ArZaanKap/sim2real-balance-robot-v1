// bringup.c — unified sensor/actuator check for the balancing robot.
// STEP 1: boot skeleton only. Prove the build assembles (sh2 + pcnt + ledc)
// before any subsystem code goes in. Fill the rest in yourself, subsystem by
// subsystem: IMU read, encoder x2, motor x2, then the printed dashboard.
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"


static const char *TAG = "bringup";

// pins + consts (left encoder)
#define ENC_A_GPIO 32
#define ENC_B_GPIO 33
#define PCNT_HIGH_LIMIT 3000 // max till counter reset to 0
#define PCNT_LOW_LIMIT (-3000) // min till counter reset to 0
#define GLITCH_NS 1000  // <N nanosecs then ignore encoder reading
#define COUNTS_PER_REV 1976.0f // measured ourselves
#define SAMPLE_MS 20 // ??


void app_main(void)
{
    ESP_LOGI(TAG, "bringup skeleton up");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
