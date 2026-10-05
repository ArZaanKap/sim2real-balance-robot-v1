// rl_inference -- runs the exported RL policy on the ESP32 (successor to pid/).
// SKELETON: builds, does nothing yet. Build order:
//   policy.h / policy.c  (forward pass, host fixture test)
//   obs.c / obs.h        (history buffers + obs assembly)
//   this file            (100 Hz loop, SHADOW flag first: compute action, don't drive motors)

#include "motor.h"
#include "encoder.h"
#include "imu.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "policy_main";

void app_main(void){
    ESP_LOGI(TAG, "rl_inference skeleton up");
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}
