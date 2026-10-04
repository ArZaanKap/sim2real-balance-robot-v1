#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "imu.h"

#define SAMPLE_MS 10
#define DRAIN_GUARD 16

static const char *TAG = "imu_noise";

void app_main(void)
{
    // Keep each CSV row visible immediately to the host capture script.
    setvbuf(stdout, NULL, _IONBF, 0);

    ESP_LOGI(TAG, "100 Hz static IMU logger starting");
    ESP_LOGI(TAG, "keep the robot fixed upright and the motor supply OFF");

    sh2_Hal_t *hal = make_hal();
    int rc = sh2_open(hal, async_event_handler, NULL);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "sh2_open failed: %d", rc);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    sh2_setSensorCallback(sensor_event_handler, NULL);
    log_product_ids();

    s_runtime_reset = false;
    if (!enable_test_reports()) {
        ESP_LOGE(TAG, "one or more reports could not be enabled");
    }

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_MS);
    uint32_t tick = 0;

    // capture.py deliberately keeps only rows beginning with "IMU,".
    // Columns: tick,time_us,sample_count,theta_y_rad,gyro_y_rad_s,accuracy
    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        int guard = 0;
        while ((gpio_get_level(IMU_INT_GPIO) == 0) && (guard++ < DRAIN_GUARD)) {
            sh2_service();
        }

        if (s_runtime_reset) {
            s_runtime_reset = false;
            s_latest.have_quaternion = false;
            s_latest.have_gyro = false;
            if (!enable_test_reports()) {
                ESP_LOGE(TAG, "failed to restore reports after sensor reset");
            }
        }

        if (!s_latest.have_quaternion || !s_latest.have_gyro) {
            continue;
        }

        float theta_x;
        float theta_y;
        float theta_z;
        quaternion_to_euler(s_latest.real, s_latest.i, s_latest.j, s_latest.k,
                            &theta_x, &theta_y, &theta_z);

        printf("IMU,%lu,%lld,%lu,%.9f,%.9f,%u\n",
               (unsigned long)tick,
               (long long)esp_timer_get_time(),
               (unsigned long)s_latest.sample_count,
               (double)theta_y,
               (double)s_latest.gy,
               (unsigned)s_latest.accuracy);
        tick++;
    }
}
