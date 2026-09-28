#include "motor.h"
#include "encoder.h"
#include "imu.h"


#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_timer.h"




// compare with standalone files - make modular next
static const char *TAG = "bringup";


#define SAMPLE_MS 10 // (1/hz) latency of measurements



void app_main(void){
    //ESP_LOGI(TAG, "bringup skeleton up");

    static motor_t mot_l = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};
    static motor_t mot_r = {LEDC_CHANNEL_2, LEDC_CHANNEL_3};

    ledc_common_init();
    motor_init(&mot_l, MOT_L_IN1_GPIO, MOT_L_IN2_GPIO);
    motor_init(&mot_r, MOT_R_IN1_GPIO, MOT_R_IN2_GPIO);

    static encoder_t enc_l, enc_r;
    encoder_init(&enc_l, ENC_L_A_GPIO, ENC_L_B_GPIO);
    encoder_init(&enc_r, ENC_R_A_GPIO, ENC_R_B_GPIO);

    sh2_Hal_t *hal = make_hal();
    int rc = sh2_open(hal, async_event_handler, NULL);

    // no point continuing
    if (rc != SH2_OK){
        ESP_LOGE(TAG, "sh2_open failed: %d", rc);
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000));}
    }

    sh2_setSensorCallback(sensor_event_handler, NULL);
    log_product_ids();

    s_runtime_reset = false;
    if (!enable_test_reports()){
        ESP_LOGE(TAG, "one or more reports could not be enabled");
    }

    TickType_t last_wake = xTaskGetTickCount(); //?
    const TickType_t period = pdMS_TO_TICKS(SAMPLE_MS); //?

    int64_t last_pos_l = read_position(&enc_l);
    int64_t last_pos_r = read_position(&enc_r);

    int tick = 0;
    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS)); 
        vTaskDelayUntil(&last_wake, period); // difference??

        // IMU
        // drain imu so s_latest holds newest - cos queue not stack internally?
        int guard = 0;
        while (gpio_get_level(IMU_INT_GPIO) == 0 && guard++ < 16){ // explain??
            sh2_service();
        }
        if (s_runtime_reset){
            s_runtime_reset = false;
            enable_test_reports();
        }

        float theta_x, theta_y, theta_z;
        quaternion_to_euler(s_latest.real, s_latest.i, s_latest.j, s_latest.k, &theta_x, &theta_y, &theta_z); // takes pointer , not pass by ref??
        float theta_y_deg = pitch * RAD_TO_DEG;
        float w_y = s_latest.gy; 


        // ENCODER
        int64_t pos_l = read_position(&enc_l); // encoder counts
        int64_t pos_r = read_position(&enc_r);

        // difference in encoder counts / COUNTS_PER_REV = revs (in 20ms)
        float revs_l = (float)(pos_l - last_pos_l) / COUNTS_PER_REV; // convert encoder counts -> revolutions
        float revs_r = (float)(pos_r - last_pos_r) / COUNTS_PER_REV; // convert encoder counts -> revolutions
        
        // revs * 2pi = radians moved during 1 sample - scale by Hz to get radians per sec
        float rads_l = revs_l * 6.283185f * (1000.0f / (float)SAMPLE_MS);
        float rads_r = revs_r * 6.283185f * (1000.0f / (float)SAMPLE_MS);

        last_pos_l = pos_l;
        last_pos_r = pos_r;


        // MOTOR
        
        
        int u = pid_controller();
        motor_drive(&mot_l, drive);
        motor_drive(&mot_r, drive);
        

        if (tick % 20 == 0){
            //ESP_LOGI(TAG, "L pos=%lld rad/s=%.2f | R pos=%lld rad/s=%.2f", (long long)pos_l, rads_l, (long long)pos_r, rads_r);
            //ESP_LOGI(TAG, "pitch=%6.2f deg  rate=%6.2f", pitch_deg, pitch_rate);
            // TEMP debug — identify pitch axis + sign, then delete these two lines
            ESP_LOGI(TAG, "  gyro  gx=%6.2f gy=%6.2f gz=%6.2f", 
                s_latest.gx * RAD_TO_DEG, s_latest.gy * RAD_TO_DEG, s_latest.gz * RAD_TO_DEG);
            ESP_LOGI(TAG, "  roll=%6.2f pitch=%6.2f yaw=%6.2f",
                     roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);
        }

        tick++;
        
    }
}
