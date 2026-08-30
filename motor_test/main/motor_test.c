// motor_test.c — first real motor spin for the sentry rover.
// One DRV8871 H-bridge, one JGA25-370 motor. Sequence: fwd 2s, stop, rev 2s, stop, repeat.
// Pattern mirrors esp_link.c: single .c authored here, built + flashed on the Pi.
//
// Wiring: GPIO25->IN1, GPIO26->IN2, VM=12V+, driver GND + ESP32 GND = COMMON.
// Bring 12V up AFTER flashing, bare shaft, low duty first.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "motor_test";

// --- Pins ---
#define IN1_GPIO        25
#define IN2_GPIO        26

// --- PWM config ---
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_FREQ_HZ     20000                // 20 kHz -> above hearing, no coil whine.
                                             // NOTE: at 20 kHz the motor won't start below ~duty 100-150
                                             // (smooth current, no per-pulse torque kick). See TEST_DUTY.
#define PWM_RES         LEDC_TIMER_8_BIT     // 8-bit -> duty range 0..255 (same as Arduino)
#define IN1_CHANNEL     LEDC_CHANNEL_0
#define IN2_CHANNEL     LEDC_CHANNEL_1

// --- Test parameters ---
// PROVEN in Arduino: 20 kHz + duty 100 = FAILED (too low to break static friction),
//                    20 kHz + duty 180 = WORKED. So 180 is our known-good starting point.
#define TEST_DUTY       180
#define RUN_MS          2000

static void set_pwm(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(PWM_MODE, ch, duty);
    ledc_update_duty(PWM_MODE, ch);
}

static void motors_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t e_timer = ledc_timer_config(&timer);
    ESP_LOGI(TAG, "ledc_timer_config -> %s", esp_err_to_name(e_timer));

    ledc_channel_config_t in1 = {
        .speed_mode = PWM_MODE,
        .channel    = IN1_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .gpio_num   = IN1_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    esp_err_t e_in1 = ledc_channel_config(&in1);
    ESP_LOGI(TAG, "ledc_channel_config IN1 (gpio %d) -> %s", IN1_GPIO, esp_err_to_name(e_in1));

    ledc_channel_config_t in2 = in1;         // same settings, different channel + pin
    in2.channel  = IN2_CHANNEL;
    in2.gpio_num = IN2_GPIO;
    esp_err_t e_in2 = ledc_channel_config(&in2);
    ESP_LOGI(TAG, "ledc_channel_config IN2 (gpio %d) -> %s", IN2_GPIO, esp_err_to_name(e_in2));

    // On ESP32, repeated ledc_set_duty()/ledc_update_duty() changes only stick
    // reliably once the fade service is installed. Without this, the duty works
    // the first time then gets stuck at 0. Install it once here.
    esp_err_t e_fade = ledc_fade_func_install(0);
    ESP_LOGI(TAG, "ledc_fade_func_install -> %s", esp_err_to_name(e_fade));
}

// --- TODO (yours): encode the DRV8871 truth table. ---
// forward: PWM IN1, hold IN2 at 0.   reverse: PWM IN2, hold IN1 at 0.   stop: both 0.
static void forward(uint32_t duty)
{
    set_pwm(IN1_CHANNEL, duty);
    set_pwm(IN2_CHANNEL, 0);
}

static void reverse(uint32_t duty)
{
    set_pwm(IN1_CHANNEL, 0);
    set_pwm(IN2_CHANNEL, duty);
}

static void stop(void)
{
    set_pwm(IN1_CHANNEL, 0);
    set_pwm(IN2_CHANNEL, 0);
}

void app_main(void)
{
    motors_init();

    for (;;) {

        forward(TEST_DUTY);
        vTaskDelay(pdMS_TO_TICKS(RUN_MS));
        // Read AFTER the duty has been stable for 2s -> reliable value.
        ESP_LOGI(TAG, "FORWARD (stable): IN1=%lu IN2=%lu",
                 (unsigned long)ledc_get_duty(PWM_MODE, IN1_CHANNEL),
                 (unsigned long)ledc_get_duty(PWM_MODE, IN2_CHANNEL));

        stop();
        vTaskDelay(pdMS_TO_TICKS(500)); // pause 500ms

        reverse(TEST_DUTY);
        vTaskDelay(pdMS_TO_TICKS(RUN_MS));
        ESP_LOGI(TAG, "REVERSE (stable): IN1=%lu IN2=%lu",
                 (unsigned long)ledc_get_duty(PWM_MODE, IN1_CHANNEL),
                 (unsigned long)ledc_get_duty(PWM_MODE, IN2_CHANNEL));

        stop();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
