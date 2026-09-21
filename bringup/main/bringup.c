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

#include "driver/ledc.h"

// compare with standalone files - make modular next
static const char *TAG = "bringup";

// pins + consts

// motor
#define MOT_L_IN1_GPIO        27
#define MOT_L_IN2_GPIO        14
#define MOT_R_IN1_GPIO        25
#define MOT_R_IN2_GPIO        26

#define PWM_FREQ_HZ     20000                        
#define PWM_RES         LEDC_TIMER_8_BIT     // 8-bit -> duty range 0..255 (same as Arduino)
#define COMMAND       180

// encoder
#define ENC_L_A_GPIO 33
#define ENC_L_B_GPIO 32
#define ENC_R_A_GPIO 4
#define ENC_R_B_GPIO 13

#define PCNT_HIGH_LIMIT 3000 // max till counter reset to 0
#define PCNT_LOW_LIMIT (-3000) // min till counter reset to 0
#define GLITCH_NS 1000  // <N nanosecs then ignore encoder reading
#define COUNTS_PER_REV 1976.0f // measured ourselves
#define SAMPLE_MS 20 // (1/hz) latency of measurements


// MOTOR
typedef struct{
    ledc_channel_t in1_ch, in2_ch;
} motor_t;

static motor_t mot_l = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};
static motor_t mot_r = {LEDC_CHANNEL_2, LEDC_CHANNEL_3};


// ENCODER

// overflow = big running total, edited by ISR (Interrupt Service Routine)
// when something counter hits +/-3000
typedef struct {
    pcnt_unit_handle_t unit; // handle
    volatile int64_t overflow; // volatile - standard for variables accessed by interrupts and main?
} encoder_t;

// init both
static encoder_t enc_l;
static encoder_t enc_r;


// ----------- MOTOR FUNCTIONS --------------- // 
static void ledc_common_init(void){
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE, // ??
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));
    ESP_ERROR_CHECK(ledc_fade_func_install(0)); // ONCE, globally — or set_duty sticks at 0
}

static void motor_init(motor_t *m, int in1_gpio, int in2_gpio){
    ledc_channel_config_t ccfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty = 0, 
        .hpoint = 0,
        .channel = m->in1_ch, 
        .gpio_num = in1_gpio,
    };
    // whats happening under here now?
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));   // IN1

    ccfg.channel  = m->in2_ch;
    ccfg.gpio_num = in2_gpio;
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));   // IN2

    ESP_LOGI(TAG, "motor ready: IN1=GPIO%d IN2=GPIO%d", in1_gpio, in2_gpio);
}

static void motor_drive(motor_t *m, int duty){
    if (duty >  255) duty =  255;
    if (duty < -255) duty = -255;
    int in1 = (duty > 0) ?  duty : 0;
    int in2 = (duty < 0) ? -duty : 0;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->in1_ch, in1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->in1_ch);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->in2_ch, in2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->in2_ch);
}

// ------ ENCODER FUNCTIONS ------------ //

// ISR runs automatically on a +/-3000 watch_point. 
// IRAM_ATTR -> put this func in fast RAM
static bool IRAM_ATTR on_watch_point(pcnt_unit_handle_t unit, // weird how is variable before func name?
               const pcnt_watch_event_data_t *edata, 
               void *user_ctx){
            
    encoder_t *e = (encoder_t*)user_ctx; // which encoder arrives here
    e->overflow += edata->watch_point_value; // update THAT encoder's total
    return false;
}

// true pos = big total + whatever in counter right now (whats hw?)
static int64_t read_position(encoder_t *e){
    int hw = 0; // hardware counter
    pcnt_unit_get_count(e->unit, &hw); // place count from unit into hw -> thats why & used
    return e->overflow + hw;
}

static void encoder_init(encoder_t *e, int a_gpio, int b_gpio){

    // 1) Create the counter unit with its wrap-around limits.
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &e->unit));

    // 2) Hardware glitch filter (debounce).
    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = GLITCH_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(e->unit, &filter_cfg));

    // 3) Two channels for 4x quadrature decode.
    //    Channel A watches EDGES on A, and reads the LEVEL of B to decide up/down.
    //    Channel B watches EDGES on B, and reads the LEVEL of A. Together = all 4 edges.
    pcnt_chan_config_t chan_a_cfg = { .edge_gpio_num = a_gpio, .level_gpio_num = b_gpio};
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(e->unit, &chan_a_cfg, &chan_a));

    pcnt_chan_config_t chan_b_cfg = { .edge_gpio_num = b_gpio, .level_gpio_num = a_gpio};
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(e->unit, &chan_b_cfg, &chan_b));

    // 4) The quadrature truth table, expressed as edge + level actions.
    //    (This is the canonical ESP-IDF rotary-encoder config: count on both edges of both channels,
    //     direction flips based on the other channel's level.)
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));   // A rising/falling
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));      // gated by B
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));   // B rising/falling
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));      // gated by A
    // If the sign comes out backwards vs the direction you spin, swap A and B wires (or swap the
    // INCREASE/DECREASE pair on one channel). Easiest fix is at the connector.

    // 5) Watch points at the limits so overflow accumulation works, plus the callback.
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(e->unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(e->unit, PCNT_LOW_LIMIT));
    pcnt_event_callbacks_t cbs = { .on_reach = on_watch_point };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(e->unit, &cbs, e));

    // 6) Internal pull-ups on A/B. Harmless if the encoder drives push-pull; protective if it's
    //    open-collector or a wire falls off (keeps the line from floating and false-counting).
    gpio_set_pull_mode(a_gpio, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(b_gpio, GPIO_PULLUP_ONLY);

    // 7) Enable, zero, go.
    ESP_ERROR_CHECK(pcnt_unit_enable(e->unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(e->unit));
    ESP_ERROR_CHECK(pcnt_unit_start(e->unit));

    ESP_LOGI(TAG, "PCNT encoder ready: A=GPIO%d B=GPIO%d, 4x quadrature, filter %d ns",
             a_gpio, b_gpio, GLITCH_NS);

}


void app_main(void)
{
    //ESP_LOGI(TAG, "bringup skeleton up");

    ledc_common_init();
    motor_init(&mot_l, MOT_L_IN1_GPIO, MOT_L_IN2_GPIO);
    motor_init(&mot_r, MOT_R_IN1_GPIO, MOT_R_IN2_GPIO);

    encoder_init(&enc_l, ENC_L_A_GPIO, ENC_L_B_GPIO);
    encoder_init(&enc_r, ENC_R_A_GPIO, ENC_R_B_GPIO);

    int64_t last_pos_l = read_position(&enc_l);
    int64_t last_pos_r = read_position(&enc_r);

    int tick = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS)); // 20ms for now

        // MOTOR
        // 20ms * 50 = every 1s switch mode
        int phase = (tick / 50) % 4; // forward, stop, backward, stop
        int drive = (phase==0) ? COMMAND : (phase==2) ? -COMMAND : 0;
        motor_drive(&mot_l, drive);
        motor_drive(&mot_r, drive);
        tick++;


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

        ESP_LOGI(TAG, "L pos=%lld rad/s=%.2f | R pos=%lld rad/s=%.2f", (long long)pos_l, rads_l, (long long)pos_r, rads_r);
    }
}
