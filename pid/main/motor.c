#include "motor.h"


// MOTOR
typedef struct{
    ledc_channel_t in1_ch, in2_ch;
} motor_t;

static motor_t mot_l = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};
static motor_t mot_r = {LEDC_CHANNEL_2, LEDC_CHANNEL_3};



// ----------- MOTOR FUNCTIONS --------------- // 
void ledc_common_init(void){
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

void motor_init(motor_t *m, int in1_gpio, int in2_gpio){
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

void motor_drive(motor_t *m, int duty){
    if (duty >  255) duty =  255;
    if (duty < -255) duty = -255;
    int in1 = (duty > 0) ?  duty : 0;
    int in2 = (duty < 0) ? -duty : 0;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->in1_ch, in1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->in1_ch);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->in2_ch, in2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->in2_ch);
}

