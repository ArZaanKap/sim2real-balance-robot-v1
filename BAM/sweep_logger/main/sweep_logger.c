// sweep_logger.c — bench data capture for BAM actuator-model identification (GA25-370).
//
// This is the DATA-CAPTURE half of the BAM plan. It drives ONE motor through a DRV8871
// (LEDC PWM, from motor_test.c) while reading its encoder (PCNT, from encoder_test.c), and
// prints clean CSV over serial. You pipe that CSV to a file, then BAM (on the PC) fits a
// friction/back-EMF model to it, and those fitted numbers become the wheel actuator in MuJoCo.
//
// It is NOT a control loop — it runs an open-loop schedule of commanded voltages (duties) and
// records how the wheel actually responds. Three sweeps, run ONCE, then it idles (coasts):
//   1) STAIRCASE  — hold a series of duties both directions -> steady-state speed vs voltage
//                    (gives viscous + Stribeck) AND the spin-up transient (gives inertia).
//   2) SPINDOWN   — spin up, then cut power to COAST -> deceleration is friction ONLY
//                    (separates friction from the drive torque; the cleanest friction signal).
//   3) DEADBAND   — creep the duty up from ~0 -> the duty where it FIRST moves = static
//                    friction / start-up deadband (the term that most hurts fine balance moves).
//
// Bench setup: bare shaft (no chassis), motor clamped, 12 V up AFTER flashing. Nothing here is
// closed-loop, so a runaway is impossible — worst case the wheel just spins.
//
// ---- Wiring (same as motor_test.c + encoder_test.c on ONE motor) -------------------------------
//   Motor drive:  GPIO25 -> DRV8871 IN1,  GPIO26 -> IN2,  VM = 12 V+,  driver GND + ESP32 GND COMMON.
//   Encoder:      GPIO32 -> channel A,     GPIO33 -> channel B,  encoder VCC = 3.3 V,  GND COMMON.
// ------------------------------------------------------------------------------------------------
//
// ---- Capturing the data ------------------------------------------------------------------------
//   Flash, then log ONLY the data rows (they start with "BAM,") to a file:
//       idf.py -p /dev/ttyUSB0 flash
//       stty -F /dev/ttyUSB0 115200 raw && cat /dev/ttyUSB0 | grep --line-buffered '^BAM,' > run.csv
//   Press the board's EN/RST button to restart the run. Stop the cat once you see "BAM,DONE".
//   (idf.py monitor also works, but it decorates lines; the grep above keeps the CSV pure.)
// ------------------------------------------------------------------------------------------------

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "bam_sweep";

// --- Motor drive pins + PWM (from motor_test.c) -------------------------------------------------
#define IN1_GPIO        25
#define IN2_GPIO        26
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_FREQ_HZ     20000
#define PWM_RES         LEDC_TIMER_8_BIT     // duty 0..255, like Arduino analogWrite
#define IN1_CHANNEL     LEDC_CHANNEL_0
#define IN2_CHANNEL     LEDC_CHANNEL_1

// --- Encoder pins (from encoder_test.c) ---------------------------------------------------------
#define ENC_A_GPIO      32
#define ENC_B_GPIO      33
#define PCNT_HIGH_LIMIT  3000
#define PCNT_LOW_LIMIT  (-3000)
#define GLITCH_NS        1000

// --- Scaling / logging --------------------------------------------------------------------------
// MEASURED 2026-08-31: 3 hand-turn trials (15/14/10 revs) all gave ~1976 -> 13 hall poles
// x4 quadrature x 38:1 gearbox = 1976 (and 130RPM x 38 ~= 4940 RPM base motor, checks out).
#define COUNTS_PER_REV   1976.0f
// V_BUS is what "duty 255" actually applies across the motor. MEASURE your pack under load and set
// it here — BAM works in volts, so voltage = (duty/255) * V_BUS. Duty alone is only as good as this.
// MEASURED 2026-08-31: bench supply set 12.0V, read 11.97V at VM->GND under load (rock-steady rail).
#define V_BUS_VOLTS      11.97f
#define SAMPLE_MS        10         // 100 Hz logging — fast enough to catch spin-up + spin-down curves

// Position accumulator (written from the PCNT watch-point ISR) — see encoder_test.c for the why.
static volatile int64_t s_overflow_accu = 0;
static int64_t g_last_pos = 0;      // for per-sample velocity (fixed dt = SAMPLE_MS)
static int64_t g_t0_ms    = 0;      // run start, so the CSV timestamp starts near 0

static bool IRAM_ATTR on_watch_point(pcnt_unit_handle_t unit,
                                     const pcnt_watch_event_data_t *edata, void *ctx)
{
    s_overflow_accu += edata->watch_point_value;
    return false;
}

static int64_t read_position(pcnt_unit_handle_t unit)
{
    int hw = 0;
    pcnt_unit_get_count(unit, &hw);
    return s_overflow_accu + hw;
}

// ---- Motor helpers -----------------------------------------------------------------------------
static void set_pwm(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(PWM_MODE, ch, duty);
    ledc_update_duty(PWM_MODE, ch);
}

// SIGNED command: +duty = forward (PWM on IN1), -duty = reverse (PWM on IN2), 0 = COAST (both low).
// Coast (not brake) is what SPINDOWN needs: both pins low = DRV8871 high-Z, so only friction slows it.
static void set_motor(int duty)
{
    if (duty >  255) duty =  255;
    if (duty < -255) duty = -255;
    if (duty >= 0) { set_pwm(IN1_CHANNEL, (uint32_t)duty);  set_pwm(IN2_CHANNEL, 0); }
    else           { set_pwm(IN1_CHANNEL, 0);  set_pwm(IN2_CHANNEL, (uint32_t)(-duty)); }
}

// ---- One CSV sample. Call EXACTLY every SAMPLE_MS so the fixed-dt velocity is valid. ------------
// Columns: BAM, t_ms, phase, duty(-255..255), pos_counts, vel_rad_s
static float sample_and_log(const char *phase, int duty, pcnt_unit_handle_t enc)
{
    int64_t pos   = read_position(enc);
    int64_t delta = pos - g_last_pos;
    g_last_pos    = pos;

    float rad_s = ((float)delta / COUNTS_PER_REV) * 6.283185f * (1000.0f / (float)SAMPLE_MS);
    int64_t ms  = (esp_timer_get_time() / 1000) - g_t0_ms;

    // Raw printf (not ESP_LOGI) so the line is clean CSV with no "I (123) tag:" decoration.
    printf("BAM,%lld,%s,%d,%lld,%.3f\n",
           (long long)ms, phase, duty, (long long)pos, rad_s);
    return rad_s;
}

// Hold a fixed duty for ms milliseconds, logging every SAMPLE_MS. Returns the last velocity seen.
static float hold(const char *phase, int duty, int ms, pcnt_unit_handle_t enc)
{
    set_motor(duty);
    float v = 0.0f;
    int n = ms / SAMPLE_MS;
    for (int i = 0; i < n; i++) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
        v = sample_and_log(phase, duty, enc);
    }
    return v;
}

// ---- Sweep 1: STAIRCASE ------------------------------------------------------------------------
// Each duty is held long enough to (a) show the spin-up transient then (b) settle to steady state.
// Both directions, because friction is not perfectly symmetric.
static void sweep_staircase(pcnt_unit_handle_t enc)
{
    static const int duties[] = { 60, 90, 120, 150, 180, 210, 255 };
    const int HOLD_MS = 1500;

    for (int s = 0; s < 2; s++) {                 // s=0 forward, s=1 reverse
        int sign = s == 0 ? +1 : -1;
        for (int i = 0; i < (int)(sizeof(duties)/sizeof(duties[0])); i++) {
            hold("stair", sign * duties[i], HOLD_MS, enc);
        }
        hold("stair", 0, 600, enc);               // coast to rest between directions
    }
}

// ---- Sweep 2: SPINDOWN -------------------------------------------------------------------------
// Reach a good speed, then command 0 (coast) and record the deceleration — friction only, no drive.
static void sweep_spindown(pcnt_unit_handle_t enc)
{
    for (int s = 0; s < 2; s++) {
        int sign = s == 0 ? +1 : -1;
        hold("spinup",   sign * 220, 1500, enc);  // get up to speed
        hold("coast",    0,          1800, enc);  // <-- the money data: pure friction decel
    }
}

// ---- Sweep 3: DEADBAND -------------------------------------------------------------------------
// Creep the duty up slowly; the data will show the exact duty where motion begins (breakaway).
// We also print a one-off marker when we first detect real motion, as a convenience.
static void sweep_deadband(pcnt_unit_handle_t enc)
{
    const float MOVING_RAD_S = 0.5f;              // "it's really turning" threshold
    for (int s = 0; s < 2; s++) {
        int sign = s == 0 ? +1 : -1;
        bool announced = false;
        for (int d = 30; d <= 160; d += 2) {
            float v = hold("deadband", sign * d, 200, enc);
            if (!announced && (v > MOVING_RAD_S || v < -MOVING_RAD_S)) {
                printf("BAM,MARK,breakaway,%d,0,%.3f\n", sign * d, v);
                announced = true;                 // keep sweeping to map the low-speed regime
            }
        }
        hold("deadband", 0, 600, enc);            // rest before the other direction
    }
}

// ---- Init (LEDC + PCNT, straight from the two test files) --------------------------------------
static void motor_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE, .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RES, .freq_hz = PWM_FREQ_HZ, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t in1 = {
        .speed_mode = PWM_MODE, .channel = IN1_CHANNEL, .timer_sel = PWM_TIMER,
        .gpio_num = IN1_GPIO, .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&in1));
    ledc_channel_config_t in2 = in1; in2.channel = IN2_CHANNEL; in2.gpio_num = IN2_GPIO;
    ESP_ERROR_CHECK(ledc_channel_config(&in2));

    // Required or duty sticks at 0 after the first change — see the ledc-fade gotcha memory.
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
}

static pcnt_unit_handle_t encoder_init(void)
{
    pcnt_unit_config_t unit_cfg = { .high_limit = PCNT_HIGH_LIMIT, .low_limit = PCNT_LOW_LIMIT };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = GLITCH_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_cfg));

    pcnt_chan_config_t chan_a_cfg = { .edge_gpio_num = ENC_A_GPIO, .level_gpio_num = ENC_B_GPIO };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_cfg, &chan_a));
    pcnt_chan_config_t chan_b_cfg = { .edge_gpio_num = ENC_B_GPIO, .level_gpio_num = ENC_A_GPIO };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_b_cfg, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, PCNT_LOW_LIMIT));
    pcnt_event_callbacks_t cbs = { .on_reach = on_watch_point };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(unit, &cbs, NULL));

    gpio_set_pull_mode(ENC_A_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENC_B_GPIO, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

void app_main(void)
{
    motor_init();
    pcnt_unit_handle_t enc = encoder_init();
    set_motor(0);

    // Give you time to start logging serial to a file before data starts.
    ESP_LOGI(TAG, "BAM sweep starting in 3 s — start your capture now.");
    for (int i = 3; i > 0; i--) { ESP_LOGI(TAG, "  %d...", i); vTaskDelay(pdMS_TO_TICKS(1000)); }

    g_t0_ms    = esp_timer_get_time() / 1000;
    g_last_pos = read_position(enc);
    // CSV header (a comment line the parser can skip; every real row starts with "BAM,").
    printf("BAM,HEADER,phase,duty,pos_counts,vel_rad_s  (V_BUS=%.1fV COUNTS_PER_REV=%.0f dt=%dms)\n",
           V_BUS_VOLTS, COUNTS_PER_REV, SAMPLE_MS);

    sweep_staircase(enc);
    sweep_spindown(enc);
    sweep_deadband(enc);

    set_motor(0);
    printf("BAM,DONE,end,0,%lld,0.000\n", (long long)read_position(enc));
    ESP_LOGI(TAG, "Sweep complete. Motor coasting. Press EN/RST to run again.");
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
