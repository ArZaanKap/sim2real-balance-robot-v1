// encoder_test.c — read ONE GA25-370 (JGA25-370) hall quadrature encoder with the ESP32 PCNT peripheral.
//
// Goal: prove we can measure wheel POSITION + VELOCITY in hardware — no missed counts, ~0 CPU.
// This is the SENSOR half of the velocity loop that will later close around the DRV8871 motor drive
// (motor_test.c is the actuator half). Bench test: no chassis needed, spin the shaft by hand.
//
// Pattern mirrors motor_test.c / esp_link.c: one .c authored here, built + flashed on the Pi.
//
// WHY PCNT (and why the classic ESP32-WROOM, not the C3): PCNT is a hardware pulse counter.
// Arduino analogy: it's like doing attachInterrupt(A, ..., CHANGE) + attachInterrupt(B, ...) to
// count an encoder — but done entirely in silicon. It NEVER misses an edge even at high RPM, and
// costs the CPU nothing: we just read a register. (The C3 has no PCNT — that's why we're on WROOM.)
//
// ---- Wiring (ONE encoder to start) -------------------------------------------------------------
//   Encoder channel A  -> GPIO 32
//   Encoder channel B  -> GPIO 33
//   Encoder VCC        -> 3.3V   (NOT 5V — these hall outputs would swing 5V into a 3.3V GPIO)
//   Encoder GND        -> GND    (COMMON ground with the ESP32)
//   Motor +/- wires    -> go to the DRV8871, NOT here. This test only reads the sensor.
// GA25 encoders usually have 6 wires: motor+ , motor- , VCC , GND , A , B. Only VCC/GND/A/B come here.
// ------------------------------------------------------------------------------------------------
//
// Quadrature: A and B are square waves 90 deg out of phase. Which one LEADS tells us direction.
// We decode all four edges ("4x mode") for maximum resolution.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "encoder_test";

// --- Pins (one encoder). Chosen because 32/33 support internal pull-ups and are not strapping pins.
#define ENC_A_GPIO      32
#define ENC_B_GPIO      33

// --- PCNT hardware counter limits ---------------------------------------------------------------
// The hardware counter is only 16-bit. We set "watch points" at these limits; when the count hits
// one, the hardware resets to 0 and fires a callback where we add the limit to a 64-bit accumulator.
// Net effect: an effectively unbounded position counter. +/-3000 keeps callbacks rare (a few/sec).
#define PCNT_HIGH_LIMIT  3000
#define PCNT_LOW_LIMIT  (-3000)

// --- Glitch filter (hardware debounce) ----------------------------------------------------------
// Ignore pulses shorter than this — electrical noise. Memory says ~2900 edges/s at full speed, so
// real edges are ~300 us apart; 1 us is safely below the signal and above the noise.
#define GLITCH_NS        1000

// --- Encoder scaling — MEASURE THIS, do not trust the datasheet. --------------------------------
// counts_per_output_rev = (hall lines) x 4 (quadrature) x (gearbox ratio).
// For the 130 RPM 25GA the ballpark is 11 x 4 x ~46.8 ~= 2060, but ratios vary per batch.
// CALIBRATE (do this first): flash, open the monitor, rotate the OUTPUT shaft EXACTLY one full turn
// by hand, read the "pos" value. That number is your COUNTS_PER_REV. RPM/rad-s below are only as
// correct as this constant.
#define COUNTS_PER_REV   1976.0f   // MEASURED 2026-08-31 (13 poles x4 x 38:1 gearbox); was a 2060 guess

// --- Sampling rate for the velocity print -------------------------------------------------------
#define SAMPLE_MS        20        // 50 Hz — the rate the real velocity loop will run at

// Position accumulator, written from the watch-point ISR, read from the sample task.
// volatile because the ISR changes it behind the task's back.
static volatile int64_t s_overflow_accu = 0;

// Runs in ISR context when the counter hits a watch point (+/- limit). Keep it tiny.
// The hardware has already reset the count to 0; we just remember how far we'd gone.
static bool IRAM_ATTR on_watch_point(pcnt_unit_handle_t unit,
                                     const pcnt_watch_event_data_t *edata,
                                     void *user_ctx)
{
    s_overflow_accu += edata->watch_point_value;   // += +3000 or += -3000
    return false;                                  // no task woken -> no context switch needed
}

// Full 64-bit position = accumulated overflows + whatever is currently in the 16-bit counter.
// NOTE: for the real control loop, read these two inside a short portENTER_CRITICAL() so an
// overflow can't land between them. For a hand-spin bench test the tiny race is harmless.
static int64_t read_position(pcnt_unit_handle_t unit)
{
    int hw = 0;
    pcnt_unit_get_count(unit, &hw);
    return s_overflow_accu + hw;
}

static pcnt_unit_handle_t encoder_init(void)
{
    // 1) Create the counter unit with its wrap-around limits.
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    // 2) Hardware glitch filter (debounce).
    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = GLITCH_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_cfg));

    // 3) Two channels for 4x quadrature decode.
    //    Channel A watches EDGES on A, and reads the LEVEL of B to decide up/down.
    //    Channel B watches EDGES on B, and reads the LEVEL of A. Together = all 4 edges.
    pcnt_chan_config_t chan_a_cfg = { .edge_gpio_num = ENC_A_GPIO, .level_gpio_num = ENC_B_GPIO };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_cfg, &chan_a));

    pcnt_chan_config_t chan_b_cfg = { .edge_gpio_num = ENC_B_GPIO, .level_gpio_num = ENC_A_GPIO };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_b_cfg, &chan_b));

    // 4) The quadrature truth table, expressed as edge + level actions.
    //    (This is the canonical ESP-IDF rotary-encoder config: count on both edges of both channels,
    //     direction flips based on the other channel's level.)
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));   // A rising/falling
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));      // gated by B
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));   // B rising/falling
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));      // gated by A
    // If the sign comes out backwards vs the direction you spin, swap A and B wires (or swap the
    // INCREASE/DECREASE pair on one channel). Easiest fix is at the connector.

    // 5) Watch points at the limits so overflow accumulation works, plus the callback.
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, PCNT_LOW_LIMIT));
    pcnt_event_callbacks_t cbs = { .on_reach = on_watch_point };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(unit, &cbs, NULL));

    // 6) Internal pull-ups on A/B. Harmless if the encoder drives push-pull; protective if it's
    //    open-collector or a wire falls off (keeps the line from floating and false-counting).
    gpio_set_pull_mode(ENC_A_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENC_B_GPIO, GPIO_PULLUP_ONLY);

    // 7) Enable, zero, go.
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));

    ESP_LOGI(TAG, "PCNT encoder ready: A=GPIO%d B=GPIO%d, 4x quadrature, filter %d ns",
             ENC_A_GPIO, ENC_B_GPIO, GLITCH_NS);
    return unit;
}

void app_main(void)
{
    pcnt_unit_handle_t enc = encoder_init();

    int64_t last_pos = read_position(enc);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));

        int64_t pos   = read_position(enc);
        int64_t delta = pos - last_pos;          // counts since last sample
        last_pos = pos;

        // counts -> revs -> speed. dt is SAMPLE_MS milliseconds.
        float revs   = (float)delta / COUNTS_PER_REV;
        float rpm    = revs * (60000.0f / (float)SAMPLE_MS);
        float rad_s  = revs * 6.283185f * (1000.0f / (float)SAMPLE_MS);

        // pos = absolute count (use to CALIBRATE COUNTS_PER_REV: one hand-turn of the output shaft).
        // Prints ~50x/sec; comment the log or slow SAMPLE_MS if the monitor is too busy.
        ESP_LOGI(TAG, "pos=%lld  delta=%lld  rpm=%.1f  rad/s=%.2f",
                 (long long)pos, (long long)delta, rpm, rad_s);
    }
}
