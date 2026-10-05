#include "encoder.h"

#include "esp_log.h"
#include "driver/gpio.h"


#define PCNT_HIGH_LIMIT 3000 // max till counter reset to 0
#define PCNT_LOW_LIMIT (-3000) // min till counter reset to 0
#define GLITCH_NS 1000  // <N nanosecs then ignore encoder reading


static const char *TAG = "encoder";


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
int64_t read_position(encoder_t *e){
    int hw = 0; // hardware counter
    pcnt_unit_get_count(e->unit, &hw); // place count from unit into hw -> thats why & used
    return e->overflow + hw;
}

void encoder_init(encoder_t *e, int a_gpio, int b_gpio){

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

