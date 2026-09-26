// bringup.c — unified sensor/actuator check for the balancing robot.
// STEP 1: boot skeleton only. Prove the build assembles (sh2 + pcnt + ledc)
// before any subsystem code goes in. Fill the rest in yourself, subsystem by
// subsystem: IMU read, encoder x2, motor x2, then the printed dashboard.
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"

// compare with standalone files - make modular next
static const char *TAG = "bringup";

// pins + consts

// motor
#define MOT_L_IN1_GPIO 27
#define MOT_L_IN2_GPIO 14
#define MOT_R_IN1_GPIO 25
#define MOT_R_IN2_GPIO 26

#define PWM_FREQ_HZ 20000                        
#define PWM_RES  LEDC_TIMER_8_BIT     // 8-bit -> duty range 0..255 (same as Arduino)
#define COMMAND  180

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

// imu
#define IMU_SPI_HOST       SPI3_HOST
#define IMU_SCK_GPIO       GPIO_NUM_18
#define IMU_MISO_GPIO      GPIO_NUM_19
#define IMU_MOSI_GPIO      GPIO_NUM_23
#define IMU_CS_GPIO        GPIO_NUM_17
#define IMU_INT_GPIO       GPIO_NUM_21
#define IMU_RST_GPIO       GPIO_NUM_22
#define IMU_WAKE_GPIO      GPIO_NUM_16

// The BNO08X limit is 3 MHz. Two MHz conservative for breadboard and jumper wires.
#define IMU_SPI_HZ         2000000
#define RESET_HOLD_MS      10
#define READY_TIMEOUT_MS   2000
#define REPORT_INTERVAL_US 10000  // 100 Hz
#define SHTP_HEADER_LEN    4
#define RAD_TO_DEG         57.29577951308232f


// GLOBALS //

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


// IMU 
typedef struct {
    sh2_Hal_t hal;
    spi_device_handle_t spi;
    bool bus_initialized;
    bool device_added;
    uint8_t pending_rx[SH2_HAL_MAX_TRANSFER_OUT];
    size_t pending_rx_len;
    uint32_t pending_timestamp_us;
} esp32_sh2_hal_t;

static esp32_sh2_hal_t s_hal;
// DMA reads directly from this buffer during long inbound SHTP packets, so it
// must live in internal DMA-capable RAM rather than flash-backed const data.
static DMA_ATTR uint8_t s_tx_zeroes[SH2_HAL_MAX_TRANSFER_IN] = {0};

typedef struct {
    bool have_quaternion;
    bool have_accel;
    bool have_gyro;
    uint8_t accuracy;
    uint32_t sample_count;
    float real;
    float i;
    float j;
    float k;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} latest_data_t;

static latest_data_t s_latest;
static bool s_runtime_reset;


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


// ----- IMU FUNCTIONS -------- //
static bool wait_for_int_low(uint32_t timeout_ms)
{
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000);
    while (gpio_get_level(IMU_INT_GPIO) != 0) {
        if (esp_timer_get_time() >= deadline_us) {
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

static esp_err_t spi_exchange(const void *tx, void *rx, size_t len)
{
    spi_transaction_t transaction = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_polling_transmit(s_hal.spi, &transaction);
}

static int hal_open(sh2_Hal_t *self){
    (void)self;

    // Assert reset before setting the SPI-mode strap.  The module may have
    // briefly powered up in its default I2C mode before the ESP32 booted.
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << IMU_CS_GPIO) |
                        (1ULL << IMU_RST_GPIO) |
                        (1ULL << IMU_WAKE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&output_config) != ESP_OK) {
        return SH2_ERR_IO;
    }
    gpio_set_level(IMU_RST_GPIO, 0);
    gpio_set_level(IMU_CS_GPIO, 1);
    gpio_set_level(IMU_WAKE_GPIO, 1);  // PS0=1; external PS1 must also be 1.

    gpio_config_t input_config = {
        .pin_bit_mask = 1ULL << IMU_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&input_config) != ESP_OK) {
        return SH2_ERR_IO;
    }

    spi_bus_config_t bus_config = {
        .mosi_io_num = IMU_MOSI_GPIO,
        .miso_io_num = IMU_MISO_GPIO,
        .sclk_io_num = IMU_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SH2_HAL_MAX_TRANSFER_IN,
    };
    // Without DMA the classic ESP32 SPI driver caps transactions at 64 bytes;
    // BNO08X startup and sensor packets can be considerably larger.
    esp_err_t err = spi_bus_initialize(IMU_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return SH2_ERR_IO;
    }
    s_hal.bus_initialized = true;

    // CS is driven manually because a BNO08X read uses two transfers (header
    // then body) while CS must remain low between them.
    spi_device_interface_config_t device_config = {
        .clock_speed_hz = IMU_SPI_HZ,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    err = spi_bus_add_device(IMU_SPI_HOST, &device_config, &s_hal.spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        spi_bus_free(IMU_SPI_HOST);
        s_hal.bus_initialized = false;
        return SH2_ERR_IO;
    }
    s_hal.device_added = true;
    s_hal.pending_rx_len = 0;

    vTaskDelay(pdMS_TO_TICKS(RESET_HOLD_MS));
    gpio_set_level(IMU_RST_GPIO, 1);

    ESP_LOGI(TAG, "reset released with PS1=HIGH and PS0=HIGH; waiting for INT LOW");
    if (!wait_for_int_low(READY_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "BNO08X did not assert INT within %d ms", READY_TIMEOUT_MS);
        ESP_LOGE(TAG, "check PS1=3.3V, PS0=GPIO16, INT=GPIO21 and RST=GPIO22");
        return SH2_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "INT is LOW: BNO08X has completed reset and is ready");
    return SH2_OK;
}

static void hal_close(sh2_Hal_t *self){
    (void)self;
    gpio_set_level(IMU_RST_GPIO, 0);
    gpio_set_level(IMU_CS_GPIO, 1);

    if (s_hal.device_added) {
        spi_bus_remove_device(s_hal.spi);
        s_hal.device_added = false;
    }
    if (s_hal.bus_initialized) {
        spi_bus_free(IMU_SPI_HOST);
        s_hal.bus_initialized = false;
    }
}

static int copy_pending_read(uint8_t *buffer, unsigned capacity, uint32_t *timestamp_us){
    if (s_hal.pending_rx_len == 0) {
        return 0;
    }
    if (capacity < s_hal.pending_rx_len) {
        s_hal.pending_rx_len = 0;
        return SH2_ERR_BAD_PARAM;
    }

    size_t len = s_hal.pending_rx_len;
    memcpy(buffer, s_hal.pending_rx, len);
    *timestamp_us = s_hal.pending_timestamp_us;
    s_hal.pending_rx_len = 0;
    return (int)len;
}

static int hal_read(sh2_Hal_t *self, uint8_t *buffer, unsigned capacity,
                    uint32_t *timestamp_us)
{
    (void)self;
    if ((buffer == NULL) || (timestamp_us == NULL)) {
        return SH2_ERR_BAD_PARAM;
    }

    int pending = copy_pending_read(buffer, capacity, timestamp_us);
    if (pending != 0) {
        return pending;
    }
    if (gpio_get_level(IMU_INT_GPIO) != 0) {
        return 0;
    }

    *timestamp_us = (uint32_t)esp_timer_get_time();
    gpio_set_level(IMU_CS_GPIO, 0);

    esp_err_t err = spi_exchange(s_tx_zeroes, buffer, SHTP_HEADER_LEN);
    if (err != ESP_OK) {
        gpio_set_level(IMU_CS_GPIO, 1);
        ESP_LOGE(TAG, "SPI header read failed: %s", esp_err_to_name(err));
        return SH2_ERR_IO;
    }

    size_t packet_len = ((size_t)buffer[0] | ((size_t)buffer[1] << 8)) & 0x7FFF;
    if ((packet_len <= SHTP_HEADER_LEN) || (packet_len > capacity)) {
        gpio_set_level(IMU_CS_GPIO, 1);
        if (packet_len > capacity) {
            ESP_LOGE(TAG, "incoming SHTP packet (%u bytes) exceeds buffer (%u)",
                     (unsigned)packet_len, capacity);
            return SH2_ERR_BAD_PARAM;
        }
        return 0;
    }

    err = spi_exchange(s_tx_zeroes, buffer + SHTP_HEADER_LEN,
                       packet_len - SHTP_HEADER_LEN);
    gpio_set_level(IMU_CS_GPIO, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI body read failed: %s", esp_err_to_name(err));
        return SH2_ERR_IO;
    }

    return (int)packet_len;
}

static int hal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len){
    (void)self;
    if ((buffer == NULL) || (len < SHTP_HEADER_LEN) ||
        (len > SH2_HAL_MAX_TRANSFER_OUT)) {
        return SH2_ERR_BAD_PARAM;
    }

    uint8_t simultaneous_rx[SH2_HAL_MAX_TRANSFER_OUT] = {0};

    // PS0 becomes active-low WAKE after startup.  The sensor responds by
    // asserting active-low INT when it is ready for the host transaction.
    gpio_set_level(IMU_WAKE_GPIO, 0);
    if (!wait_for_int_low(READY_TIMEOUT_MS)) {
        gpio_set_level(IMU_WAKE_GPIO, 1);
        ESP_LOGE(TAG, "timeout waiting for INT before %u-byte write", len);
        return SH2_ERR_TIMEOUT;
    }

    s_hal.pending_timestamp_us = (uint32_t)esp_timer_get_time();
    gpio_set_level(IMU_CS_GPIO, 0);
    gpio_set_level(IMU_WAKE_GPIO, 1);
    esp_err_t err = spi_exchange(buffer, simultaneous_rx, len);
    gpio_set_level(IMU_CS_GPIO, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(err));
        return SH2_ERR_IO;
    }

    // SPI is full-duplex.  Preserve a complete inbound packet if one arrived
    // while the host was writing, matching CEVA's reference HAL behaviour.
    size_t rx_len = ((size_t)simultaneous_rx[0] |
                     ((size_t)simultaneous_rx[1] << 8)) & 0x7FFF;
    if ((rx_len > SHTP_HEADER_LEN) && (rx_len <= len) &&
        (rx_len <= sizeof(s_hal.pending_rx))) {
        memcpy(s_hal.pending_rx, simultaneous_rx, rx_len);
        s_hal.pending_rx_len = rx_len;
    }

    return (int)len;
}

static uint32_t hal_get_time_us(sh2_Hal_t *self){
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

static sh2_Hal_t *make_hal(void){
    memset(&s_hal, 0, sizeof(s_hal));
    s_hal.hal.open = hal_open;
    s_hal.hal.close = hal_close;
    s_hal.hal.read = hal_read;
    s_hal.hal.write = hal_write;
    s_hal.hal.getTimeUs = hal_get_time_us;
    return &s_hal.hal;
}


void app_main(void){
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
