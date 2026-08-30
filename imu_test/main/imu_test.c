// imu_test.c -- BNO08X SPI bring-up and motion test for the ESP32-WROOM.
//
// Keep the motor 12 V supply disconnected for this test.  Expected wiring:
//   GY-BNO08X VCC -> 3.3 V       GND -> GND
//   SCL/SCK       -> GPIO18      SDA/MISO -> GPIO19
//   AD0/MOSI      -> GPIO23      CS       -> GPIO17
//   INT           -> GPIO21      RST      -> GPIO22
//   PS1           -> 3.3 V       PS0/WAKE -> GPIO16
//   BOOT          -> not connected

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"

static const char *TAG = "imu_test";

// SPI3 is the classic ESP32's VSPI controller.  These are its usual pins.
#define IMU_SPI_HOST       SPI3_HOST
#define IMU_SCK_GPIO       GPIO_NUM_18
#define IMU_MISO_GPIO      GPIO_NUM_19
#define IMU_MOSI_GPIO      GPIO_NUM_23
#define IMU_CS_GPIO        GPIO_NUM_17
#define IMU_INT_GPIO       GPIO_NUM_21
#define IMU_RST_GPIO       GPIO_NUM_22
#define IMU_WAKE_GPIO      GPIO_NUM_16

// The BNO08X limit is 3 MHz.  Two MHz is deliberately conservative for a
// breadboard and jumper wires.
#define IMU_SPI_HZ         2000000
#define RESET_HOLD_MS      10
#define READY_TIMEOUT_MS   2000
#define REPORT_INTERVAL_US 10000  // 100 Hz
#define PRINT_INTERVAL_MS  100    // 10 readable lines/second

#define SHTP_HEADER_LEN    4
#define RAD_TO_DEG         57.29577951308232f

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

static int hal_open(sh2_Hal_t *self)
{
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

static void hal_close(sh2_Hal_t *self)
{
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

static int copy_pending_read(uint8_t *buffer, unsigned capacity, uint32_t *timestamp_us)
{
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

static int hal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len)
{
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

static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

static sh2_Hal_t *make_hal(void)
{
    memset(&s_hal, 0, sizeof(s_hal));
    s_hal.hal.open = hal_open;
    s_hal.hal.close = hal_close;
    s_hal.hal.read = hal_read;
    s_hal.hal.write = hal_write;
    s_hal.hal.getTimeUs = hal_get_time_us;
    return &s_hal.hal;
}

static void async_event_handler(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event->eventId == SH2_RESET) {
        s_runtime_reset = true;
        ESP_LOGW(TAG, "sensor-hub reset event received");
    } else if (event->eventId == SH2_SHTP_EVENT) {
        ESP_LOGW(TAG, "SHTP transport event: %d", event->shtpEvent);
    }
}

static void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        return;
    }

    switch (value.sensorId) {
    case SH2_GAME_ROTATION_VECTOR:
        s_latest.real = value.un.gameRotationVector.real;
        s_latest.i = value.un.gameRotationVector.i;
        s_latest.j = value.un.gameRotationVector.j;
        s_latest.k = value.un.gameRotationVector.k;
        s_latest.accuracy = value.status;
        s_latest.have_quaternion = true;
        s_latest.sample_count++;
        break;
    case SH2_ACCELEROMETER:
        s_latest.ax = value.un.accelerometer.x;
        s_latest.ay = value.un.accelerometer.y;
        s_latest.az = value.un.accelerometer.z;
        s_latest.have_accel = true;
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        s_latest.gx = value.un.gyroscope.x;
        s_latest.gy = value.un.gyroscope.y;
        s_latest.gz = value.un.gyroscope.z;
        s_latest.have_gyro = true;
        break;
    default:
        break;
    }
}

static int enable_report(sh2_SensorId_t sensor_id, const char *name)
{
    sh2_SensorConfig_t config = {0};
    config.reportInterval_us = REPORT_INTERVAL_US;
    int rc = sh2_setSensorConfig(sensor_id, &config);
    if (rc == SH2_OK) {
        ESP_LOGI(TAG, "enabled %s at %d Hz", name,
                 1000000 / REPORT_INTERVAL_US);
    } else {
        ESP_LOGE(TAG, "failed to enable %s: SH2 error %d", name, rc);
    }
    return rc;
}

static bool enable_test_reports(void)
{
    bool ok = true;
    ok &= enable_report(SH2_GAME_ROTATION_VECTOR, "game rotation vector") == SH2_OK;
    ok &= enable_report(SH2_ACCELEROMETER, "calibrated accelerometer") == SH2_OK;
    ok &= enable_report(SH2_GYROSCOPE_CALIBRATED, "calibrated gyroscope") == SH2_OK;
    return ok;
}

static void log_product_ids(void)
{
    sh2_ProductIds_t ids = {0};
    int rc = sh2_getProdIds(&ids);
    if ((rc != SH2_OK) || (ids.numEntries == 0)) {
        ESP_LOGE(TAG, "product-ID request failed: SH2 error %d", rc);
        ESP_LOGE(TAG, "SPI did not complete a valid SH-2 exchange");
        for (;;) {
            ESP_LOGE(TAG, "levels: INT=%d RST=%d PS0/WAKE=%d; check SDA=MISO and AD0=MOSI",
                     gpio_get_level(IMU_INT_GPIO), gpio_get_level(IMU_RST_GPIO),
                     gpio_get_level(IMU_WAKE_GPIO));
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    for (int n = 0; n < ids.numEntries; ++n) {
        const sh2_ProductId_t *id = &ids.entry[n];
        ESP_LOGI(TAG,
                 "PRODUCT OK [%d]: SW %u.%u.%u, part 0x%08lx, build %lu, reset cause %u",
                 n, id->swVersionMajor, id->swVersionMinor, id->swVersionPatch,
                 (unsigned long)id->swPartNumber,
                 (unsigned long)id->swBuildNumber, id->resetCause);
    }
}

static void quaternion_to_euler(float r, float i, float j, float k,
                                float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f * i * j - 2.0f * r * k,
                  2.0f * r * r + 2.0f * j * j - 1.0f);

    float pitch_arg = 2.0f * j * k + 2.0f * r * i;
    pitch_arg = fmaxf(-1.0f, fminf(1.0f, pitch_arg));
    *pitch = asinf(pitch_arg);

    *roll = atan2f(-2.0f * i * k + 2.0f * r * j,
                   2.0f * r * r + 2.0f * k * k - 1.0f);
}

static void print_latest_data(void)
{
    if (!s_latest.have_quaternion) {
        ESP_LOGW(TAG, "waiting for first orientation report...");
        return;
    }

    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    quaternion_to_euler(s_latest.real, s_latest.i, s_latest.j, s_latest.k,
                        &yaw, &pitch, &roll);
    float accel_magnitude = sqrtf(s_latest.ax * s_latest.ax +
                                  s_latest.ay * s_latest.ay +
                                  s_latest.az * s_latest.az);

    ESP_LOGI(TAG,
             "n=%lu acc=%u yaw=%7.2f pitch=%7.2f roll=%7.2f deg | "
             "a=[%6.2f %6.2f %6.2f] |a|=%5.2f m/s2 | "
             "g=[%6.3f %6.3f %6.3f] rad/s",
             (unsigned long)s_latest.sample_count, s_latest.accuracy,
             yaw * RAD_TO_DEG, pitch * RAD_TO_DEG, roll * RAD_TO_DEG,
             s_latest.ax, s_latest.ay, s_latest.az, accel_magnitude,
             s_latest.gx, s_latest.gy, s_latest.gz);
}

void app_main(void)
{
    ESP_LOGI(TAG, "BNO08X SPI test starting; keep the 12 V motor supply OFF");
    ESP_LOGI(TAG, "SCK=18 MISO=19 MOSI=23 CS=17 INT=21 RST=22 WAKE=16");
    ESP_LOGI(TAG, "SPI mode 3, %.1f MHz", IMU_SPI_HZ / 1000000.0);

    sh2_Hal_t *hal = make_hal();
    int rc = sh2_open(hal, async_event_handler, NULL);
    if (rc != SH2_OK) {
        ESP_LOGE(TAG, "sh2_open failed: %d", rc);
        ESP_LOGE(TAG, "power off and recheck every wire against the table in README.md");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    sh2_setSensorCallback(sensor_event_handler, NULL);
    log_product_ids();

    // Ignore the expected reset event from initial startup.
    s_runtime_reset = false;
    if (!enable_test_reports()) {
        ESP_LOGE(TAG, "one or more reports could not be enabled");
    }

    int64_t next_print_us = esp_timer_get_time();
    for (;;) {
        sh2_service();

        if (s_runtime_reset) {
            s_runtime_reset = false;
            ESP_LOGW(TAG, "re-enabling reports after sensor reset");
            enable_test_reports();
        }

        int64_t now_us = esp_timer_get_time();
        if (now_us >= next_print_us) {
            print_latest_data();
            next_print_us = now_us + ((int64_t)PRINT_INTERVAL_MS * 1000);
        }
        // One real scheduler tick is required here.  pdMS_TO_TICKS(1) rounds
        // down to zero with ESP-IDF's default 100 Hz FreeRTOS tick rate.
        vTaskDelay(1);
    }
}
