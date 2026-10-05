#pragma once

#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"


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


typedef struct {
    sh2_Hal_t hal;
    spi_device_handle_t spi;
    bool bus_initialized;
    bool device_added;
    uint8_t pending_rx[SH2_HAL_MAX_TRANSFER_OUT];
    size_t pending_rx_len;
    uint32_t pending_timestamp_us;
} esp32_sh2_hal_t;


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

extern latest_data_t s_latest;
extern bool s_runtime_reset;


// public API
sh2_Hal_t *make_hal(void);
void async_event_handler(void *cookie, sh2_AsyncEvent_t *event);
void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event);
bool enable_test_reports(void);
void log_product_ids(void);
void quaternion_to_euler(float r, float i, float j, float k, float *theta_x, float *theta_y, float *theta_z);