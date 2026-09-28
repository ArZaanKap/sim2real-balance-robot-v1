#pragma once

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal.h"


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



bool wait_for_int_low(uint32_t timeout_ms);
esp_err_t spi_exchange(const void *tx, void *rx, size_t len);
int hal_open(sh2_Hal_t *self);
void hal_close(sh2_Hal_t *self);
int copy_pending_read(uint8_t *buffer, unsigned capacity, uint32_t *timestamp_us);
int hal_read(sh2_Hal_t *self, uint8_t *buffer, unsigned capacity, uint32_t *timestamp_us);
int hal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len);
uint32_t hal_get_time_us(sh2_Hal_t *self);
sh2_Hal_t *make_hal(void);
void async_event_handler(void *cookie, sh2_AsyncEvent_t *event);
void sensor_event_handler(void *cookie, sh2_SensorEvent_t *event);
int enable_report(sh2_SensorId_t sensor_id, const char *name);
bool enable_test_reports(void);
void log_product_ids(void);
void quaternion_to_euler(float r, float i, float j, float k, float *theta_x, float *theta_y, float *theta_z)