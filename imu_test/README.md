# BNO08X SPI smoke test

This standalone ESP-IDF project verifies the `GY-BNO08X` breakout before it is
used in the balancing controller. Keep the 12 V motor supply disconnected.

## Wiring

| GY-BNO08X | ESP32-WROOM |
|---|---:|
| VCC | 3.3 V |
| GND | GND |
| SCL | GPIO18 (SCK) |
| SDA | GPIO19 (MISO) |
| AD0 | GPIO23 (MOSI) |
| CS | GPIO17 |
| INT | GPIO21 |
| RST | GPIO22 |
| PS1 | 3.3 V |
| PS0 | GPIO16 (WAKE) |
| BOOT | Leave disconnected |

## Build, flash, and monitor

Run these from an ESP-IDF shell:

```bash
cd /home/arzaan/PROJECTS/balancing-2-wheel-robot/imu_test
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Replace `/dev/ttyUSB0` if the ESP32 appears under a different serial device.
Exit the monitor with `Ctrl-]`.

## Pass criteria

The first decisive line is `PRODUCT OK`, containing the BNO08X software and
part information. After that, three reports are enabled at 100 Hz and the
monitor prints at 10 Hz.

- Stationary acceleration magnitude (`|a|`) should be close to 9.81 m/s2.
- Stationary gyro values should settle near zero rad/s.
- Tilt the board slowly: pitch or roll should move smoothly by roughly the
  angle through which the board was moved.
- Accuracy is 0=unreliable, 1=low, 2=medium, 3=high. It can start low while
  the sensor calibrates.

For the robot, note which of `pitch` or `roll` changes when the chassis rocks
forward and backward. That is the angle the balance controller must use.

## Failure clues

- `did not assert INT`: check PS1=3.3 V, PS0=GPIO16, INT=GPIO21 and RST=GPIO22.
- Product-ID failure: first check the easy-to-swap lines: SDA is MISO and AD0
  is MOSI in SPI mode.
- Repeated resets: verify a solid 3.3 V supply and short ground connection.
- The bus must use SPI mode 3. This test runs at 2 MHz, below the 3 MHz limit.

The SH-2 protocol implementation is CEVA's official driver, vendored under
`components/sh2`. `main/imu_test.c` supplies the ESP-IDF SPI transport.
