# Static IMU noise measurement

This tool measures the two BNO085 signals used by the balancing policy:

- `theta_y_rad`: the fused balance angle, in radians
- `gyro_y_rad_s`: the calibrated Y-axis gyro rate, in radians/second

The ESP32 samples the latest values at the same 100 Hz rate as the controller.
It emits one machine-readable line per control tick. The host scripts capture
those rows and calculate mean, population standard deviation, percentiles, and
the equivalent `uniform(-a, a)` half-range for the current simulation model.

The project deliberately reuses `pid/main/imu.c` and `pid/components/sh2` so the
axis extraction, sensor reports, and SPI transport are identical to the real
controller. It does not build or drive the motors.

## 1. Build and flash

Keep the 12 V motor supply **off**. Attach the CH340 to WSL first if necessary.

```bash
source idf-env.sh
cd imu_noise
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

Flashing is the human hardware step. The firmware output has this form:

```text
IMU,tick,time_us,sample_count,theta_y_rad,gyro_y_rad_s,accuracy
```

The literal header above is explanatory; firmware data rows contain numeric
values after `IMU,`.

## 2. Mount the robot

Rigidly support the complete robot at its known physical upright angle. Do not
hold it by hand: hand motion would be measured as sensor noise. Avoid a soft or
wobbly support. Leave the motors unpowered and let the IMU settle.

The measured `theta_y` mean represents mounting offset only to the extent that
the fixture really is upright. A fixture error and an IMU mounting error are
indistinguishable in one static recording.

## 3. Capture

From another terminal, after flashing:

```bash
cd imu_noise
python3 capture.py --port /dev/ttyUSB0 --warmup 10 --duration 120
```

This uses only Python's standard library plus the system `stty` command. It
discards ten seconds, records two minutes, and writes a timestamped CSV under
`logs/`. Use `Ctrl-C` to stop early. Set `--duration 0` to record until stopped.

## 4. Analyse

Pass the CSV path printed by the capture script:

```bash
python3 analyze.py logs/imu_static_YYYYMMDD_HHMMSS.csv
```

The analyser prints the results and creates a neighbouring `.stats.json` file.
For each signal, `uniform +/- a` is calculated as:

```text
a = sqrt(3) * measured_standard_deviation
```

That conversion matters because a uniform distribution over `[-a, a]` has
standard deviation `a / sqrt(3)`. Putting the measured standard deviation
directly into `uniform(-a, a)` would make the simulated noise too small.

Do not blindly treat the reported means as per-sample noise. The angle mean is
a fixed fixture/mount offset and the gyro mean is a stationary bias. They need
separate modelling or calibration from the zero-mean sample jitter.
