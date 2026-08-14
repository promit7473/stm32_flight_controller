# YMFC-32 Auto Flight Controller — Interview Prep Guide

## 1. Project Overview
- **YMFC-32** = Joop Brokking's autonomous drone flight controller
- **MCU**: STM32F103C8T6 (Blue Pill / Flip32) — ARM Cortex-M3, 72MHz
- **IDE**: Arduino IDE with STM32duino core
- **Loop rate**: 250 Hz (4 ms per loop), precisely enforced via `micros()` timing
- **Sensors**: MPU-6050 (IMU), HMC5883L (compass), MS5611 (barometer), u-blox GPS

---

## 2. Hardware & Sensors

### MPU-6050 (Gyro + Accelerometer)
- **I2C address**: `0x68`
- **Registers configured**:
  - `PWR_MGMT_1 (0x6B)` → 0x00 to wake up
  - `GYRO_CONFIG (0x1B)` → ±500°/s full scale
  - `ACCEL_CONFIG (0x1C)` → ±8g full scale
  - `CONFIG (0x1A)` → DLPF ~43Hz
- **Data read**: burst read 14 bytes from `0x3B` (accel first, then temp, then gyro)
- **Gyro sensitivity**: 65.5 LSB/(°/s) at ±500°/s

### HMC5883L (Compass / Magnetometer)
- **I2C address**: `0x1E`
- **Configuration Register A**: 0x78 (8 samples avg, 75Hz output rate)
- **Configuration Register B**: 0x20 (gain ±1.3Ga)
- **Mode Register**: 0x00 (continuous measurement)
- **Hard-iron calibration**: Stores min/max for each axis in EEPROM; calculates offset & scale
- **Tilt compensation**: Projects raw X/Y/Z to horizontal plane using `angle_pitch`/`angle_roll`
- **Heading**: `atan2(Y_horizontal, X_horizontal)` then convert radians → degrees
- **Declination**: Added to convert magnetic north → geographic north

### MS5611 (Barometer / Altitude)
- **I2C address**: `0x77`
- **6 PROM calibration coefficients** (C1–C6) read from `0xA2–0xAE`
- **Pressure/temp measurement cycle**: Request → wait 9ms → read 3 bytes
- **Rotating memory buffer**: 20-sample average for pressure smoothing
- **Complementary filter**: `actual_pressure_slow = slow * 0.985 + fast * 0.015`
- **Pressure → altitude**: Uses difference from `ground_pressure` captured at startup

### u-blox GPS (Serial UART)
- **Configuration**: UBX protocol to disable GPGSV, set 5Hz, set 57600 baud
- **NMEA parsing**: $GPGGA for lat/lon/sats, $GPGSA for fix type
- **Interpolation**: GPS at 5Hz → interpolated to 50Hz (9 intermediate values per cycle) using `gps_add_counter`
- **Rotating memory**: 35-sample buffer for D-term of GPS PD controller
- **Watchdog**: 1-second timeout — falls back to flight mode 2 if GPS lost

### Battery Monitoring
- **ADC**: 12-bit, reads voltage divider (1kΩ + 10kΩ = 1:11 ratio)
- **Scale**: 4095 counts = 36.3V → `battery_voltage = analogRead(4) / 112.81`
- **Filter**: Complementary filter `battery_voltage = V * 0.92 + new/1410.1`
- **Compensation**: When voltage < 12.4V, adds `(12.4 - V) * battery_compensation` to each ESC pulse

---

## 3. IMU & Sensor Fusion

### Gyro Integration (Dead Reckoning)
```
angle_pitch += gyro_pitch * 0.0000611  // 1/(250Hz * 65.5)
angle_roll  += gyro_roll  * 0.0000611
angle_yaw   += gyro_yaw   * 0.0000611
```

### Yaw-induced axis correction
```
angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066)  // PI/180 * 0.0000611
angle_roll  += angle_pitch * sin(gyro_yaw * 0.000001066)
```

### Complementary Filter (Gyro drift correction)
```
angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004
angle_roll  = angle_roll  * 0.9996 + angle_roll_acc  * 0.0004
```
- **Gyro** dominates high-frequency (short-term)
- **Accelerometer** corrects low-frequency drift (long-term)

### Key Interview Point: Why complementary filter vs Kalman?
**Answer**: Complementary filter is computationally cheap (no matrix operations), works well on resource-constrained MCUs, and is sufficient when gyro drift is slow and accel noise is high-frequency.

---

## 4. Control Theory — PID Controllers

### 4.1 Rate PID (Roll, Pitch, Yaw) — `calculate_pid()`

```
error = gyro_input - setpoint
I_mem += Ki * error
I_mem = constrain(I_mem, -max, max)
output = Kp * error + I_mem + Kd * (error - last_error)
output = constrain(output, -max, max)
last_error = error
```

| Axis   | Kp    | Ki    | Kd    | Max  |
|--------|-------|-------|-------|------|
| Roll   | 1.3   | 0.04  | 18.0  | 400  |
| Pitch  | 1.3   | 0.04  | 18.0  | 400  |
| Yaw    | 4.0   | 0.02  | 0.0   | 400  |

- **Deadband**: ±8µs around 1500µs center stick
- **Setpoint**: `(channel - 1500) / 3.0` → ~164°/s max rate
- **Leveling**: `pid_setpoint -= angle * 15` (angle correction)

### 4.2 Altitude PID — `Barometer.ino`
```
error = actual_pressure - setpoint
I_mem += (Ki/100) * error
I_mem = constrain(I_mem, -max, max)
output = (Kp + error_gain) * error + I_mem + Kd * parachute_throttle
```
- **Gain scheduling**: `error_gain` increases when `|error| > 10`
- **D-term**: Uses `parachute_throttle` (35-sample rotating buffer of pressure change) as velocity estimate
- **Manual altitude change**: Throttle >1600 or <1400 disables hold, adjusts setpoint

### 4.3 GPS Position Controller — `read_gps.ino`
```
gps_pitch_adjust = lat_error * Kp + lat_error_derivative * Kd
gps_roll_adjust  = lon_error * Kp + lon_error_derivative * Kd
```
- **PD only** (no I term)
- **Rotating buffer** (35 samples) for D-term
- **Heading rotation**: Converts north-aligned correction to current heading using `cos(angle_yaw)`

### 4.4 Motor Mixing (X-configuration)
```
esc_1 = throttle - pitch_out + roll_out - yaw_out   // front-right (CCW)
esc_2 = throttle + pitch_out + roll_out + yaw_out   // rear-right  (CW)
esc_3 = throttle + pitch_out - roll_out - yaw_out   // rear-left   (CCW)
esc_4 = throttle - pitch_out - roll_out + yaw_out   // front-left  (CW)
```

---

## 5. RC Receiver & ESC — Timer/ PWM

### Input Capture (Reading Receiver PPM/PWM) — `input_capture_mode_handlers.ino`
- **Timer 2** configured in input capture mode on `TI1`
- **PSC** = 71 (72MHz / 72 = 1µs per tick)
- Interrupt handler measures pulse width from CCR1
- PPM frames: sync gap > 3000µs resets channel counter; 6 channels decoded

### Output Compare (ESC PWM)
- **Timer 4**: 4 channels, PWM mode, PSC=71 (1µs resolution)
- **ARR** = 5000 (5ms period, fast update for 250Hz loop)
- **CCR1-4** = ESC pulse width (1000–2000µs)
- Direct register writes: `TIMER4_BASE->CCR1 = esc_1;`
- **Reset trick**: `TIMER4_BASE->CNT = 5000` forces immediate pulse update

---

## 6. Flight Modes & State Machine

| Mode | Ch5  | Ch6  | Behavior |
|------|------|------|----------|
| 1    | <1200| —    | Manual (rate mode) |
| 2    | 1200–1600| >1200 | Altitude hold |
| 3    | 1600–2100| >1200 | GPS position hold |

### Startup / Arm Sequence (`start_stop_takeoff.ino`)
1. **Start**: Throttle low + yaw left → `start = 1`
2. **Arm**: Yaw center → `start = 2`, motors at idle speed
3. **Takeoff detection**: Accelerometer vector change > 800 → `takeoff_detected = 1`
4. **Land/Stop**: Throttle low + yaw right → `start = 0`

### Takeoff Detection Detail
- Rolling average of acc_total_vector (25-sample short, 50-sample long)
- When short avg - start_vector > 800 → airborne
- Captures `takeoff_throttle = throttle - 1530` for hover compensation

---

## 7. Communication & Telemetry

- **Telemetry TX**: PB0, bit-banged serial (custom protocol)
- **GPS**: Serial1 (PA10/PA9), 9600 → 57600 baud
- **I2C**: `HWire (TwoWire HWire(2, I2C_FAST_MODE))` — second I2C at 400kHz
- Loop runs `send_telemetry_data()` every cycle

---

## 8. Error Signaling (Red LED)

| Error Code | Meaning |
|-----------|---------|
| 0         | No error |
| 1         | Low battery or battery disconnected |
| 2         | Gyro not responding, or loop > 4050µs |
| 3         | Compass not responding / level cal error |
| 4         | Receiver signal lost / GPS lost in mode 3 |
| 5         | Invalid manual takeoff throttle |
| 6         | Takeoff not detected at full throttle |
| 7         | Auto takeoff throttle outside range |

---

## 9. Calibration Procedures

### Gyro Calibration
- 2000 samples at 250Hz (~8 seconds)
- Average stored in `gyro_roll_cal`, `gyro_pitch_cal`, `gyro_yaw_cal`
- Subtracted from raw readings during operation

### Accelerometer Level Calibration
- Sticks top-left (ch1<1100, ch2<1100, ch3>1900, ch4<1100)
- 64 samples averaged → stored in EEPROM at `0x16`, `0x17`
- Subtracted from acc_y/acc_x during normal operation

### Compass Calibration
- Sticks top-right (ch1>1900, ch2<1100, ch3>1900, ch4>1900)
- Rotate drone 360° while ch2 held; records min/max per axis
- Stored in EEPROM `0x10–0x15`
- Calculates: `offset`, `scale_y`, `scale_z`

### Online Tuning
- Flick ch6 >1900 four times within 1 second → `change_settings()` mode

---

## 10. Key Interview Topics to Expect

### Embedded C / Arduino / STM32
- **I2C protocol**: Start, address+W/R, ACK/NACK, stop — how `beginTransmission`, `endTransmission`, `requestFrom` work
- **Timer/Counter modes**: Input capture vs output compare vs PWM
- **Interrupts**: ISR latency concerns, shared data protection (`volatile`)
- **Register-level programming**: `TIMER4_BASE->CCR1`, `TIMER2_BASE->PSC`, `GPIO_CRL`
- **ADC**: 12-bit, reference voltage, voltage dividers
- **UART**: Baud rate, start/stop bits, serial buffer management

### Control Theory
- **PID breakdown**: Proportional (present error), Integral (past), Derivative (future/prediction)
- **Integral windup**: Prevented by clamping `pid_i_mem` to `±pid_max`
- **Derivative kick**: Using `(error - last_error)` instead of derivative of setpoint
- **Gain scheduling**: Altitude P-gain increases with error magnitude
- **Why PD for GPS?**: GPS position is already an integrated measurement (velocity integrated = position); adding I would cause instability

### Sensor Fusion
- **Complementary filter**: High-pass on gyro, low-pass on accel → combine at crossover frequency
- **Why not Kalman?**: Computational cost, tuning complexity, complementary filter is "good enough" at 250Hz
- **Tilt compensation**: Rotating magnetometer readings from body frame to horizontal frame

### Real-time Systems
- **Hard real-time**: 4000µs loop must complete or error flag is set
- **Timing**: `micros()` for precision, `millis()` for non-critical timers
- **Polling** vs **Interrupts**: GPS parsed via polling in main loop; RC input via hardware interrupt

### Data Structures
- **Rotating memory buffers**: O(1) sliding window for averaging sensor data (pressure, GPS, acceleration)
- **Checksumless NMEA parsing**: Relies on `*` delimiter rather than checksum validation

### Common Interview Questions

**Q: How does the IMU prevent gyro drift?**
A: Complementary filter — integrates gyro for fast response, uses accelerometer as long-term reference (gravity vector). Gyro angle weighted 0.9996, accel angle weighted 0.0004.

**Q: Why 250Hz loop rate?**
A: Human pilot inputs ~50Hz; gyro needs 2× the bandwidth of expected motion (drone maneuvers at ~100°/s); 250Hz provides adequate phase margin for PID control.

**Q: How is the barometer stabilized against prop wash?**
A: 20-sample rotating average + complementary filter (slow = 0.985, fast = 0.015). The `parachute_throttle` buffer (30 samples) smooths altitude D-term.

**Q: What happens if GPS signal is lost mid-flight?**
A: 1-second watchdog timer. If no new GPS data within 1s, flight mode drops from 3 (GPS hold) → 2 (altitude hold), error LED flashes code 4.

**Q: Explain the motor mixing equations.**
A: X-configuration quadcopter: Roll = opposite motors (±), Pitch = opposite motors (±), Yaw = torque reaction (±). Each ESC = throttle ± pitch ± roll ± yaw with signs determined by motor position and rotation direction.

**Q: How does the arming sequence prevent accidental startup?**
A: Two-step arming: (1) throttle low + yaw left → `start=1`, (2) yaw center → `start=2`, motors spin. Two-step disarm: throttle low + yaw right → `start=0`. This prevents ESCs from arming on power-up.

---

## 11. File-by-File Summary

| File | Purpose |
|------|---------|
| `YMFC-32_auto_Flight_Controller_ver-1b.ino` | Main: setup, loop (250Hz), sensor reads, motor mixing, timing enforcement |
| `gyro.ino` | MPU-6050 I2C init, data read, gyro calibration (2000 samples) |
| `calculate_pid.ino` | PID for roll, pitch, yaw (rate control) |
| `read_compass.ino` | HMC5883L read, tilt compensation, heading calc, course_deviation() |
| `read_gps.ino` | u-blox config (UBX), NMEA parsing, GPS interpolation, PD position control |
| `Barometer.ino` | MS5611 read, complementary filter, altitude PID |
| `timer_setup.ino` | Timer 2 (input capture), Timer 4 (PWM output) |
| `input_capture_mode_handlers.ino` | ISR for RC PPM/PWM decoding |
| `start_stop_takeoff.ino` | Arm/disarm state machine, takeoff detection |
| `vertical_acceleration_calculations.ino` | Short/long rotating averages of acc Z |
| `calibration.ino` | Compass calibrate, level calibrate |
| `LED_control.ino` | Red/green LED error + flight mode signaling |
| `send_telemetry_data.ino` | Bit-banged serial telemetry to ground station |
| `change_settings.ino` | Online PID tuning via transmitter |
