#pragma once
///////////////////////////////////////////////////////////////////////////////
// Vehicle configuration: everything you would change to fly a different frame
///////////////////////////////////////////////////////////////////////////////
// Gathered here so that tuning does not mean hunting through the control code,
// and so that the constants which are *not* free to change are visible as such.
//
// Read the timing section before touching LOOP_PERIOD_US. It is not a
// parameter; it is baked into the attitude integration constants.
///////////////////////////////////////////////////////////////////////////////

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
// The control loop is hard real time. It busy-waits until LOOP_PERIOD_US has
// elapsed and flags an error if the work overran, because the attitude
// integration below assumes this exact period.
static const uint32_t LOOP_PERIOD_US    = 4000;   // 250 Hz
static const uint32_t LOOP_OVERRUN_US   = 4050;   // 1.25% margin before error
static const float    LOOP_FREQUENCY_HZ = 250.0f;

// ---------------------------------------------------------------------------
// Sensor scaling
// ---------------------------------------------------------------------------
// MPU-6050 at +-500 deg/s: 65.5 LSB per deg/s (datasheet).
static const float GYRO_LSB_PER_DEG_PER_SEC = 65.5f;

// Raw gyro counts straight to degrees travelled in one loop. This folds the
// sensor scale and the sample period into one multiply:
//     1 / (250 Hz * 65.5 LSB) = 6.107e-5
// Change the loop rate and this is wrong, silently: every integrated angle
// scales by the ratio and nothing raises an error.
static const float GYRO_LSB_TO_DEG_PER_LOOP = 0.0000611f;

// The same step in radians, for the small-angle yaw coupling:
//     6.107e-5 * pi/180 = 1.066e-6
static const float GYRO_LSB_TO_RAD_PER_LOOP = 0.000001066f;

// Note: RAD_TO_DEG and DEG_TO_RAD are already provided as macros by Arduino.h
// at full double precision. Defining them here shadows the core and fails to
// compile, so use the core's.

// First-order IIR on gyro rate: w = (1-a)*w + a*w_raw.
// a = 0.3 gives tau ~13 ms, -3 dB near 14 Hz at 250 Hz: below propeller noise,
// above the rate loop bandwidth.
static const float GYRO_FILTER_ALPHA = 0.3f;

// Complementary filter weight on the gyro. tau = dt*a/(1-a) ~ 10 s, long
// enough that a coordinated turn does not tilt the estimate, short enough to
// wash out gyro bias.
static const float ATTITUDE_GYRO_WEIGHT  = 0.9996f;
static const float ATTITUDE_ACCEL_WEIGHT = 0.0004f;

// Fraction of the gyro/compass heading disagreement corrected each loop.
// 1/1200 at 250 Hz is a ~4.8 s time constant: keeps yaw absolute without
// letting a magnetic disturbance jerk the estimate.
static const float HEADING_CORRECTION_DIVISOR = 1200.0f;

// ---------------------------------------------------------------------------
// Receiver
// ---------------------------------------------------------------------------
static const int32_t RC_MIN_US    = 1000;
static const int32_t RC_CENTRE_US = 1500;
static const int32_t RC_MAX_US    = 2000;

// A receiver at rest jitters a few microseconds. Without a dead band the
// aircraft chases that jitter.
static const int32_t RC_DEAD_BAND_US = 8;

// Microseconds of stick per deg/s of demanded rate. Full stick is therefore
// (500 - 8) / 3 = 164 deg/s.
static const float RC_US_PER_DEG_PER_SEC = 3.0f;

// Below this the throttle counts as closed, for arming and for inhibiting yaw.
static const int32_t RC_THROTTLE_IDLE_US = 1050;

// ---------------------------------------------------------------------------
// Attitude cascade
// ---------------------------------------------------------------------------
// Outer loop: proportional gain from angle error to demanded rate.
static const float ANGLE_TO_RATE_GAIN = 15.0f;

// ---------------------------------------------------------------------------
// Motor output
// ---------------------------------------------------------------------------
static const int16_t MOTOR_MIN_US = 1000;   // disarmed / stopped
static const int16_t MOTOR_MAX_US = 2000;

// Throttle ceiling in flight. The mixer needs headroom above the throttle to
// retain control authority: a saturated mixer cannot stabilise the aircraft.
static const int16_t THROTTLE_CEILING_US = 1800;

// ---------------------------------------------------------------------------
// Battery
// ---------------------------------------------------------------------------
// As the pack sags a given pulse produces less thrust, so the mixer adds an
// open-loop correction of BATTERY_COMPENSATION us per volt below nominal.
static const float BATTERY_NOMINAL_V     = 12.40f;
static const float BATTERY_MIN_VALID_V   = 6.0f;   // below this, assume no sensor
static const float BATTERY_FILTER_ALPHA  = 0.92f;
static const float BATTERY_ADC_DIVISOR   = 1410.1f;

// ---------------------------------------------------------------------------
// I2C devices
// ---------------------------------------------------------------------------
static const uint8_t GYRO_I2C_ADDRESS       = 0x68;  // MPU-6050
static const uint8_t BAROMETER_I2C_ADDRESS  = 0x77;  // MS5611
static const uint8_t COMPASS_I2C_ADDRESS    = 0x1E;  // HMC5883L

// ---------------------------------------------------------------------------
// Board pins
// ---------------------------------------------------------------------------
#define BOARD_LED_PIN   PC13   // on-board LED, inverted logic
#define LED_GREEN_PIN   PB3
#define LED_RED_PIN     PB4
static const uint8_t BATTERY_SENSE_PIN = 4;   // analog, port A4

// ---------------------------------------------------------------------------
// Rate controller state
// ---------------------------------------------------------------------------
// Gains and state travel together so that one function can serve all three
// axes. See control_pid.ino for what ki and kd actually mean here.
struct RatePid {
  float kp;
  float ki;
  float kd;
  float limit;             // symmetric clamp on both integrator and output
  float integrator;
  float previous_error;
  float output;
};
