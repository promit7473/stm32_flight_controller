///////////////////////////////////////////////////////////////////////////////
// Rate control: three axes, one controller
///////////////////////////////////////////////////////////////////////////////
// The inner loop regulates angular *rate*, not angle. Roll, pitch and yaw ran
// as three copies of the same twelve lines, which meant a fix to one axis had
// to be remembered for the other two. They now share updateRatePid(), and each
// axis keeps its own gains and state in a RatePid.
//
// Two properties of this discretisation are worth knowing before touching the
// gains, because they are not the textbook ones:
//
//   ki  accumulates once per loop, not per second, so the continuous-time
//       integral gain is ki * LOOP_FREQUENCY_HZ.
//   kd  multiplies the difference between successive errors with no division
//       by dt, so the continuous-time derivative gain is kd / LOOP_FREQUENCY_HZ.
//
// The integrator is clamped to the same limit as the output. That is what
// stops wind-up from parking a motor at full throttle after a long
// disturbance: without it the integrator can grow without bound while the
// output sits saturated, and it then has to unwind before the axis responds.
///////////////////////////////////////////////////////////////////////////////

// Rate demand from a stick, in degrees per second.
//
// The dead band matters more than it looks: a receiver at rest jitters by a
// few microseconds, and without it the aircraft would chase that jitter.
static float rateSetpointFromStick(int32_t stick_us, float level_adjust) {
  float setpoint = 0.0f;
  if (stick_us > RC_CENTRE_US + RC_DEAD_BAND_US) {
    setpoint = stick_us - (RC_CENTRE_US + RC_DEAD_BAND_US);
  } else if (stick_us < RC_CENTRE_US - RC_DEAD_BAND_US) {
    setpoint = stick_us - (RC_CENTRE_US - RC_DEAD_BAND_US);
  }
  // level_adjust is the outer attitude loop: a P controller on angle whose
  // output is a rate demand. Subtracting it makes level flight the fixed point.
  setpoint -= level_adjust;
  return setpoint / RC_US_PER_DEG_PER_SEC;
}

// One PID step. Returns the output and stores it in the controller.
static float updateRatePid(RatePid &pid, float measured_rate, float setpoint) {
  const float error = measured_rate - setpoint;

  pid.integrator += pid.ki * error;
  if (pid.integrator > pid.limit) pid.integrator = pid.limit;
  else if (pid.integrator < -pid.limit) pid.integrator = -pid.limit;

  float output = pid.kp * error
               + pid.integrator
               + pid.kd * (error - pid.previous_error);

  if (output > pid.limit) output = pid.limit;
  else if (output < -pid.limit) output = -pid.limit;

  pid.previous_error = error;
  pid.output = output;
  return output;
}

// Clear one axis. Called when arming and when the throttle returns to idle, so
// that a stale integrator cannot kick the aircraft on the next take-off.
static void resetRatePid(RatePid &pid) {
  pid.integrator = 0.0f;
  pid.previous_error = 0.0f;
  pid.output = 0.0f;
}

void resetAllRatePids(void) {
  resetRatePid(roll_pid);
  resetRatePid(pitch_pid);
  resetRatePid(yaw_pid);
}

void calculate_pid(void) {
  roll_rate_setpoint = rateSetpointFromStick(roll_command_us, roll_level_adjust);
  pitch_rate_setpoint = rateSetpointFromStick(pitch_command_us, pitch_level_adjust);

  // Yaw takes the stick directly: there is no angle to hold, only a rate. The
  // guard stops the aircraft yawing while the throttle is being held down to
  // disarm, which is the same stick position that stops the motors.
  yaw_rate_setpoint = 0.0f;
  if (rc_throttle > RC_THROTTLE_IDLE_US) {
    yaw_rate_setpoint = rateSetpointFromStick(rc_yaw, 0.0f);
  }

  updateRatePid(roll_pid, roll_rate_filtered, roll_rate_setpoint);
  updateRatePid(pitch_pid, pitch_rate_filtered, pitch_rate_setpoint);
  updateRatePid(yaw_pid, yaw_rate_filtered, yaw_rate_setpoint);
}
