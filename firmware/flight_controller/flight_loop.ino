///////////////////////////////////////////////////////////////////////////////
// The control loop, one phase per function
///////////////////////////////////////////////////////////////////////////////
// loop() used to be 185 lines in one block, which made the order of
// operations hard to see and easy to disturb. The phases below are that same
// code, moved verbatim and unchanged, so loop() now reads as the list of
// steps the aircraft performs every 4 ms.
//
// The order is load-bearing. Sensors are read before the estimate is updated,
// the estimate is updated before the controllers run on it, and the motor
// pulses are written last so they carry the freshest possible commands.
///////////////////////////////////////////////////////////////////////////////

// Calibration and settings, reachable only while disarmed.
// Stick corners are used as a keypad: there is no other input device.
static void handleDisarmedRequests(void) {
  if (arming_state == 0) {
    //For compass calibration move both sticks to the top right.
    if (rc_roll > 1900 && rc_pitch < 1100 && rc_throttle > 1900 && rc_yaw > 1900)calibrate_compass();
    //Level calibration move both sticks to the top left.
    if (rc_roll < 1100 && rc_pitch < 1100 && rc_throttle > 1900 && rc_yaw < 1100)calibrate_level();
    //Change settings
    if (rc_aux >= 1900 && rc_aux_previous == 0) {
      rc_aux_previous = 1;
      if (setting_adjust_timer > millis())setting_click_counter ++;
      else setting_click_counter = 0;
      setting_adjust_timer = millis() + 1000;
      if (setting_click_counter > 3) {
        setting_click_counter = 0;
        change_settings();
      }
    }
    if (rc_aux < 1900)rc_aux_previous = 0;
  }

  heading_lock = 0;
}

// Decode the mode and heading-lock switches, then drive the indicator LEDs.
static void selectFlightMode(void) {
  if (rc_aux > 1200)heading_lock = 1;                                           //If channel 6 is between 1200us and 1600us the flight mode is 2

  flight_mode = 1;                                                                 //In all other situations the flight mode is 1;
  if (rc_flight_mode >= 1200 && rc_flight_mode < 1600)flight_mode = 2;                       //If channel 6 is between 1200us and 1600us the flight mode is 2
  if (rc_flight_mode >= 1600 && rc_flight_mode < 2100)flight_mode = 3;                       //If channel 6 is between 1600us and 1900us the flight mode is 3

  flight_mode_signal();                                                            //Show the flight_mode via the green LED.
  error_signal();                                                                  //Show the error via the red LED.
}

// Pull one sample from every sensor. Order is not arbitrary: the IMU is
// read first because everything downstream depends on it.
static void readSensors(void) {
  read_imu();                                                                 //Read the gyro and accelerometer data.
  read_barometer();                                                                //Read and calculate the barometer data.
  read_compass();                                                                  //Read and calculate the compass data.

  if (gps_add_counter >= 0)gps_add_counter --;

  read_gps();

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
}

// Gyro filtering, integration, yaw coupling, and the complementary blend
// with the accelerometer. See README section 2 for the derivation.
static void updateAttitudeEstimate(void) {
  roll_rate_filtered = (roll_rate_filtered * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  pitch_rate_filtered = (pitch_rate_filtered * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  yaw_rate_filtered = (yaw_rate_filtered * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  pitch_angle += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the pitch_angle variable.
  roll_angle += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the roll_angle variable.
  yaw_angle += (float)gyro_yaw * 0.0000611;                                        //Calculate the traveled yaw angle and add this to the yaw_angle variable.
  if (yaw_angle < 0) yaw_angle += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (yaw_angle >= 360) yaw_angle -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  pitch_angle -= roll_angle * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  roll_angle += pitch_angle * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  yaw_angle -= course_deviation(yaw_angle, actual_compass_heading) / 1200.0;       //Calculate the difference between the gyro and compass heading and make a small correction.
  if (yaw_angle < 0) yaw_angle += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (yaw_angle >= 360) yaw_angle -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.


  //Accelerometer angle calculations
  accel_magnitude = sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));    //Calculate the total accelerometer vector.

  if (abs(accel_y) < accel_magnitude) {                                             //Prevent the asin function to produce a NaN.
    pitch_from_accel = asin((float)accel_y / accel_magnitude) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(accel_x) < accel_magnitude) {                                             //Prevent the asin function to produce a NaN.
    roll_from_accel = asin((float)accel_x / accel_magnitude) * 57.296;               //Calculate the roll angle.
  }

  pitch_angle = pitch_angle * 0.9996 + pitch_from_accel * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  roll_angle = roll_angle * 0.9996 + roll_from_accel * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = pitch_angle * 15;                                           //Calculate the pitch angle correction.
  roll_level_adjust = roll_angle * 15;                                             //Calculate the roll angle correction.

  vertical_acceleration_calculations();                                            //Calculate the vertical accelration.
}

// Turn stick positions into roll and pitch commands, rotating them for
// heading lock and adding the GPS correction when position hold is on.
static void computeStickCommands(void) {
  roll_command_us = rc_roll;                                              //Normally rc_roll is the roll_rate_setpoint input.
  pitch_command_us = rc_pitch;                                             //Normally rc_pitch is the pitch_rate_setpoint input.
  //When the heading_lock mode is activated the roll and pitch pid setpoints are heading dependent.
  //At startup the heading is registerd in the variable course_lock_heading.
  //First the course deviation is calculated between the current heading and the course_lock_heading is calculated.
  //Based on this deviation the pitch and roll controls are calculated so the responce is the same as on startup.
  if (heading_lock == 1) {
    heading_lock_course_deviation = course_deviation(yaw_angle, course_lock_heading);
    roll_command_us = 1500 + ((float)(rc_roll - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(rc_pitch - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
    pitch_command_us = 1500 + ((float)(rc_pitch - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(rc_roll - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
  }

  if (flight_mode >= 3 && waypoint_set == 1) {
    roll_command_us += gps_roll_adjust;
    pitch_command_us += gps_pitch_adjust;
  }

  //Because we added the GPS adjust values we need to make sure that the control limits are not exceded.
  if (roll_command_us > 2000)roll_command_us = 2000;
  if (roll_command_us < 1000)roll_command_us = 1000;
  if (pitch_command_us > 2000)pitch_command_us = 2000;
  if (pitch_command_us < 1000)pitch_command_us = 1000;
}

// Rate PIDs, arming state machine, and the filtered battery voltage that
// the mixer needs for sag compensation.
static void runControllers(void) {
  calculate_pid();                                                                 //Calculate the pid outputs based on the receiver inputs.

  start_stop_takeoff();                                                            //Starting, stopping and take-off detection

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1);

  //Turn on the led if battery voltage is to low. Default setting is 10.5V
  if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && error == 0)error = 1;


  //The variable base_throttle is calculated in the following part. It forms the base throttle_base for every motor.
}

// Base throttle common to all four motors, before mixing.
static void computeBaseThrottle(void) {
  if (takeoff_detected == 1 && arming_state == 2) {                                         //If the quadcopter is started and flying.
    throttle_base = rc_throttle + takeoff_throttle;                                         //The base throttle_base is the receiver throttle_base channel + the detected take-off throttle_base.
    if (flight_mode >= 2) {                                                          //If altitude mode is active.
      throttle_base = 1500 + takeoff_throttle + altitude_output + manual_throttle;    //The base throttle_base is the receiver throttle_base channel + the detected take-off throttle_base + the PID controller output.
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/Nju9rvZOjVQ
  ////////////////////////////////////////////////////////////////////////////////////////////////////
}

// Motor mixing, battery compensation, limits, and the timer compare
// registers that become the ESC pulses. Spans the whole armed/disarmed
// if-else: cutting inside it would unbalance the braces.
static void mixAndDriveMotors(void) {
  if (arming_state == 2) {                                                                //The motors are started.
    if (throttle_base > 1800) throttle_base = 1800;                                          //We need some room to keep full control at full throttle_base.
    motor_front_right = throttle_base - pitch_pid.output + roll_pid.output - yaw_pid.output;        //Calculate the pulse for esc 1 (front-right - CCW).
    motor_rear_right = throttle_base + pitch_pid.output + roll_pid.output + yaw_pid.output;        //Calculate the pulse for esc 2 (rear-right - CW).
    motor_rear_left = throttle_base + pitch_pid.output - roll_pid.output - yaw_pid.output;        //Calculate the pulse for esc 3 (rear-left - CCW).
    motor_front_left = throttle_base - pitch_pid.output - roll_pid.output + yaw_pid.output;        //Calculate the pulse for esc 4 (front-left - CW).

    if (battery_voltage < 12.40 && battery_voltage > 6.0) {                        //Is the battery connected?
      motor_front_right += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-1 pulse for voltage drop.
      motor_rear_right += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-2 pulse for voltage drop.
      motor_rear_left += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-3 pulse for voltage drop.
      motor_front_left += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-4 pulse for voltage drop.
    }

    if (motor_front_right < motor_idle_speed) motor_front_right = motor_idle_speed;                        //Keep the motors running.
    if (motor_rear_right < motor_idle_speed) motor_rear_right = motor_idle_speed;                        //Keep the motors running.
    if (motor_rear_left < motor_idle_speed) motor_rear_left = motor_idle_speed;                        //Keep the motors running.
    if (motor_front_left < motor_idle_speed) motor_front_left = motor_idle_speed;                        //Keep the motors running.

    if (motor_front_right > 2000)motor_front_right = 2000;                                                 //Limit the esc-1 pulse to 2000us.
    if (motor_rear_right > 2000)motor_rear_right = 2000;                                                 //Limit the esc-2 pulse to 2000us.
    if (motor_rear_left > 2000)motor_rear_left = 2000;                                                 //Limit the esc-3 pulse to 2000us.
    if (motor_front_left > 2000)motor_front_left = 2000;                                                 //Limit the esc-4 pulse to 2000us.
  }

  else {
    motor_front_right = 1000;                                                                  //If arming_state is not 2 keep a 1000us pulse for ess-1.
    motor_rear_right = 1000;                                                                  //If arming_state is not 2 keep a 1000us pulse for ess-2.
    motor_rear_left = 1000;                                                                  //If arming_state is not 2 keep a 1000us pulse for ess-3.
    motor_front_left = 1000;                                                                  //If arming_state is not 2 keep a 1000us pulse for ess-4.
  }


  TIMER4_BASE->CCR1 = motor_front_right;                                                       //Set the throttle_base receiver input pulse to the ESC 1 output pulse.
  TIMER4_BASE->CCR2 = motor_rear_right;                                                       //Set the throttle_base receiver input pulse to the ESC 2 output pulse.
  TIMER4_BASE->CCR3 = motor_rear_left;                                                       //Set the throttle_base receiver input pulse to the ESC 3 output pulse.
  TIMER4_BASE->CCR4 = motor_front_left;                                                       //Set the throttle_base receiver input pulse to the ESC 4 output pulse.
  TIMER4_BASE->CNT = 5000;                                                         //This will reset timer 4 and the ESC pulses are directly created.
}

// Telemetry, then hold the loop open until the full period has elapsed.
// The busy-wait is what makes the 250 Hz rate exact; see README 8.
static void sendTelemetryAndWait(void) {
  send_telemetry_data();                                                           //Send telemetry data to the ground station.

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  //that the loop time is still 4000us and no longer! More information can be found on
  //the Q&A page:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  if (micros() - loop_timer > 4050)error = 2;                                      //Output an error if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  loop_timer = micros();                                                           //Set the timer for the next loop.
}
