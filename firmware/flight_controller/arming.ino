///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the starting, stopping and take-off detection is managed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_stop_takeoff(void) {
  if (rc_throttle < 1050 && rc_yaw < 1050)arming_state = 1;                              //For starting the motors: throttle_base low and yaw left (step 1).
  if (arming_state == 1 && rc_throttle < 1050 && rc_yaw > 1450) {                        //When yaw stick is back in the center position arming_state the motors (step 2).
    throttle_base = motor_idle_speed;                                                   //Set the base throttle_base to the motor_idle_speed variable.
    pitch_angle = pitch_from_accel;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    roll_angle = roll_from_accel;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    ground_pressure = actual_pressure;                                             //Register the pressure at ground level for altitude calculations.
    course_lock_heading = yaw_angle;                                               //Set the current compass heading as the course lock heading.
    accel_magnitude_at_arm = accel_magnitude;                                  //Register the acceleration when the quadcopter is started.
    arming_state = 2;                                                                     //Set the arming_state variable to 2 to indicate that the quadcopter is started.
    acc_alt_integrated = 0;                                                        //Reset the integrated acceleration value.
    if (manual_takeoff_throttle > 1400 && manual_takeoff_throttle < 1600) {        //If the manual hover throttle_base is used and valid (between 1400us and 1600us pulse).
      takeoff_throttle = manual_takeoff_throttle - 1500;                           //Use the manual hover throttle_base.
      takeoff_detected = 1;                                                        //Set the auto take-off detection to 1, indicated that the quadcopter is flying.
      //Reset the PID controllers for a smooth take-off.
      resetAllRatePids();                        //Clear stale integrators so take-off is smooth.
    }
    else if (manual_takeoff_throttle) {                                            //If the manual hover throttle_base value is invalid.
      error = 5;                                                                   //Error = 5.
      takeoff_throttle = 0;                                                        //No hover throttle_base compensation.
      arming_state = 0;                                                                   //Set the arming_state variable to 0 to stop the motors.
    }
  }
  //Stopping the motors: throttle_base low and yaw right.
  if (arming_state == 2 && rc_throttle < 1050 && rc_yaw > 1950) {
    arming_state = 0;                                                                     //Set the arming_state variable to 0 to disable the motors.
    takeoff_detected = 0;                                                          //Reset the auto take-off detection.
  }

  if (takeoff_detected == 0 && arming_state == 2) {                                       //When the quadcopter is started and no take-off is detected.
    if (rc_throttle > 1480 && throttle_base < 1750) throttle_base++;                           //When the throttle_base is half way or higher, increase the throttle_base.
    if (throttle_base == 1750)error = 6;                                                //If take-off is not detected when the throttle_base has reached 1700: error = 6.
    if (rc_throttle <= 1480) {                                                       //When the throttle_base is below the center stick position.
      if (throttle_base > motor_idle_speed)throttle_base--;                                  //Lower the throttle_base to the motor_idle_speed variable.
      //Reset the PID controllers for a smooth take-off.
      else {                                                                       //When the throttle_base is back at idle speed reset the PID controllers.
        resetAllRatePids();                        //Clear stale integrators so take-off is smooth.
      }
    }
    if (accel_z_short_average_total / 25 - accel_magnitude_at_arm > 800) {        //A take-off is detected when the quadcopter is accelerating.
      takeoff_detected = 1;                                                        //Set the take-off detected variable to 1 to indicate a take-off.
      altitude_setpoint = ground_pressure - 22;                                //Set the altitude setpoint at groundlevel + approximately 2.2 meters.
      if (throttle_base > 1400 && throttle_base < 1700) takeoff_throttle = throttle_base - 1530;  //If the automated throttle_base is between 1400 and 1600us during take-off, calculate take-off throttle_base.
      else {                                                                       //If the automated throttle_base is not between 1400 and 1600us during take-off.
        takeoff_throttle = 0;                                                      //No take-off throttle_base is calculated.
        error = 7;                                                                 //Show error 7 on the red LED.
      }
    }
  }
}

