///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(void) {

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  roll_rate_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (roll_command_us > 1508)roll_rate_setpoint = roll_command_us - 1508;
  else if (roll_command_us < 1492)roll_rate_setpoint = roll_command_us - 1492;

  roll_rate_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
  roll_rate_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pitch_rate_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (pitch_command_us > 1508)pitch_rate_setpoint = pitch_command_us - 1508;
  else if (pitch_command_us < 1492)pitch_rate_setpoint = pitch_command_us - 1492;

  pitch_rate_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
  pitch_rate_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  yaw_rate_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (rc_throttle > 1050) { //Do not yaw when turning off the motors.
    if (rc_yaw > 1508)yaw_rate_setpoint = (rc_yaw - 1508) / 3.0;
    else if (rc_yaw < 1492)yaw_rate_setpoint = (rc_yaw - 1492) / 3.0;
  }

  //Roll calculations
  pid_error = roll_rate_filtered - roll_rate_setpoint;
  roll_integrator += roll_rate_ki * pid_error;
  if (roll_integrator > roll_output_limit)roll_integrator = roll_output_limit;
  else if (roll_integrator < roll_output_limit * -1)roll_integrator = roll_output_limit * -1;

  roll_output = roll_rate_kp * pid_error + roll_integrator + roll_rate_kd * (pid_error - roll_previous_error);
  if (roll_output > roll_output_limit)roll_output = roll_output_limit;
  else if (roll_output < roll_output_limit * -1)roll_output = roll_output_limit * -1;

  roll_previous_error = pid_error;

  //Pitch calculations
  pid_error = pitch_rate_filtered - pitch_rate_setpoint;
  pitch_integrator += pitch_rate_ki * pid_error;
  if (pitch_integrator > pitch_output_limit)pitch_integrator = pitch_output_limit;
  else if (pitch_integrator < pitch_output_limit * -1)pitch_integrator = pitch_output_limit * -1;

  pitch_output = pitch_rate_kp * pid_error + pitch_integrator + pitch_rate_kd * (pid_error - pitch_previous_error);
  if (pitch_output > pitch_output_limit)pitch_output = pitch_output_limit;
  else if (pitch_output < pitch_output_limit * -1)pitch_output = pitch_output_limit * -1;

  pitch_previous_error = pid_error;

  //Yaw calculations
  pid_error = yaw_rate_filtered - yaw_rate_setpoint;
  yaw_integrator += yaw_rate_ki * pid_error;
  if (yaw_integrator > yaw_output_limit)yaw_integrator = yaw_output_limit;
  else if (yaw_integrator < yaw_output_limit * -1)yaw_integrator = yaw_output_limit * -1;

  yaw_output = yaw_rate_kp * pid_error + yaw_integrator + yaw_rate_kd * (pid_error - yaw_previous_error);
  if (yaw_output > yaw_output_limit)yaw_output = yaw_output_limit;
  else if (yaw_output < yaw_output_limit * -1)yaw_output = yaw_output_limit * -1;

  yaw_previous_error = pid_error;
}

