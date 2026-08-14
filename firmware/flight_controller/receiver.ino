///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//More information can be found in these two videos:
//STM32 for Arduino - Connecting an RC receiver via input capture mode: https://youtu.be/JFSFbSg0l2M
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void handler_channel_1(void) {
  measured_time = TIMER2_BASE->CCR1 - measured_time_start;
  if (measured_time < 0)measured_time += 0xFFFF;
  measured_time_start = TIMER2_BASE->CCR1;
  if (measured_time > 3000)rc_channel_select_counter = 0;
  else rc_channel_select_counter++;

  if (rc_channel_select_counter == 1)rc_roll = measured_time;
  if (rc_channel_select_counter == 2)rc_pitch = measured_time;
  if (rc_channel_select_counter == 3)rc_throttle = measured_time;
  if (rc_channel_select_counter == 4)rc_yaw = measured_time;
  if (rc_channel_select_counter == 5)rc_flight_mode = measured_time;
  if (rc_channel_select_counter == 6)rc_aux = measured_time;
}

