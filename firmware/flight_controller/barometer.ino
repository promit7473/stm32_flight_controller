void read_barometer(void) {
  barometer_counter ++;

  //Every time this function is called the barometer_counter variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  if (barometer_counter == 1) {                                                 //When the barometer_counter variable is 1.
    if (temperature_counter == 0) {                                             //And the temperature counter is 0.
      //Get temperature data from MS-5611
      HWire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      HWire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      HWire.endTransmission();                                                  //End the transmission with the MS5611.
      HWire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = HWire.read() << 16 | HWire.read() << 8 | HWire.read();
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
    }
    else {
      //Get pressure data from MS-5611
      HWire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      HWire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      HWire.endTransmission();                                                  //End the transmission with the MS5611.
      HWire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      raw_pressure = HWire.read() << 16 | HWire.read() << 8 | HWire.read();     //Shift the individual bytes in the correct position and add them to the raw_pressure variable.
    }

    temperature_counter ++;                                                     //Increase the temperature_counter variable.
    if (temperature_counter == 20) {                                            //When the temperature counter equals 20.
      temperature_counter = 0;                                                  //Reset the temperature_counter variable.
      //Request temperature data
      HWire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      HWire.write(0x58);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
      HWire.endTransmission();                                                  //End the transmission with the MS5611.
    }
    else {                                                                      //If the temperature_counter variable does not equal 20.
      //Request pressure data
      HWire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      HWire.write(0x48);                                                        //Send a 0x48 to indicate that we want to request the pressure data.
      HWire.endTransmission();                                                  //End the transmission with the MS5611.
    }
  }
  if (barometer_counter == 2) {                                                 //If the barometer_counter variable equals 2.
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressure_average_total -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_average_total += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_average_total / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
  }

  if (barometer_counter == 3) {                                                                               //When the barometer counter is 3

    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

    if (flight_mode >= 2 && takeoff_detected == 1) {                                                          //If the quadcopter is in altitude mode and flying.
      if (altitude_setpoint == 0)altitude_setpoint = actual_pressure;                                 //If not yet set, set the PID altitude setpoint.
      //When the throttle_base stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
      if (rc_throttle > 1600) {                                                        //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (rc_throttle - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle_base.
      }
      if (rc_throttle < 1400) {                                                        //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (rc_throttle - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle_base.
      }

      //Calculate the PID output of the altitude hold.
      altitude_input = actual_pressure;                                          //Set the setpoint (altitude_input) of the PID-controller.
      pid_error = altitude_input - altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable altitude_error_gain will be used to adjust the P-gain of the PID-controller.
      altitude_error_gain = 0;                                                   //Set the altitude_error_gain to 0.
      if (pid_error > 10 || pid_error < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        altitude_error_gain = (abs(pid_error) - 10) / 20.0;                 //The positive altitude_error_gain variable is calculated based based on the error.
        if (altitude_error_gain > 3)altitude_error_gain = 3;                 //To prevent extreme P-gains it must be limited to 3.
      }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      altitude_integrator += (altitude_ki / 100.0) * pid_error;
      if (altitude_integrator > altitude_output_limit)altitude_integrator = altitude_output_limit;
      else if (altitude_integrator < altitude_output_limit * -1)altitude_integrator = altitude_output_limit * -1;
      //In the following line the PID-output is calculated.
      //P = (altitude_kp + altitude_error_gain) * pid_error.
      //I = altitude_integrator += (altitude_ki / 100.0) * pid_error (see above).
      //D = altitude_kd * parachute_throttle.
      altitude_output = (altitude_kp + altitude_error_gain) * pid_error + altitude_integrator + altitude_kd * parachute_throttle;
      //To prevent extreme PID-output the output must be limited.
      if (altitude_output > altitude_output_limit)altitude_output = altitude_output_limit;
      else if (altitude_output < altitude_output_limit * -1)altitude_output = altitude_output_limit * -1;
    }

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless arming_state when the altitude hold function is activated again.
    else if (flight_mode < 2 && altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      altitude_output = 0;                                                       //Reset the output of the PID controller.
      altitude_integrator = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }
  }
}

