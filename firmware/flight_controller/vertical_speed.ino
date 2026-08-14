///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the vertical acceleration is calculated over a longer period via a rotating memory.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void vertical_acceleration_calculations(void) {
  acc_z_average_short_rotating_mem_location++;
  if (acc_z_average_short_rotating_mem_location == 25)acc_z_average_short_rotating_mem_location = 0;

  accel_z_short_average_total -= acc_z_average_short[acc_z_average_short_rotating_mem_location];
  acc_z_average_short[acc_z_average_short_rotating_mem_location] = accel_magnitude;
  accel_z_short_average_total += acc_z_average_short[acc_z_average_short_rotating_mem_location];

  if (acc_z_average_short_rotating_mem_location == 0) {
    acc_z_average_long_rotating_mem_location++;

    if (acc_z_average_long_rotating_mem_location == 50)acc_z_average_long_rotating_mem_location = 0;

    accel_z_long_average_total -= acc_z_average_long[acc_z_average_long_rotating_mem_location];
    acc_z_average_long[acc_z_average_long_rotating_mem_location] = accel_z_short_average_total / 25;
    accel_z_long_average_total += acc_z_average_long[acc_z_average_long_rotating_mem_location];
  }
  acc_z_average_total = accel_z_long_average_total / 50;


  acc_alt_integrated += accel_magnitude - acc_z_average_total;
  if (accel_magnitude - acc_z_average_total < 400 || accel_magnitude - acc_z_average_total > 400) {
    if (accel_z_short_average_total / 25 - acc_z_average_total < 500 && accel_z_short_average_total / 25 - acc_z_average_total > -500)
      if (acc_alt_integrated > 200)acc_alt_integrated -= 200;
      else if (acc_alt_integrated < -200)acc_alt_integrated += 200;
  }
}

