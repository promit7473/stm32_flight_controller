///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>
#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include "config.h"                        //Frame, sensor, timing and controller configuration.
TwoWire HWire (2, I2C_FAST_MODE);          //Initiate I2C port 2 at 400kHz.

///////////////////////////////////////////////////////////////////////////////
//Rate controllers
///////////////////////////////////////////////////////////////////////////////
//Gains and state per axis. All three are driven by the same updateRatePid()
//in control_pid.ino, so a change to the algorithm applies to every axis.
//Roll and pitch share gains because the frame is symmetric about both.
//Yaw carries no derivative term: it has the largest rotational inertia and the
//least aerodynamic damping, so D amplifies motor noise for little phase lead.
RatePid roll_pid  = { 1.3f, 0.04f, 18.0f, 400.0f, 0.0f, 0.0f, 0.0f };
RatePid pitch_pid = { 1.3f, 0.04f, 18.0f, 400.0f, 0.0f, 0.0f, 0.0f };
RatePid yaw_pid   = { 4.0f, 0.02f,  0.0f, 400.0f, 0.0f, 0.0f, 0.0f };

//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effect on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
float battery_compensation = 40.0;

float altitude_kp = 1.4;           //Gain setting for the altitude P-controller (default = 1.4).
float altitude_ki = 0.2;           //Gain setting for the altitude I-controller (default = 0.2).
float altitude_kd = 0.75;          //Gain setting for the altitude D-controller (default = 0.75).
int altitude_output_limit = 400;                //Maximum output of the PID-controller (+/-).

float gps_p_gain = 2.7;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 6.5;                    //Gain setting for the GPS D-controller (default = 6.5).

float declination = 0.0;                   //Set the declination between the magnetic and geographic north.

int16_t manual_takeoff_throttle = 1500;    //Enter the manual hover point when auto take-off detection is not desired (between 1400 and 1600).
int16_t motor_idle_speed = 1100;           //Enter the minimum throttle_base pulse of the motors when they idle (between 1000 and 1200). 1170 for DJI

float low_battery_warning = 10.5;          //Set the battery warning at 10.5V (default = 10.5V).

//Tuning parameters/settings is explained in this video: https://youtu.be/ys-YpOaA2ME
#define variable_1_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.
#define variable_2_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.
#define variable_3_to_adjust dummy_float   //Change dummy_float to any setting that you want to tune.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

uint8_t rc_roll_was_high, rc_pitch_was_high, rc_throttle_was_high, rc_yaw_was_high;
uint8_t check_byte, flip32, arming_state;
uint8_t error, error_counter, error_led;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint8_t takeoff_detected, manual_altitude_change;
uint8_t telemetry_send_byte, telemetry_bit_counter, telemetry_loop_counter;
uint8_t rc_channel_select_counter;
uint8_t level_calibration_on;

uint32_t telemetry_buffer_byte;

int16_t motor_front_right, motor_rear_right, motor_rear_left, motor_front_left;
int16_t manual_throttle;
int16_t throttle_base, takeoff_throttle, cal_int;
int16_t temperature, count_var;
int16_t accel_x, accel_y, accel_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t rc_roll_pulse_start, rc_roll, roll_command_us;
int32_t rc_pitch_pulse_start, rc_pitch, pitch_command_us;
int32_t rc_throttle_pulse_start, rc_throttle;
int32_t rc_yaw_pulse_start, rc_yaw;
int32_t rc_mode_pulse_start, rc_flight_mode;
int32_t rc_aux_pulse_start, rc_aux;
int32_t measured_time, measured_time_start;
int32_t accel_magnitude, accel_magnitude_at_arm;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int16_t acc_pitch_cal_value;
int16_t acc_roll_cal_value;

int32_t accel_z_short_average_total, accel_z_long_average_total, acc_z_average_total ;
int16_t acc_z_average_short[26], acc_z_average_long[51];

uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;

int32_t acc_alt_integrated;

uint32_t loop_timer, error_timer, flight_mode_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error;
//Integrator, previous error and output now live in the RatePid structs above.
float roll_rate_setpoint, roll_rate_filtered;
float pitch_rate_setpoint, pitch_rate_filtered;
float yaw_rate_setpoint, yaw_rate_filtered;
float roll_from_accel, pitch_from_accel, pitch_angle, roll_angle, yaw_angle;
float battery_voltage, dummy_float;

//Compass variables
uint8_t compass_calibration_on, heading_lock;
int16_t compass_x, compass_y, compass_z;
int16_t compass_cal_values[6];
float compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float compass_scale_y, compass_scale_z;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
float course_lock_heading, heading_lock_course_deviation;


//Pressure variables.
float altitude_error_gain, altitude_throttle_gain;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altitude_hold_pressure;
int32_t dT, dT_C5;
//Altitude PID variables
float altitude_integrator, altitude_setpoint, altitude_input, altitude_output, altitude_previous_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_average_total;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

//GPS variables
uint8_t read_serial_byte, incoming_message[100], number_used_sats, fix_type;
uint8_t waypoint_set, latitude_north, longitude_east ;
uint16_t message_counter;
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location;
int32_t gps_lat_average_total, gps_lon_average_total;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;

//Adjust settings online
uint32_t setting_adjust_timer;
uint16_t setting_click_counter;
uint8_t rc_aux_previous;
float adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(4, INPUT_ANALOG);                                     //This is needed for reading the analog value of port A4.
  //Port PB3 and PB4 are used as JTDO and JNTRST by default.
  //The following function connects PB3 and PB4 to the
  //alternate output function.
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                     //Connects PB3 and PB4 to output function.

  pinMode(PB3, OUTPUT);                                         //Set PB3 as output for green LED.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output for red LED.
  pinMode(BOARD_LED_PIN, OUTPUT);                             //This is the LED on the STM32 board. Used for GPS indication.
  digitalWrite(BOARD_LED_PIN, HIGH);                          //Turn the LED on the STM32 off. The LED function is inverted. Check the STM32 schematic.

  green_led(LOW);                                               //Set output PB3 low.
  red_led(HIGH);                                                //Set output PB4 high.

  pinMode(PB0, OUTPUT);                                         //Set PB0 as output for telemetry TX.

  //EEPROM emulation setup
  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x400;

  //Serial.begin(57600);                                        //Set the serial output to 57600 kbps. (for debugging only)
  //delay(250);                                                 //Give the serial port some time to arming_state to prevent data loss.

  timer_setup();                                                //Setup the timers for the receiver inputs and ESC's output.
  delay(50);                                                    //Give the timers some time to arming_state.

  gps_setup();                                                  //Set the baud rate and output refreshrate of the GPS module.

  //Check if the MPU-6050 is responding.
  HWire.begin();                                                //Start the I2C as master
  HWire.beginTransmission(GYRO_I2C_ADDRESS);                        //Start communication with the MPU-6050.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not responde.
    error = 1;                                                  //Set the error status to 1.
    error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  //Check if the compass is responding.
  HWire.beginTransmission(COMPASS_I2C_ADDRESS);                     //Start communication with the HMC5883L.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the HMC5883L did not responde.
    error = 2;                                                  //Set the error status to 2.
    error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  //Check if the MS5611 barometer is responding.
  HWire.beginTransmission(BAROMETER_I2C_ADDRESS);                      //Start communication with the MS5611.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MS5611 did not responde.
    error = 3;                                                  //Set the error status to 2.
    error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  gyro_setup();                                                 //Initiallize the gyro and set the correct registers.
  setup_compass();                                              //Initiallize the compass and set the correct registers.
  read_compass();                                               //Read and calculate the compass data.
  yaw_angle = actual_compass_heading;                           //Set the initial compass heading.

  //Create a 5 second delay before calibration.
  for (count_var = 0; count_var < 1250; count_var++) {          //1250 loops of 4 microseconds = 5 seconds.
    if (count_var % 125 == 0) {                                 //Every 125 loops (500ms).
      digitalWrite(PB4, !digitalRead(PB4));                     //Change the led status.
    }
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  count_var = 0;                                                //Set arming_state back to 0.
  calibrate_gyro();                                             //Calibrate the gyro offset.

  //Wait until the receiver is active.
  while (rc_roll < 990 || rc_pitch < 990 || rc_throttle < 990 || rc_yaw < 990)  {
    error = 4;                                                  //Set the error status to 4.
    error_signal();                                             //Show the error via the red LED.
    delay(4);                                                   //Delay 4ms to simulate a 250Hz loop
  }
  error = 0;                                                    //Reset the error status to 0.


  //When everything is done, turn off the led.
  red_led(LOW);                                                 //Set output PB4 low.

  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //36.3 / 4095 = 112.81.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (float)analogRead(4) / 112.81;

  //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
  //These 2 byte values are stored in the memory location 0xA2 and up.
  for (arming_state = 1; arming_state <= 6; arming_state++) {
    HWire.beginTransmission(BAROMETER_I2C_ADDRESS);                    //Start communication with the MPU-6050.
    HWire.write(0xA0 + arming_state * 2);                              //Send the address that we want to read.
    HWire.endTransmission();                                    //End the transmission.

    HWire.requestFrom(BAROMETER_I2C_ADDRESS, 2);                       //Request 2 bytes from the MS5611.
    C[arming_state] = HWire.read() << 8 | HWire.read();                //Add the low and high byte to the C[x] calibration variable.
  }

  OFF_C2 = C[2] * 65536.0;                                   //This value is pre-calculated to offload the main program loop.
  SENS_C1 = C[1] * 32768.0;                                  //This value is pre-calculated to offload the main program loop.

  //The MS5611 needs a few readings to stabilize.
  for (arming_state = 0; arming_state < 100; arming_state++) {                       //This loop runs 100 times.
    read_barometer();                                           //Read and calculate the barometer data.
    delay(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
  }
  actual_pressure = 0;                                          //Reset the pressure calculations.

  //Before starting the avarage accelerometer value is preloaded into the variables.
  for (arming_state = 0; arming_state <= 24; arming_state++)acc_z_average_short[arming_state] = accel_z;
  for (arming_state = 0; arming_state <= 49; arming_state++)acc_z_average_long[arming_state] = accel_z;
  accel_z_short_average_total = accel_z * 25;
  accel_z_long_average_total = accel_z * 50;
  arming_state = 0;

  if (motor_idle_speed < 1000)motor_idle_speed = 1000;          //Limit the minimum idle motor speed to 1000us.
  if (motor_idle_speed > 1200)motor_idle_speed = 1200;          //Limit the maximum idle motor speed to 1200us.

  loop_timer = micros();                                        //Set the timer for the first loop.
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //Some functions are only accessible when the quadcopter is off.
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
  if (rc_aux > 1200)heading_lock = 1;                                           //If channel 6 is between 1200us and 1600us the flight mode is 2

  flight_mode = 1;                                                                 //In all other situations the flight mode is 1;
  if (rc_flight_mode >= 1200 && rc_flight_mode < 1600)flight_mode = 2;                       //If channel 6 is between 1200us and 1600us the flight mode is 2
  if (rc_flight_mode >= 1600 && rc_flight_mode < 2100)flight_mode = 3;                       //If channel 6 is between 1600us and 1900us the flight mode is 3

  flight_mode_signal();                                                            //Show the flight_mode via the green LED.
  error_signal();                                                                  //Show the error via the red LED.
  read_imu();                                                                 //Read the gyro and accelerometer data.
  read_barometer();                                                                //Read and calculate the barometer data.
  read_compass();                                                                  //Read and calculate the compass data.

  if (gps_add_counter >= 0)gps_add_counter --;

  read_gps();

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
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

  calculate_pid();                                                                 //Calculate the pid outputs based on the receiver inputs.

  start_stop_takeoff();                                                            //Starting, stopping and take-off detection

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1);

  //Turn on the led if battery voltage is to low. Default setting is 10.5V
  if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && error == 0)error = 1;


  //The variable base_throttle is calculated in the following part. It forms the base throttle_base for every motor.
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
