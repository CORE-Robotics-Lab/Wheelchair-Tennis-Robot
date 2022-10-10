#ifndef CUSTOM_VARIABLES_H
#define CUSTOM_VARIABLES_H

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

const float axis_max_velocity = 50;        // Maximum velocity for an individual axis (motor rev/s)
const float linear_velocity_max = 5;       // Maximum linear velocity [m/s]
const float angular_velocity_max = 5;      // Maximum angular velocity [rads/s]
const float wheel_separation = 0.92;       // Wheel separation
const float wheel_radius = 0.3225;         // Wheel radius
const float motor_to_wheel_ratio = 20.0;   // Gear ratio of motor to wheels
const float manual_max_velocity_ramp = 30; // Manual maximum velocity ramp (motor rev/s^2)
const float auto_max_velocity_ramp = 150;  // Autonomous maximum velocity ramp (motor rev/s^2)

const int serial_update_period = 100;        // Every 0.2ms (5kHz)
const int background_timer_period = 100000;  // Every 100ms (10Hz)

volatile struct CalibrationConfig
{
    int throttle_min;         // Minimum magnitude for throttle when zeroed
    int throttle_max;         // Maximum magnitude for throttle when zeroed
    int throttle_zero_offset; // Average magnitude when throttle zero
    int throttle_zero_min;    // us difference to keep zeroed throttle at zero
    int throttle_zero_max;    // us difference to keep zeroed throttle at zero
    int steering_min;         // Minimum magnitude for steering when zeroed
    int steering_max;         // Maximum magnitude for steering when zeroed
    int steering_zero_offset; // Average magnitude when steering zero
    int steering_zero_min;    // us difference to keep zeroed steering at zero
    int steering_zero_max;    // us difference to keep zeroed steering at zero

    int number_samples = 50; // Number of samples to take while calibrating

} calibration;

const int throttle_threshold_band = 25;      // Required difference to trigger calibration
const int steering_threshold_band = 25;      // Required difference to trigger calibration
const int mode_threshold_idle = 1259;        // Average mode idle time
const int mode_threshold_manual = 1511;      // Average mode manual time
const int mode_threshold_auto = 1763;        // Average mode autonomous time
const int estop_threshold_on = 1742;         // Average estop enabled time
const int estop_threshold_off = 1280;        // Average estop disabled time
const int calibration_threshold_low = 875;   // Average calibration remote time
const int calibration_threshold_high = 2125; // Average calibration odrive time
const int pulse_threshold_band = 30;         // +/- band for pulse times

const int max_pulse_length = 2500; // 2.5ms
const int min_pulse_length = 500;  // 0.5ms

volatile unsigned int input_throttle;    // Stores value of raw throttle pin
volatile unsigned int input_steering;    // Stores value of raw steering pin
volatile unsigned int input_estop;       // Stores value of raw estop pin
volatile unsigned int input_mode;        // Stores value of raw mode pin
volatile unsigned int prev_input_mode;   // Stores previous value of raw mode pin
volatile unsigned int input_calibration; // Stores value of raw calibration pin
volatile unsigned int input_multiplier;  // Stores value of raw multiplier pin

volatile unsigned long throttle_rising;
volatile unsigned long throttle_falling;
volatile unsigned long steering_rising;
volatile unsigned long steering_falling;
volatile unsigned long estop_rising;
volatile unsigned long estop_falling;
volatile unsigned long mode_rising;
volatile unsigned long mode_falling;
volatile unsigned long calibration_rising;
volatile unsigned long calibration_falling;
volatile unsigned long multiplier_rising;
volatile unsigned long multiplier_falling;

float processed_throttle;
float processed_steering;

volatile float manual_axis0_setpoint;
volatile float manual_axis1_setpoint;
volatile float command_axis0_setpoint;
volatile float command_axis1_setpoint;

IntervalTimer serial_update_timer;       // Timer for getting new data from odrive serial
IntervalTimer autonomous_feedback_timer; // Timer for requesting encoder data from odrive serial
IntervalTimer autonomous_control_timer;  // Timer for writing velocity to odrive serial
IntervalTimer background_timer;          // Timer for background tasks

const int serial_buffer_length = 1000;
volatile char serial_measurement_buffer[serial_buffer_length];
volatile int serial_measurement_buffer_index = 0;
volatile bool waiting_for_axis0 = false;
volatile bool waiting_for_axis1 = false;
volatile bool check_for_measurement = false;
volatile char serial_command_buffer[serial_buffer_length];
volatile int serial_command_buffer_index = 0;

volatile float axis0_estimate_position;
volatile float axis0_estimate_velocity;
volatile float axis1_estimate_position;
volatile float axis1_estimate_velocity;

const int user_blink_period = 500; // Every 500ms
volatile bool user_blink_enable = false;
volatile unsigned long user_blink_last = 0;
volatile bool user_blink_state = false;

const int messaage_checking_period = 1000; // Every 1 seconds
volatile bool message_checking_enable = false;
volatile unsigned long message_checking_last = 0;
volatile float message_checking_axis0;
volatile float message_checking_axis1;

const int connected_checking_period = 1000; // Every 1 seconds
volatile bool connected_checking_enable = false;
volatile unsigned long connected_checking_last = 0;

volatile bool run_odrive_calibration = false;
volatile bool run_remote_calibration = false;
volatile bool running_calibration = false;

volatile bool flag = false;

volatile bool feedback = false;

volatile float drive_multiplier = 0.1;

#endif
