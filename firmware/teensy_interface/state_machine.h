#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "custom_functions.h"
#include "custom_odrive.h"
#include "custom_print.h"
#include "pinout.h"
#include "custom_math.h"
#include "custom_variables.h"

enum STATES
{
    STATE_UNKNOWN = -1,
    STATE_ESTOP = 0,
    STATE_ESTOP_RECOVER = 1,
    STATE_IDLE = 2,
    STATE_MANUAL = 3,
    STATE_AUTONOMOUS = 4,
    STATE_CALIBRATION = 10
};

class StateMachine
{
public:
    int state = STATE_UNKNOWN;

    void set_state(int desired_state)
    {
        if (desired_state != state)
        {
            switch (desired_state)
            {
            case STATE_UNKNOWN:
                // Startup state

                break;
            case STATE_ESTOP:
                // First time estop enabled

                // Set velocities zero
                odrive_write_velocities(0.0f, 0.0f);

                // Enable closed loop control
                odrive_request_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL);
                odrive_request_state(1, AXIS_STATE_CLOSED_LOOP_CONTROL);

                // Set velocities zero (again, just in case)
                odrive_write_velocities(0.0f, 0.0f);

                // Enable brakes
                brakes(true);

                // Disable wam
                wam(false);

                // Set indicator red
                digitalWrite(indicator_red_pin, LOW);
                digitalWrite(indicator_yellow_pin, HIGH);
                digitalWrite(indicator_green_pin, HIGH);

                // Disable blinking light
                user_blink_enable = false;

                // Set new state as estop
                state = STATE_ESTOP;

                break;
            case STATE_ESTOP_RECOVER:
                // Check if in estop mode
                if (state == STATE_ESTOP)
                {
                    // First time estop disabled

                    // Disable brakes
                    brakes(false);

                    // Enable wam
                    wam(true);

                    // Blink light to show waiting for user
                    user_blink_enable = true;

                    // Set new state as recover
                    state = STATE_ESTOP_RECOVER;
                }

                break;
            case STATE_IDLE:
                // First time state idle

                // Ensure estop goes to idle
                if (state != STATE_ESTOP)
                {

                    // Disable blinking light
                    user_blink_enable = false;

                    // Clear errors
                    odrive_clear_errors();

                    // Set indicator as in idle mode
                    digitalWrite(teensy_indicator_pin, LOW);

                    // Set indicator yellow
                    digitalWrite(indicator_red_pin, HIGH);
                    digitalWrite(indicator_yellow_pin, LOW);
                    digitalWrite(indicator_green_pin, HIGH);

                    // Set motor velocities zero
                    odrive_write_velocities(0, 0);

                    // Idle motors
                    odrive_request_state(0, AXIS_STATE_IDLE);
                    odrive_request_state(1, AXIS_STATE_IDLE);

                    // Set new state as idle
                    state = STATE_IDLE;
                }

                break;
            case STATE_MANUAL:
                // First time state manual

                // Ensure cleared estop goes to idle
                if ((state != STATE_ESTOP_RECOVER) && (state != STATE_ESTOP) && (state != STATE_UNKNOWN) && (!running_calibration))
                {
                    // Disable feedback timer momentarily
                    detachInterrupt(digitalPinToInterrupt(calibration_pin));
                    detachInterrupt(digitalPinToInterrupt(multiplier_pin));
                    waiting_for_axis0 = false;
                    waiting_for_axis1 = false;

                    // Set indicator as in manual mode
                    digitalWrite(teensy_indicator_pin, HIGH);

                    // Set indicator yellow/green
                    digitalWrite(indicator_red_pin, HIGH);
                    digitalWrite(indicator_yellow_pin, LOW);
                    digitalWrite(indicator_green_pin, LOW);

                    // Set motor velocities zero
                    odrive_write_velocities(0, 0);

                    // Set input mode velocity ramp
                    odrive_set_input_mode(INPUT_MODE_VEL_RAMP);

                    // Enable closed loop control
                    odrive_request_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL);
                    odrive_request_state(1, AXIS_STATE_CLOSED_LOOP_CONTROL);

                    // Reenable feedback timer momentarily
                    attachInterrupt(digitalPinToInterrupt(calibration_pin), isr_calibration, CHANGE);
                    attachInterrupt(digitalPinToInterrupt(multiplier_pin), isr_multiplier, CHANGE);
                    waiting_for_axis0 = false;
                    waiting_for_axis1 = false;

                    // Set new state as manual
                    state = STATE_MANUAL;
                }

                break;
            case STATE_AUTONOMOUS:
                // First time state autonomous

                // Ensure cleared estop goes to idle
                if ((state != STATE_ESTOP_RECOVER) && (state != STATE_ESTOP) && (state != STATE_UNKNOWN) && (!running_calibration))
                {
                    // Disable feedback timer momentarily
                    detachInterrupt(digitalPinToInterrupt(calibration_pin));
                    detachInterrupt(digitalPinToInterrupt(multiplier_pin));
                    waiting_for_axis0 = false;
                    waiting_for_axis1 = false;

                    // Set indicator as in autonomous mode
                    digitalWrite(teensy_indicator_pin, HIGH);

                    // Set indicator green
                    digitalWrite(indicator_red_pin, HIGH);
                    digitalWrite(indicator_yellow_pin, HIGH);
                    digitalWrite(indicator_green_pin, LOW);

                    // Set motor velocities zero
                    odrive_write_velocities(0, 0);

                    // Set input mode passthrough
                    odrive_set_input_mode(INPUT_MODE_PASSTHROUGH);

                    // Enable closed loop control
                    odrive_request_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL);
                    odrive_request_state(1, AXIS_STATE_CLOSED_LOOP_CONTROL);

                    // Flush Buffer
                    SerialUSB.flush();
                    serial_command_buffer_index = 0;

                    // Reenable feedback timer momentarily
                    attachInterrupt(digitalPinToInterrupt(calibration_pin), isr_calibration, CHANGE);
                    attachInterrupt(digitalPinToInterrupt(multiplier_pin), isr_multiplier, CHANGE);
                    waiting_for_axis0 = false;
                    waiting_for_axis1 = false;

                    // Set new state as autonomous
                    state = STATE_AUTONOMOUS;
                }

                break;
            }
        }
    }

    void rc_update()
    {
        // Constrain inputs
        int throttle_constrained = constrain(input_throttle, calibration.throttle_min, calibration.throttle_max);
        int steering_constrained = constrain(input_steering, calibration.steering_min, calibration.steering_max);

        // Initialize mapped throttle and steering
        float mapped_throttle = 0;
        float mapped_steering = 0;

        // Process throttle forward and backward separately
        if (throttle_constrained < (calibration.throttle_zero_min + calibration.throttle_zero_offset)) // Reverse
        {
            // debug_println("Reverse...");
            // Map input
            mapped_throttle = fmap(throttle_constrained, calibration.throttle_min, calibration.throttle_zero_offset + calibration.throttle_zero_min, -linear_velocity_max, 0);
        }
        else if (throttle_constrained > (calibration.throttle_zero_max + calibration.throttle_zero_offset)) // Forward
        {
            // debug_println("Forward...");
            // Map input
            mapped_throttle = fmap(throttle_constrained, calibration.throttle_zero_offset + calibration.throttle_zero_max, calibration.throttle_max, 0, linear_velocity_max);
        }
        else // Zero
        {
            // debug_println("Zero...");
            // Set velocities to zero
            mapped_throttle = 0;
        }

        // Process steering forward and backward separately
        if (steering_constrained < (calibration.steering_zero_min + calibration.steering_zero_offset)) // Reverse
        {
            // debug_println("Reverse...");
            // Map input
            mapped_steering = fmap(steering_constrained, calibration.steering_min, calibration.steering_zero_offset + calibration.steering_zero_min, angular_velocity_max, 0);
        }
        else if (steering_constrained > (calibration.steering_zero_max + calibration.steering_zero_offset)) // Forward
        {
            // debug_println("Forward...");
            // Map input
            mapped_steering = fmap(steering_constrained, calibration.steering_zero_offset + calibration.steering_zero_max, calibration.steering_max, 0, -angular_velocity_max);
        }
        else // Zero
        {
            // debug_println("Zero...");
            // Set velocities to zero
            mapped_steering = 0;
        }

        // Integrate multiplier on throttle and steering
        processed_throttle = mapped_throttle * drive_multiplier;
        processed_steering = mapped_steering * drive_multiplier;

        // log_println("Processed throttle: ");
        // log_printlnf(processed_throttle, 4);
        // log_println("Processed steering: ");
        // log_printlnf(processed_steering, 4);

        // Run differential drive model
        diff_drive();

        // log_println("Axis 0 setpoint: ");
        // log_println(axis0_setpoint);
        // log_println("Axis 1 setpoint: ");
        // log_println(axis1_setpoint);

        // Set velocities to calculated remote control velocities
        odrive_write_velocities(manual_axis0_setpoint, manual_axis1_setpoint);
    }
};

StateMachine sm;

#endif
