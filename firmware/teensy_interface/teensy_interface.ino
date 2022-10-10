// Compilation configuration:
// Board: Teensy 4.1
// USB Type: Triple Serial
// CPU Speed: 600 MHz
// Optimize: Faster
// Keyboard Layout: US English

#define DEBUG_PRINT
#define LOG_PRINT

#include <Arduino.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include "custom_variables.h"
#include "custom_functions.h"
#include "custom_odrive.h"
#include "custom_print.h"
#include "pinout.h"
#include "custom_math.h"
#include "state_machine.h"

void setup()
{
    log_setup();
    debug_setup();
    feedback_stream_setup();

    debug_printlnm("Starting up...");
    delay(5000);

    debug_printlnm("Setting up ODrive serial at 500000 baud...");
    odrive_serial.begin(500000);

    debug_printlnm("Setting up remote controller pins...");
    pinMode(throttle_pin, INPUT);
    pinMode(steering_pin, INPUT);
    pinMode(estop_pin, INPUT);
    pinMode(mode_pin, INPUT);
    pinMode(calibration_pin, INPUT);
    pinMode(multiplier_pin, INPUT);

    debug_printlnm("Setting up indicator pins...");
    pinMode(indicator_green_pin, OUTPUT);
    pinMode(indicator_yellow_pin, OUTPUT);
    pinMode(indicator_red_pin, OUTPUT);
    digitalWrite(indicator_green_pin, HIGH);
    digitalWrite(indicator_yellow_pin, HIGH);
    digitalWrite(indicator_red_pin, HIGH);

    debug_printlnm("Setting up wam relay pin...");
    pinMode(wam_pin, OUTPUT);
    digitalWrite(wam_pin, LOW);

    debug_printlnm("Setting up built-in indicator pin...");
    pinMode(teensy_indicator_pin, OUTPUT);
    digitalWrite(teensy_indicator_pin, LOW);

    debug_printlnm("Setting up debug timing pins...");
    pinMode(led0, OUTPUT);
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
    pinMode(led4, OUTPUT);
    digitalWrite(led0, LOW);
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);

    debug_printlnm("Setting up hardware interrupts...");
    attachInterrupt(digitalPinToInterrupt(throttle_pin), isr_throttle, CHANGE);
    attachInterrupt(digitalPinToInterrupt(steering_pin), isr_steering, CHANGE);
    attachInterrupt(digitalPinToInterrupt(estop_pin), isr_estop, CHANGE);
    attachInterrupt(digitalPinToInterrupt(mode_pin), isr_mode, CHANGE);
    attachInterrupt(digitalPinToInterrupt(calibration_pin), isr_calibration, CHANGE);
    attachInterrupt(digitalPinToInterrupt(multiplier_pin), isr_multiplier, CHANGE);

    debug_printlnm("Setting up timers...");
    serial_update_timer.begin(serial_update_function, serial_update_period);
    background_timer.begin(background_timer_function, background_timer_period);

    debug_printlnm("Setting up calibration...");
    EEPROM.get(0, calibration);
    debug_print_calibration();

    debug_printlnm("Setting up remote controller...");
    if ((input_throttle == 0) || (input_steering == 0) || (input_estop == 0) || (input_mode == 0))
    {
        debug_printlnm("Waiting for remote controller connection...");
        user_blink_enable = true;
    }
    while ((input_throttle == 0) || (input_steering == 0) || (input_estop == 0) || (input_mode == 0))
    {
        // Wait for interrupts to update the input variables
    }
    user_blink_enable = false;

    debug_printlnm("Verifying configuration...");

    // Setup validity variables
    bool calibration_valid = false;
    bool throttle_valid = false;
    bool steering_valid = false;

    // Check if configuration does not exist
    if (calibration.number_samples <= 0)
    {
        // Initialize calibration variable defaults and require calibration
        calibration = CalibrationConfig();
        calibration_valid = false;
    }
    else
    {
        // Calibration is valid
        calibration_valid = true;

        // Check if throttle and steering values are within last calibration ranges
        throttle_valid = (input_throttle <= (calibration.throttle_zero_offset + calibration.throttle_zero_max)) && (input_throttle >= (calibration.throttle_zero_offset + calibration.throttle_zero_min));
        steering_valid = (input_steering <= (calibration.steering_zero_offset + calibration.steering_zero_max)) && (input_steering >= (calibration.steering_zero_offset + calibration.steering_zero_min));
    }

    debug_printlnv("Configuration valid", calibration_valid);

    // If calibration not valid or current inputs not within calibration ranges then require calibration
    if ((!throttle_valid) || (!steering_valid) || (!calibration_valid))
    {
        debug_printm();
        debug_printt("Calibration required: ");
        debug_print(calibration_valid);
        debug_print(throttle_valid);
        debug_println(steering_valid);

        // Run remote calibration
        remote_calibration();
    }

    // debug_printlnm("Running remote calibration anyway.");
    // remote_calibration();

    debug_printlnm("Setup complete!");
}

void loop()
{
    // Everything is done in interrupts and timers

    // Check if need to run calibration
    if (run_remote_calibration)
    {
        debug_printlnm("Running remote calibration...");

        // Run remote calibration
        remote_calibration();
    }
    if (run_odrive_calibration)
    {
        debug_printlnm("Running orive calibration...");

        // Run odrive calibration
        odrive_calibration();

        // Update flag
        run_odrive_calibration = false;
    }
}

/* Calibration steps:
   1. Wait for all values to be nonzero
   2. Calibrate zero for throttle
   3. Calibrate zero for steering
   4. Calibrate max throttle (toward handle)
   5. Calibrate min throttle (away from handle)
   6. Calibrate min steering (clockwise)
   7. Calibrate max steering (counterclockwise)
*/
void remote_calibration()
{
    debug_printlnm("Entering remote_calibration...");

    // Set indicator as reading zeros
    digitalWrite(teensy_indicator_pin, HIGH);
    debug_printlnm("Reading throttle and steering zero...");

    // Create calibration variables
    running_calibration = true;
    int temp;
    int min = input_throttle;
    int max = input_throttle;
    float sum = 0;

    debug_printlnv("Number of samples", calibration.number_samples);

    // Get throttle samples
    for (int i = 0; i < calibration.number_samples; i++)
    {
        temp = get_next_throttle();

        // Find min, max, and zero
        min = min(min, temp);
        max = max(max, temp);
        sum += temp;
    }

    // Calculate zero, zero min, and zero max
    calibration.throttle_zero_offset = sum / calibration.number_samples;
    calibration.throttle_zero_min = min - calibration.throttle_zero_offset - 1;
    calibration.throttle_zero_max = max - calibration.throttle_zero_offset + 1;

    // Reset calibration variables
    min = input_steering;
    max = input_steering;
    sum = 0;

    // Get steering samples
    for (int i = 0; i < calibration.number_samples; i++)
    {
        temp = get_next_steering();

        // Find min, max, and zero
        min = min(min, temp);
        max = max(max, temp);
        sum += temp;
    }

    // Calculate zero, zero min, and zero max
    calibration.steering_zero_offset = sum / calibration.number_samples;
    calibration.steering_zero_min = min - calibration.steering_zero_offset - 1;
    calibration.steering_zero_max = max - calibration.steering_zero_offset + 1;

    // Set initial values for min and max
    calibration.throttle_min = input_throttle;
    calibration.throttle_max = input_throttle;
    calibration.steering_min = input_steering;
    calibration.steering_max = input_steering;

    // Set indicator LED as waiting for max throttle
    digitalWrite(teensy_indicator_pin, LOW);
    debug_printlnm("Waiting for max throttle...");

    // Wait for throttle to exceed zero threshold
    while (input_throttle < (calibration.throttle_zero_offset + calibration.throttle_zero_max + throttle_threshold_band))
    {
        // Wait for next throttle
        get_next_throttle();
    }

    // Set indicator LED as reading max throttle
    digitalWrite(teensy_indicator_pin, HIGH);
    debug_printlnm("Reading max throttle...");

    for (int i = 0; i < calibration.number_samples; i++)
    {
        // Wait for next throttle
        get_next_throttle();

        // Store max
        calibration.throttle_max = max(calibration.throttle_max, input_throttle);
    }

    // Set indicator LED as waiting for min throttle
    digitalWrite(teensy_indicator_pin, LOW);
    debug_printlnm("Waiting for min throttle...");

    // Wait for throttle to exceed zero threshold
    while (input_throttle > (calibration.throttle_zero_offset + calibration.throttle_zero_min - throttle_threshold_band))
    {
        // Wait for next throttle
        get_next_throttle();
    }

    // Set indicator LED as reading min throttle
    digitalWrite(teensy_indicator_pin, HIGH);
    debug_printlnm("Reading min throttle...");

    for (int i = 0; i < calibration.number_samples; i++)
    {
        // Wait for next throttle
        get_next_throttle();

        // Store min
        calibration.throttle_min = min(calibration.throttle_min, input_throttle);
    }

    // Set indicator LED as waiting for min steering
    digitalWrite(teensy_indicator_pin, LOW);
    debug_printlnm("Waiting for min steering...");

    // Wait for steering to exceed zero threshold
    while (input_steering > (calibration.steering_zero_offset + calibration.steering_zero_min - steering_threshold_band))
    {
        // Wait for next steering
        get_next_steering();
    }

    // Set indicator LED as reading min steering
    digitalWrite(teensy_indicator_pin, HIGH);
    debug_printlnm("Reading min steering...");

    for (int i = 0; i < calibration.number_samples; i++)
    {
        // Wait for next steering
        get_next_steering();

        // Store min
        calibration.steering_min = min(calibration.steering_min, input_steering);
    }

    // Set indicator LED as waiting for max steering
    digitalWrite(teensy_indicator_pin, LOW);
    debug_printlnm("Waiting for max steering...");

    // Wait for steering to exceed zero threshold
    while (input_steering < (calibration.steering_zero_offset + calibration.steering_zero_max + steering_threshold_band))
    {
        // Wait for next steering
        get_next_steering();
    }

    // Set indicator LED as reading max steering
    digitalWrite(teensy_indicator_pin, HIGH);
    debug_printlnm("Reading max steering...");

    for (int i = 0; i < calibration.number_samples; i++)
    {
        // Wait for next steering
        get_next_steering();

        // Store max
        calibration.steering_max = max(calibration.steering_max, input_steering);
    }

    debug_printlnm("Calculating...");

    // Add calibration adjustments
    calibration.throttle_min -= throttle_threshold_band;
    calibration.throttle_max += throttle_threshold_band;
    calibration.throttle_zero_min -= throttle_threshold_band;
    calibration.throttle_zero_max += throttle_threshold_band;
    calibration.steering_min -= steering_threshold_band;
    calibration.steering_max += steering_threshold_band;
    calibration.steering_zero_min -= steering_threshold_band;
    calibration.steering_zero_max += steering_threshold_band;

    // Write calibration to EEPROM
    EEPROM.put(0, calibration);
    debug_printlnm("EEPROM updated");

    // Wait for calibration knob to return to normal
    if (run_remote_calibration)
    {
        debug_printlnm("Waiting for calibration knob to return to normal...");

        // Blink yellow light
        user_blink_enable = true;
    }
    while (run_remote_calibration)
    {
        // Blink yellow light
    }

    // Stop blinking light
    user_blink_enable = false;

    // Finished running calibration
    running_calibration = false;

    // Set indicator LED as idle
    digitalWrite(teensy_indicator_pin, LOW);
    debug_printlnm("Calibration complete.");
    debug_print_calibration();

    // debug_printlnm("Exiting remote_calibration...");
}

// ISR for handling state change on throttle pin
void isr_throttle()
{
    // debug_printlnm("Entering isr_throttle...");
    // digitalWrite(led1, HIGH);

    if (digitalRead(throttle_pin)) // Pin has gone high
    {
        // Store rising time
        throttle_rising = micros();
    }
    else // Pin has gone low
    {
        // Store falling time
        throttle_falling = micros();

        // Check if pulse caused by unrelated delay
        if ((throttle_falling - throttle_rising) > max_pulse_length)
        {
            // Unreasonably long pulse
            debug_printlnv("Throttle pulse was too long", (throttle_falling - throttle_rising));

            return;
        }
        else if ((throttle_falling - throttle_rising) < min_pulse_length)
        {
            // Unreasonably short pulse
            debug_printlnv("Throttle pulse was too short", (throttle_falling - throttle_rising));

            return;
        }

        // Calculate pulse on time
        input_throttle = throttle_falling - throttle_rising;
        // debug_printlnv("Input throttle", input_throttle);
    }

    // digitalWrite(led1, LOW);
    // debug_printlnm("Exiting isr_throttle...");
}

// ISR for handling state change on steering pin
void isr_steering()
{
    // debug_printlnm("Entering isr_steering...");
    // digitalWrite(led2, HIGH);

    if (digitalRead(steering_pin)) // Pin has gone high
    {
        // Store rising time
        steering_rising = micros();
    }
    else // Pin has gone low
    {
        // Store falling time
        steering_falling = micros();

        // Check if pulse caused by unrelated delay
        if ((steering_falling - steering_rising) > max_pulse_length)
        {
            // Unreasonably long pulse
            debug_printlnv("Steering pulse was too long", (steering_falling - steering_rising));

            return;
        }
        else if ((steering_falling - steering_rising) < min_pulse_length)
        {
            // Unreasonably short pulse
            debug_printlnv("Steering pulse was too short", (steering_falling - steering_rising));

            return;
        }

        // Reasonable pulse

        // Calculate pulse on time
        input_steering = steering_falling - steering_rising;
        // debug_printlnv("Input steering", input_steering);

        // Check if state is manual
        if (sm.state == STATE_MANUAL)
        {
            // Run RC update
            sm.rc_update();
        }
    }

    // digitalWrite(led2, LOW);
    // debug_printlnm("Exiting isr_steering...");
}

// ISR for handling state change on estop pin
void isr_estop()
{
    // debug_printlnm("Entering isr_estop...");
    // digitalWrite(led3, HIGH);

    if (digitalRead(estop_pin)) // Pin has gone high
    {
        // Store rising time
        estop_rising = micros();
    }
    else // Pin has gone low
    {
        // Store falling time
        estop_falling = micros();

        // Check if pulse caused by unrelated delay
        if ((estop_falling - estop_rising) > max_pulse_length)
        {
            // Unreasonably long pulse
            debug_printlnv("Estop pulse was too long", (estop_falling - estop_rising));

            return;
        }
        else if ((estop_falling - estop_rising) < min_pulse_length)
        {
            // Unreasonably short pulse
            debug_printlnv("Estop pulse was too short", (estop_falling - estop_rising));

            return;
        }

        // Resonable pulse

        // Calculate pulse on time
        input_estop = estop_falling - estop_rising;
        // debug_printlnv("Input estop", input_estop);

        if (between(estop_threshold_on - pulse_threshold_band, input_estop, estop_threshold_on + pulse_threshold_band))
        {
            // Estop desired
            sm.set_state(STATE_ESTOP);
        }
        else if (between(estop_threshold_off - pulse_threshold_band, input_estop, estop_threshold_off + pulse_threshold_band))
        {
            // Estop not desired
            sm.set_state(STATE_ESTOP_RECOVER);
        }
        else
        {
            // Invalid estop
            debug_printlnv("Estop pulse was invalid", input_estop);
        }
    }

    // digitalWrite(led3, LOW);
    // debug_printlnm("Exiting isr_estop...");
}

// ISR for handling state change on mode pin
void isr_mode()
{
    // debug_printlnm("Entering isr_mode...");
    // digitalWrite(led4, HIGH);

    if (digitalRead(mode_pin)) // Pin has gone high
    {
        // Store rising time
        mode_rising = micros();
    }
    else // Pin has gone low
    {
        // Store falling time
        mode_falling = micros();

        // Check if pulse caused by unrelated delay
        if ((mode_falling - mode_rising) > max_pulse_length)
        {
            // Unreasonably long pulse
            debug_printlnv("Mode pulse was too long", (mode_falling - mode_rising));

            return;
        }
        else if ((mode_falling - mode_rising) < min_pulse_length)
        {
            // Unreasonably short pulse
            debug_printlnv("Mode pulse was too short", (mode_falling - mode_rising));

            return;
        }

        // Resonable pulse

        // Calculate pulse on time
        input_mode = mode_falling - mode_rising;
        // debug_printlnv("Input mode", input_mode);

        if (between(mode_threshold_idle - pulse_threshold_band, input_mode, mode_threshold_idle + pulse_threshold_band))
        {
            // Idle desired
            sm.set_state(STATE_IDLE);
        }
        else if (between(mode_threshold_manual - pulse_threshold_band, input_mode, mode_threshold_manual + pulse_threshold_band))
        {
            // Manual desired
            sm.set_state(STATE_MANUAL);
        }
        else if (between(mode_threshold_auto - pulse_threshold_band, input_mode, mode_threshold_auto + pulse_threshold_band))
        {
            // Autonomous desired
            sm.set_state(STATE_AUTONOMOUS);
        }
        else
        {
            // Invalid mode
            debug_printlnv("Mode pulse was invalid", input_mode);
        }

        log_print_info();
    }

    // digitalWrite(led4, LOW);
    // debug_printlnm("Exiting isr_mode...");
}

// ISR for handling state change on calibration pin
void isr_calibration()
{
    // debug_printlnm("Entering isr_calibration...");
    // digitalWrite(led1, HIGH);

    if (digitalRead(calibration_pin)) // Pin has gone high
    {
        // Store rising time
        calibration_rising = micros();
    }
    else // Pin has gone low
    {
        // Store falling time
        calibration_falling = micros();

        // Check if pulse caused by unrelated delay
        if ((calibration_falling - calibration_rising) > max_pulse_length)
        {
            // Unreasonably long pulse
            debug_printlnv("Calibration pulse was too long", (calibration_falling - calibration_rising));

            return;
        }
        else if ((calibration_falling - calibration_rising) < min_pulse_length)
        {
            // Unreasonably short pulse
            debug_printlnv("Calibration pulse was too short", (calibration_falling - calibration_rising));

            return;
        }

        // Calculate pulse on time
        input_calibration = calibration_falling - calibration_rising;
        // debug_printlnv("Input calibration", input_calibration);

        // Remote calibration request detected
        if (between(calibration_threshold_low - pulse_threshold_band, input_calibration, calibration_threshold_low + pulse_threshold_band))
        {
            // Ensure in idle state
            if (sm.state == STATE_IDLE)
            {
                debug_printlnm("Remote calibration request detected");

                // Update flag
                run_remote_calibration = true;
            }
        }
        // ODrive calibration request detected
        else if (between(calibration_threshold_high - pulse_threshold_band, input_calibration, calibration_threshold_high + pulse_threshold_band))
        {
            // Ensure in idle state
            if (sm.state == STATE_IDLE)
            {
                debug_printlnm("ODrive calibration request detected");

                // Update flag
                run_odrive_calibration = true;
            }
        }
        else
        {
            // Update flags
            run_remote_calibration = false;
            run_odrive_calibration = false;
        }

        // digitalWrite(led1, LOW);
        // debug_printlnm("Exiting isr_calibration...");
    }
}

// ISR for handling state change on multiplier pin
void isr_multiplier()
{
    // debug_printlnm("Entering isr_multiplier...");
    // digitalWrite(led1, HIGH);

    if (digitalRead(multiplier_pin)) // Pin has gone high
    {
        // Store rising time
        multiplier_rising = micros();
    }
    else // Pin has gone low
    {
        // Store falling time
        multiplier_falling = micros();

        // Check if pulse caused by unrelated delay
        if ((multiplier_falling - multiplier_rising) > max_pulse_length)
        {
            // Unreasonably long pulse
            debug_printlnv("Multiplier pulse was too long", (multiplier_falling - multiplier_rising));

            return;
        }
        else if ((multiplier_falling - multiplier_rising) < min_pulse_length)
        {
            // Unreasonably short pulse
            debug_printlnv("Multiplier pulse was too short", (multiplier_falling - multiplier_rising));

            return;
        }

        // Calculate pulse on time
        input_multiplier = multiplier_falling - multiplier_rising;
        // debug_printlnv("Input multiplier", input_multiplier);

        // Constrain input
        input_multiplier = constrain(input_multiplier, 850, 2150);

        // Map input to multiplier
        drive_multiplier = fmap(input_multiplier, 850, 2150, 0.1, 1.0);
        // debug_println(drive_multiplier);

        log_print_info();

        // digitalWrite(led1, LOW);
        // debug_printlnm("Exiting isr_multiplier...");
    }
}

// Waits for new throttle pulse
unsigned int get_next_throttle()
{
    // debug_printlnm("Entering get_next_throttle...");

    // Store current throttle temporarily
    unsigned int temp = input_throttle;

    // Ensure new sample is found
    while (temp == input_throttle)
    {
    }

    // debug_printlnm("Exiting get_next_throttle...");

    // Return new sample
    return input_throttle;
}

// Waits for new steering pulse
unsigned int get_next_steering()
{
    // debug_printlnm("Entering get_next_steering...");

    // Store current steering temporarily
    unsigned int temp = input_steering;

    // Ensure new sample is found
    while (temp == input_steering)
    {
    }

    // debug_printlnm("Exiting get_next_steering...");

    // Return new sample
    return input_steering;
}

// Waits for new estop pulse
unsigned int get_next_estop()
{
    // debug_printlnm("Entering get_next_estop...");

    // Store current estop temporarily
    unsigned int temp = input_estop;

    // Ensure new sample is found
    while (temp == input_estop)
    {
    }

    // debug_printlnm("Exiting get_next_estop...");

    // Return new sample
    return input_estop;
}

// Waits for new mode pulse
unsigned int get_next_mode()
{
    // debug_printlnm("Entering get_next_mode...");

    // Store current mode temporarily
    unsigned int temp = input_mode;

    // Ensure new sample is found
    while (temp == input_mode)
    {
    }

    // debug_printlnm("Exiting get_next_mode...");

    // Return new sample
    return input_mode;
}

// Waits for new configuration pulse
unsigned int get_next_calibration()
{
    // debug_printlnm("Entering get_next_calibration...");

    // Store current calibration temporarily
    unsigned int temp = input_calibration;

    // Ensure new sample is found
    while (temp == input_calibration)
    {
    }

    // debug_printlnm("Exiting get_next_calibration...");

    // Return new sample
    return input_calibration;
}

// Waits for new multiplier pulse
unsigned int get_next_multiplier()
{
    // debug_printlnm("Entering get_next_multiplier...");

    // Store current multiplier temporarily
    unsigned int temp = input_multiplier;

    // Ensure new sample is found
    while (temp == input_multiplier)
    {
    }

    // debug_printlnm("Exiting get_next_multiplier...");

    // Return new sample
    return input_multiplier;
}

// Uses input_throttle and input_steering to calculate individual axis velocities
void diff_drive()
{
    // debug_printlnm("Entering diff_drive...");

    // Convert throttle [m/s] and steering [rad/s] to wheel [rads/s]
    float desired_axis0 = (processed_throttle + (processed_steering * wheel_separation * 0.5f)) / (2.0f * M_PI * wheel_radius);
    float desired_axis1 = (processed_throttle - (processed_steering * wheel_separation * 0.5f)) / (2.0f * M_PI * wheel_radius);

    // Convert wheel [rev/s] to motor [rev/s]
    desired_axis0 = desired_axis0 * motor_to_wheel_ratio;
    desired_axis1 = desired_axis1 * motor_to_wheel_ratio;

    // Constrain setpoints
    manual_axis0_setpoint = constrain(-desired_axis0, -axis_max_velocity, axis_max_velocity);
    manual_axis1_setpoint = constrain(desired_axis1, -axis_max_velocity, axis_max_velocity);

    // debug_printlnm("Exiting diff_drive...");
}

// Function to handle data coming from the Computer
void serial_update_function()
{
    // debug_printlnm("Entering serial_update_function...");
    // digitalWrite(led1, HIGH);

    // Copy all new data to the buffer
    int string_end = -1;

    // Get new characters and check if string complete
    while (SerialUSB.available())
    {
        // Get new character from serial
        char c = SerialUSB.read();

        // Add character to buffer
        serial_command_buffer[serial_command_buffer_index] = c;

        // Check if complete string received
        if (string_end == -1 && c == '\n')
        {
            // Set end of string
            string_end = serial_command_buffer_index;
            break;
        }

        // Update buffer index
        serial_command_buffer_index++;

        // Check if buffer is full
        if (serial_command_buffer_index > serial_buffer_length)
        {
            debug_printlnm("Autonomous buffer is full, resetting buffer.");
            serial_command_buffer_index = 0;
        }
    }

    // Parse new found string
    if (string_end > 0)
    {
        // Example Message (V0 V1): 0.45 1.25\n

        // Get substring to be parsed
        char string_to_parse[string_end];
        for (int i = 0; i < string_end; i++)
        {
            string_to_parse[i] = serial_command_buffer[i];
        }

        // Stores resulting substring
        char *second_substring;

        // Get first float (position estimate)
        command_axis0_setpoint = strtof(string_to_parse, &second_substring);

        // Get second float (velocity estimate)
        command_axis1_setpoint = strtof(second_substring, NULL);

        // Reset buffer index
        serial_command_buffer_index = 0;

        float command_axis0_setpoint1 = command_axis0_setpoint * 20.0f / 2.0f / M_PI;
        float command_axis1_setpoint2 = command_axis1_setpoint * 20.0f / 2.0f / M_PI;

        // Write out most recent values
        if (sm.state == STATE_AUTONOMOUS)
        {
            // Send new autonomous velocities to ODrive
            odrive_write_velocities(command_axis0_setpoint1, command_axis1_setpoint2);
        }
    }


    // digitalWrite(led1, LOW);
    // debug_printlnm("Exiting serial_update_function...");
}

// Function for running a series of background tasks
void background_timer_function()
{
    // debug_printlnm("Entering background_timer_function...");
    // digitalWrite(led4, HIGH);

    // Blink user indicator
    if (user_blink_enable)
    {
        // Check if timeout completed
        if ((millis() - user_blink_last) > user_blink_period)
        {
            // Store new state
            user_blink_state = !user_blink_state;

            // Store new time
            user_blink_last = millis();

            // Output new state
            digitalWrite(indicator_yellow_pin, user_blink_state);
        }
    }

    // Check recent messages
    if (message_checking_enable)
    {
        if (sm.state >= STATE_MANUAL)
        {

            // Check if inputs have changed
            if ((message_checking_axis0 != manual_axis0_setpoint) and (message_checking_axis1 != manual_axis1_setpoint))
            {
                // Update last
                message_checking_last = millis();
            }

            // Update states
            message_checking_axis0 = manual_axis0_setpoint;
            message_checking_axis1 = manual_axis1_setpoint;

            // Check if timeout completed
            if ((millis() - message_checking_last) > messaage_checking_period)
            {
                debug_printlnm("Timeout exceeded, setting motors to zero");

                // Set velocities to zero
                manual_axis0_setpoint = 0;
                manual_axis1_setpoint = 0;
                odrive_write_velocities(manual_axis0_setpoint, manual_axis1_setpoint);
            }
        }
    }
}

// Function for enabling and disabling brakes
void brakes(bool enable)
{
}

// Function for enabling and disabling wam power
void wam(bool enable)
{
    digitalWrite(wam_pin, !enable);
}

// Prints raw values from calibration object
void debug_print_calibration()
{
    debug_print("Calibration: \t");
    debug_printt(calibration.throttle_min);
    debug_printt(calibration.throttle_max);
    debug_printt(calibration.throttle_zero_offset);
    debug_printt(calibration.throttle_zero_min);
    debug_printt(calibration.throttle_zero_max);
    debug_printt(calibration.steering_min);
    debug_printt(calibration.steering_max);
    debug_printt(calibration.steering_zero_offset);
    debug_printt(calibration.steering_zero_min);
    debug_println(calibration.steering_zero_max);
}

// Prints useful debugging information
void log_print_info()
{
    log_print(micros());
    log_print(" State: ");
    log_print(sm.state);
    log_print(" RC-V0: ");
    log_print(manual_axis0_setpoint);
    log_print(" RC-V1: ");
    log_print(manual_axis1_setpoint);
    log_print(" Auto-V0: ");
    log_print(command_axis0_setpoint);
    log_print(" Auto-V1: ");
    log_print(command_axis1_setpoint);
    log_pln();
}
