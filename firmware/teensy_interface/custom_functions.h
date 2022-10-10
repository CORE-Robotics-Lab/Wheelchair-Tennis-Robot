#ifndef CUSTOM_FUNCTIONS_H
#define CUSTOM_FUNCTIONS_H

void remote_calibration();
void isr_throttle();
void isr_steering();
void isr_estop();
void isr_mode();
void isr_calibration();
void isr_multiplier();
unsigned int get_next_throttle();
unsigned int get_next_steering();
unsigned int get_next_estop();
unsigned int get_next_mode();
void diff_drive();
void serial_update_function();
void waiting_for_user_blink_function();
void waiting_for_user(bool waiting);
void brakes(bool enable);
void wam(bool enable);
void print_calibration();

#endif
