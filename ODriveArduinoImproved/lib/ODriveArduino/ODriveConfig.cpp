#include "ODriveArduino.h"

bool ODriveConfig::get_startup_motor_calibration(int motor_number);
bool ODriveConfig::get_startup_encoder_index_search(int motor_number);
bool ODriveConfig::get_startup_encoder_offset_calibration(int motor_number);
bool ODriveConfig::get_startup_closed_loop_control(int motor_number);
bool ODriveConfig::get_startup_sensorless_control(int motor_number);
bool ODriveConfig::get_startup_homing(int motor_number);
bool ODriveConfig::get_enable_step_dir(int motor_number);
bool ODriveConfig::get_step_dir_always_on(int motor_number);

float ODriveConfig::get_patturns_per_step(int motor_number);
float ODriveConfig::get_watchdog_timeout(int motor_number);
bool ODriveConfig::get_enable_watchdog(int motor_number);
uint16_t ODriveConfig::get_step_gpio_pin(int motor_number);
uint16_t ODriveConfig::get_dir_gpio_pin(int motor_number);
uint32_t ODriveConfig::get_can_node_id(int motor_number);
bool ODriveConfig::get_can_node_id_extended(int motor_number);
uint32_t ODriveConfig::get_can_heartbeat_rate_ms(int motor_number);

void ODriveConfig::set_startup_motor_calibration(int motor_number, bool start_motor_cal);
void ODriveConfig::set_startup_encoder_index_search(int motor_number, bool start_index_search);
void ODriveConfig::set_startup_encoder_offset_calibration(int motor_number, bool start_offset_cal);
void ODriveConfig::set_startup_closed_loop_control(int motor_number, bool start_closed_loop);
void ODriveConfig::set_startup_sensorless_control(int motor_number, bool start_sensorless);
void ODriveConfig::set_startup_homing(int motor_number, bool start_homing);
void ODriveConfig::set_enable_step_dir(int motor_number, bool en_step_dir);
void ODriveConfig::set_step_dir_always_on(int motor_number, bool step_dir_al);

void ODriveConfig::set_patturns_per_step(int motor_number, float pat_step);
void ODriveConfig::set_watchdog_timeout(int motor_number, float watch_timeout);
void ODriveConfig::set_enable_watchdog(int motor_number, bool en_watch);
void ODriveConfig::set_step_gpio_pin(int motor_number, uint16_t step_gpio);
void ODriveConfig::set_dir_gpio_pin(int motor_number, uint16_t dir_gpio);
void ODriveConfig::set_can_node_id(int motor_number, uint32_t canID);
void ODriveConfig::set_can_node_id_extended(int motor_number, bool extendedId);
void ODriveConfig::set_can_heartbeat_rate_ms(int motor_number, uint32_t can_heartbeat);

ODriveConfig::ODriveConfig(int axis){
    motornum = axis;
}