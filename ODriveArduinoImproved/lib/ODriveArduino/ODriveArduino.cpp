
#include "Arduino.h"
#include "ODriveArduino.h"

static const int kMotorOffsetFloat = 2;
static const int kMotorStrideFloat = 28;
static const int kMotorOffsetInt32 = 0;
static const int kMotorStrideInt32 = 4;
static const int kMotorOffsetBool = 0;
static const int kMotorStrideBool = 4;
static const int kMotorOffsetUint16 = 0;
static const int kMotorStrideUint16 = 2;

// Print with stream operator
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
    obj.print(arg);
    return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
    obj.print(arg, 4);
    return obj;
}

ODriveArduino::ODriveArduino(Stream &serial)
    : serial_(serial) {
        config0 = new ODriveConfig(0);
        config1 = new ODriveConfig(1);
    }

void ODriveArduino::SetPosition(int motor_number, float position)
{
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward)
{
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward)
{
    serial_ << "p " << motor_number << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODriveArduino::SetVelocity(int motor_number, float velocity)
{
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward)
{
    serial_ << "v " << motor_number << " " << velocity << " " << current_feedforward << "\n";
}

void ODriveArduino::SetCurrent(int motor_number, float current)
{
    serial_ << "c " << motor_number << " " << current << "\n";
}

void ODriveArduino::SetEncoderAbs(int motor_number, int32_t pos_abs)
{
    serial_ << "w axis" << motor_number << ".encoder.pos_abs " << pos_abs << "\n";
}

void ODriveArduino::SetLinearCount(int motor_number, int32_t count)
{
    serial_ << "w axis." << motor_number << ".encoder.set_linear_count(" << count << ")\n";
}

void ODriveArduino::TrapezoidalMove(int motor_number, float position)
{
    serial_ << "t " << motor_number << " " << position << "\n";
}

void ODriveArduino::SetRequestedState(int motor_number, AxisState_t requested_state)
{
    serial_ << "w axis" << motor_number << ".requested_state " << requested_state << '\n';
}
void ODriveArduino::move_incremental(int motor_number, float displacement, bool from_input_pos)
{
    serial_ << "w axis" << motor_number << ".controller.move_incremental(" << displacement << " , " << from_input_pos << ")\n";
}
void ODriveArduino::start_anticogging_calibration(int motor_number)
{
    serial_ << "w axis" << motor_number << ".start_anticogging_calibration()\n";
}
void ODriveArduino::watchdog_feed(int motor_number)
{
    serial_ << "w axis" << motor_number << ".watchdog_feed()\n";
}
void ODriveArduino::clear_errors(int motor_number)
{
    serial_ << "w axis" << motor_number << ".clear_errors()\n";
}

float ODriveArduino::GetVelocity(int motor_number)
{
    serial_ << "r axis" << motor_number << ".encoder.vel_estimate\n";
    return ODriveArduino::readFloat();
}

float ODriveArduino::GetPosition(int motor_number)
{
    serial_ << "r axis" << motor_number << ".encoder.pos_estimate_counts\n";
    return ODriveArduino::readFloat();
}

float ODriveArduino::GetCurrent(int motor_number)
{
    serial_ << "r axis" << motor_number << ".motor.current_control.IQ_measured\n";
    return ODriveArduino::readFloat();
}

float ODriveArduino::BusVoltage()
{
    serial_ << "r vbus_voltage\n";
    return ODriveArduino::readFloat();
}

ODriveArduino::AxisState_t ODriveArduino::GetCurrentState(int motor_number)
{
    serial_ << "r axis" << motor_number << ".current_state\n";
    return (AxisState_t)readInt();
}
ODriveArduino::AxisState_t ODriveArduino::GetRequestedState(int motor_number)
{
    serial_ << "r axis" << motor_number << ".requested_state\n";
    return (AxisState_t)readInt();
}

float ODriveArduino::GetTemperature(int motor_number)
{
    serial_ << "r axis" << motor_number << ".fet_thermistor.temperature\n";
    return readFloat();
}

bool ODriveArduino::GetMotorCalibrated(int motor_number)
{
    serial_ << "r axis" << motor_number << ".motor.is_calibrated\n";
    return (bool)readInt();
}

float ODriveArduino::GetPosSet(int motor_number)
{
    serial_ << "r axis" << motor_number << ".controller.pos_setpoint\n";
    return readFloat();
}
float ODriveArduino::GetVelSet(int motor_number)
{
    serial_ << "r axis" << motor_number << ".controller.vel_setpoint\n";
    return readFloat();
}
float ODriveArduino::GetCurrentSet(int motor_number)
{
    serial_ << "r axis" << motor_number << ".controller.torque_setpoint\n";
    return readFloat();
}

bool ODriveArduino::GetEncoderReady(int motor_number)
{
    serial_ << "r axis" << motor_number << ".encoder.is_ready\n";
    return (bool)readInt();
}

int32_t ODriveArduino::GetEncoderAbs(int motor_number)
{
    serial_ << "r axis" << motor_number << ".encoder.pos_abs\n";
    return (uint32_t)readInt();
}

int ODriveArduino::GetError(int motor_number)
{
    serial_ << "r axis" << motor_number << ".error\n";
    return readInt();
}
String ODriveArduino::GetErrorString(int motor_number)
{
    int error = GetError(motor_number);
    switch (error)
    {
    case ERROR_NONE:
        return "ERROR_NONE";
    case ERROR_INVALID_STATE:
        return "ERROR_INVALID_STATE";
    case ERROR_DC_BUS_UNDER_VOLTAGE:
        return "ERROR_INVALID_STATE";
    case ERROR_DC_BUS_OVER_VOLTAGE:
        return "ERROR_DC_BUS_OVER_VOLTAGE";
    case ERROR_CURRENT_MEASUREMENT_TIMEOUT:
        return "ERROR_CURRENT_MEASUREMENT_TIMEOUT";
    case ERROR_BRAKE_RESISTOR_DISARMED:
        return "ERROR_BRAKE_RESISTOR_DISARMED";
    case ERROR_MOTOR_DISARMED:
        return "ERROR_MOTOR_DISARMED";
    case ERROR_MOTOR_FAILED:
        return "ERROR_MOTOR_FAILED";
    case ERROR_SENSORLESS_ESTIMATOR_FAILED:
        return "ERROR_SENSORLESS_ESTIMATOR_FAILED";
    case ERROR_ENCODER_FAILED:
        return "ERROR_ENCODER_FAILED";
    case ERROR_CONTROLLER_FAILED:
        return "ERROR_CONTROLLER_FAILED";
    case ERROR_POS_CTRL_DURING_SENSORLESS:
        return "ERROR_POS_CTRL_DURING_SENSORLESS";
    case ERROR_WATCHDOG_TIMER_EXPIRED:
        return "ERROR_WATCHDOG_TIMER_EXPIRED";
    case ERROR_MIN_ENDSTOP_PRESSED:
        return "ERROR_MIN_ENDSTOP_PRESSED";
    case ERROR_MAX_ENDSTOP_PRESSED:
        return "ERROR_MAX_ENDSTOP_PRESSED";
    case ERROR_ESTOP_REQUESTED:
        return "ERROR_ESTOP_REQUESTED";
    case ERROR_HOMING_WITHOUT_ENDSTOP:
        return "ERROR_HOMING_WITHOUT_ENDSTOP";
    case ERROR_OVER_TEMP:
        return "ERROR_OVER_TEMP";
    default:
        return "Error finding error";
    }
}

int ODriveArduino::GetMotorError(int motor_number)
{
    serial_ << "r axis" << motor_number << "motor.error\n";
    return readInt();
}
String ODriveArduino::GetMotorErrorString(int motor_number)
{
    int error = GetMotorError(motor_number);
    switch (error)
    {
    case ERROR_NONE:
        return "ERROR_NONE";
    case ERROR_PHASE_RESISTANCE_OUT_OF_RANGE:
        return "ERROR_PHASE_RESISTANCE_OUT_OF_RANGE";
    case ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE:
        return "ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE";
    case ERROR_ADC_FAILED:
        return "ERROR_ADC_FAILED";
    case ERROR_DRV_FAULT:
        return "ERROR_DRV_FAULT";
    case ERROR_CONTROL_DEADLINE_MISSED:
        return "ERROR_CONTROL_DEADLINE_MISSED";
    case ERROR_NOT_IMPLEMENTED_MOTOR_TYPE:
        return "ERROR_NOT_IMPLEMENTED_MOTOR_TYPE";
    case ERROR_BRAKE_CURRENT_OUT_OF_RANGE:
        return "ERROR_BRAKE_CURRENT_OUT_OF_RANGE";
    case ERROR_MODULATION_MAGNITUDE:
        return "ERROR_MODULATION_MAGNITUDE";
    case ERROR_BRAKE_DEADTIME_VIOLATION:
        return "ERROR_BRAKE_DEADTIME_VIOLATION";
    case ERROR_UNEXPECTED_TIMER_CALLBACK:
        return "ERROR_UNEXPECTED_TIMER_CALLBACK";
    case ERROR_CURRENT_SENSE_SATURATION:
        return "ERROR_CURRENT_SENSE_SATURATION";
    case ERROR_CURRENT_LIMIT_VIOLATION:
        return "ERROR_CURRENT_LIMIT_VIOLATION";
    case ERROR_BRAKE_DUTY_CYCLE_NAN:
        return "ERROR_BRAKE_DUTY_CYCLE_NAN";
    case ERROR_DC_BUS_OVER_REGEN_CURRENT:
        return "ERROR_DC_BUS_OVER_REGEN_CURRENT";
    case ERROR_DC_BUS_OVER_CURRENT:
        return "ERROR_DC_BUS_OVER_CURRENT";
    default:
        return "Error finding error";
    }
}

int ODriveArduino::GetEncoderError(int motor_number)
{
    serial_ << "r axis" << motor_number << "encoder.error\n";
    return readInt();
}
String ODriveArduino::GetEncoderErrorString(int motor_number)
{
    int error = GetEncoderError(motor_number);
    switch (error)
    {
    case ERROR_NONE:
        return "ERROR_NONE";
    case ERROR_UNSTABLE_GAIN:
        return "ERROR_UNSTABLE_GAIN";
    case ERROR_CPR_POLEPAIRS_MISMATCH:
        return "ERROR_CPR_POLEPAIRS_MISMATCH";
    case ERROR_NO_RESPONSE:
        return "ERROR_NO_RESPONSE";
    case ERROR_UNSUPPORTED_ENCODER_MODE:
        return "ERROR_UNSUPPORTED_ENCODER_MODE";
    case ERROR_ILLEGAL_HALL_STATE:
        return "ERROR_ILLEGAL_HALL_STATE";
    case ERROR_INDEX_NOT_FOUND_YET:
        return "ERROR_INDEX_NOT_FOUND_YET";
    case ERROR_ABS_SPI_TIMEOUT:
        return "ERROR_ABS_SPI_TIMEOUT";
    case ERROR_ABS_SPI_COM_FAIL:
        return "ERROR_ABS_SPI_COM_FAIL";
    case ERROR_ABS_SPI_NOT_READY:
        return "ERROR_ABS_SPI_NOT_READY";
    default:
        return "Error finding error";
    }
}

bool ODriveArduino::run_state(int axis, AxisState_t requested_state, bool wait_for_idle, float timeout)
{
    int timeout_ctr = (int)(timeout * 10.0f);
    SetRequestedState(axis, requested_state);
    if (wait_for_idle)
    {
        do
        {
            delay(100);
        } while (GetCurrentState(axis) > AXIS_STATE_IDLE);
    }

    return timeout_ctr > 0;
}

String ODriveArduino::readString()
{
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;)
    {
        while (!serial_.available())
        {
            if (millis() - timeout_start >= timeout)
            {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}

float ODriveArduino::readFloat()
{
    return readString().toFloat();
}

int32_t ODriveArduino::readInt()
{
    return readString().toInt();
}



bool ODriveArduino::get_startup_motor_calibration(int motor_number){
    serial_ << "r axis" << motor_number << ".config.startup_motor_calibration\n";
    return (bool)readInt();
}
bool ODriveArduino::get_startup_encoder_index_search(int motor_number){
    serial_ << "r axis" << motor_number << ".config.startup_encoder_index_search\n";
    return (bool)readInt();
}
bool ODriveArduino::get_startup_encoder_offset_calibration(int motor_number){
    serial_ << "r axis" << motor_number << ".config.startup_encoder_offset_calibration\n";
    return (bool)readInt();
}
bool ODriveArduino::get_startup_closed_loop_control(int motor_number){
    serial_ << "r axis" << motor_number << ".config.startup_closed_loop_control\n";
    return (bool)readInt();
}
bool ODriveArduino::get_startup_sensorless_control(int motor_number){
    serial_ << "r axis" << motor_number << ".config.startup_sensorless_control\n";
    return (bool)readInt();
}
bool ODriveArduino::get_startup_homing(int motor_number){
    serial_ << "r axis" << motor_number << ".config.startup_homing\n";
    return (bool)readInt();
}
bool ODriveArduino::get_enable_step_dir(int motor_number){
    serial_ << "r axis" << motor_number << ".config.enable_step_dir\n";
    return (bool)readInt();
}
bool ODriveArduino::get_step_dir_always_on(int motor_number){
    serial_ << "r axis" << motor_number << ".config.step_dir_always_on\n";
    return (bool)readInt();
}

float ODriveArduino::get_patturns_per_step(int motor_number){
    serial_ << "r axis" << motor_number << ".config.patturns_per_step\n";
    return readFloat();
}
float ODriveArduino::get_watchdog_timeout(int motor_number){
    serial_ << "r axis" << motor_number << ".config.watchdog_timeout\n";
    return readFloat();
}
bool ODriveArduino::get_enable_watchdog(int motor_number){
    serial_ << "r axis" << motor_number << ".config.enable_watchdog\n";
    return (bool)readInt();
}
uint16_t ODriveArduino::get_step_gpio_pin(int motor_number){
    serial_ << "r axis" << motor_number << ".config.step_gpio_pin\n";
    return (uint16_t)readInt();
}
uint16_t ODriveArduino::get_dir_gpio_pin(int motor_number){
    serial_ << "r axis" << motor_number << ".config.dir_gpio_pin\n";
    return (uint16_t)readInt();
}
uint32_t ODriveArduino::get_can_node_id(int motor_number){
    serial_ << "r axis" << motor_number << ".config.can_node_id\n";
    return (uint32_t)readInt();
}
bool ODriveArduino::get_can_node_id_extended(int motor_number){
    serial_ << "r axis" << motor_number << ".config.can_node_id_extended\n";
    return (bool)readInt();
}
uint32_t ODriveArduino::get_can_heartbeat_rate_ms(int motor_number){
    serial_ << "r axis" << motor_number << ".config.can_heartbeat_rate_ms\n";
    return (uint32_t)readInt();
}

void ODriveArduino::set_startup_motor_calibration(int motor_number, bool start_motor_cal){
    serial_ << "w axis" << motor_number << ".config.startup_motor_calibration " << start_motor_cal << "\n";
}
void ODriveArduino::set_startup_encoder_index_search(int motor_number, bool start_index_search){
    serial_ << "w axis" << motor_number << ".config.startup_encoder_index_search " << start_index_search << "\n";
}
void ODriveArduino::set_startup_encoder_offset_calibration(int motor_number, bool start_offset_cal){
    serial_ << "w axis" << motor_number << ".config.startup_encoder_offset_calibration " << start_offset_cal  << "\n";
}
void ODriveArduino::set_startup_closed_loop_control(int motor_number, bool start_closed_loop){
    serial_ << "w axis" << motor_number << ".config.startup_closed_loop_control " << start_closed_loop << "\n";
}
void ODriveArduino::set_startup_sensorless_control(int motor_number, bool start_sensorless){
    serial_ << "w axis" << motor_number << ".config.startup_sensorless_control " << start_sensorless << "\n";
}
void ODriveArduino::set_startup_homing(int motor_number, bool start_homing){
    serial_ << "w axis" << motor_number << ".config.startup_homing " << start_homing << "\n";
}
void ODriveArduino::set_enable_step_dir(int motor_number, bool en_step_dir){
    serial_ << "w axis" << motor_number << ".config.enable_step_dir " << en_step_dir << "\n";
}
void ODriveArduino::set_step_dir_always_on(int motor_number, bool step_dir_al){
    serial_ << "w axis" << motor_number << ".config.step_dir_always_on " << step_dir_al << "\n";
}

void ODriveArduino::set_patturns_per_step(int motor_number, float pat_step){
    serial_ << "w axis" << motor_number << ".config.patturns_per_step " << pat_step << "\n";
}
void ODriveArduino::set_watchdog_timeout(int motor_number, float watch_timeout){
 serial_ << "w axis" << motor_number << ".config.watchdog_timeout " << watch_timeout << "\n";
}
void ODriveArduino::set_enable_watchdog(int motor_number, bool en_watch){
serial_ << "w axis" << motor_number << ".config.enable_watchdog " << en_watch << "\n";
}
void ODriveArduino::set_step_gpio_pin(int motor_number, uint16_t step_gpio){
serial_ << "w axis" << motor_number << ".config.step_gpio_pin " << step_gpio << "\n";
}
void ODriveArduino::set_dir_gpio_pin(int motor_number, uint16_t dir_gpio){
    serial_ << "w axis" << motor_number << ".config.dir_gpio_pin " << dir_gpio << "\n";
}
void ODriveArduino::set_can_node_id(int motor_number, uint32_t canID){
serial_ << "w axis" << motor_number << ".config.can_node_id " << canID << "\n";
}
void ODriveArduino::set_can_node_id_extended(int motor_number, bool extendedId){
 serial_ << "w axis" << motor_number << ".config.can_node_id_extended " << extendedId << "\n";
}
void ODriveArduino::set_can_heartbeat_rate_ms(int motor_number, uint32_t can_heartbeat){
 serial_ << "w axis" << motor_number << ".config.can_heartbeat_rate_ms " << can_heartbeat << "\n";
}
