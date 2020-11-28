
#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"

/**
class ODriveConfig
{
public:
    bool startup_motor_calibration;
    bool startup_encoder_index_search;
    bool startup_encoder_offset_calibration;
    bool startup_closed_loop_control;
    bool startup_sensorless_control;
    bool startup_homing;
    bool enable_step_dir;
    bool step_dir_always_on;

    float patturns_per_step;
    float watchdog_timeout;
    bool enable_watchdog;
    uint16_t step_gpio_pin;
    uint16_t dir_gpio_pin;
    uint32_t can_node_id;
    bool can_node_id_extended;
    uint32_t can_heartbeat_rate_ms;

    ODriveConfig();
};
**/

class ODriveArduino
{
public:
    enum AxisState_t
    {
        AXIS_STATE_UNDEFINED = 0,                  //<! will fall through to idle
        AXIS_STATE_IDLE = 1,                       //<! disable PWM and do nothing
        AXIS_STATE_STARTUP_SEQUENCE = 2,           //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,  //<! run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,          //<! run motor calibration
        AXIS_STATE_SENSORLESS_CONTROL = 5,         //<! run sensorless control
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6,       //<! run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8,        //<! run closed loop control
        AXIS_STATE_LOCKIN_SPIN = 9,                //<! run lockpin spin
        AXIS_STATE_ENCODER_DIR_FIND = 10,          //<! run encoder direction search
        AXIS_STATE_HOMING = 11                     //<! run axis homing function
    };

    enum Error_t
    {
        ERROR_NONE = 0x00000000,
        ERROR_INVALID_STATE = 0x00000001,        //<! an invalid state was requested
        ERROR_DC_BUS_UNDER_VOLTAGE = 0x00000002, //<! the DC voltage fell below the limit configured in config.dc_bus_undervoltage_trip_level
        ERROR_DC_BUS_OVER_VOLTAGE = 0x00000004,  //<! the DC voltage exceeded the limit configured in config.dc_bus_overvoltage_trip_level
        ERROR_CURRENT_MEASUREMENT_TIMEOUT = 0x00000008,
        ERROR_BRAKE_RESISTOR_DISARMED = 0x00000010, //<! the brake resistor was unexpectedly disarmed.
        ERROR_MOTOR_DISARMED = 0x00000020,          //<! the motor was unexpectedly disarmed.
        ERROR_MOTOR_FAILED = 0x00000040,            //<! check motor.error for more information
        ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x00000080,
        ERROR_ENCODER_FAILED = 0x00000100, //<! check encoder.error for more information
        ERROR_CONTROLLER_FAILED = 0x00000200,
        ERROR_POS_CTRL_DURING_SENSORLESS = 0x00000400, //<! DEPRECATED
        ERROR_WATCHDOG_TIMER_EXPIRED = 0x00000800,
        ERROR_MIN_ENDSTOP_PRESSED = 0x00001000,
        ERROR_MAX_ENDSTOP_PRESSED = 0x00002000,
        ERROR_ESTOP_REQUESTED = 0x00004000,
        ERROR_HOMING_WITHOUT_ENDSTOP = 0x00020000, //<! check encoder.error for more information
        ERROR_OVER_TEMP = 0x00040000               //<! check fet_thermistor.error and motor_thermistor.error for more information

    };

    enum Motor_Error_t
    {
        ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x00000001, //<! the measured motor phase resistance is outside of the plausible range
        ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x00000002, //<! the measured motor phase inductance is outside of the plausible range
        ERROR_ADC_FAILED = 0x00000004,
        ERROR_DRV_FAULT = 0x00000008, //<! the gate driver chip reported an error
        ERROR_CONTROL_DEADLINE_MISSED = 0x00000010,
        ERROR_NOT_IMPLEMENTED_MOTOR_TYPE = 0x00000020,
        ERROR_BRAKE_CURRENT_OUT_OF_RANGE = 0x00000040,
        ERROR_MODULATION_MAGNITUDE = 0x00000080,
        ERROR_BRAKE_DEADTIME_VIOLATION = 0x00000100,
        ERROR_UNEXPECTED_TIMER_CALLBACK = 0x00000200,
        ERROR_CURRENT_SENSE_SATURATION = 0x00000400, //<! the phase current was outside the measurable range
        ERROR_CURRENT_LIMIT_VIOLATION = 0x00001000,  //<! motor current magnitude exceeded the limit and margin
        ERROR_BRAKE_DUTY_CYCLE_NAN = 0x00002000,
        ERROR_DC_BUS_OVER_REGEN_CURRENT = 0x00004000, //<! too much current pushed into the power supply
        ERROR_DC_BUS_OVER_CURRENT = 0x00008000        //<! too much current pulled out of the power supply
    };

    enum Encoder_Error_t
    {
        ERROR_UNSTABLE_GAIN = 0x00000001,
        ERROR_CPR_POLEPAIRS_MISMATCH = 0x00000002,
        ERROR_NO_RESPONSE = 0x00000004,
        ERROR_UNSUPPORTED_ENCODER_MODE = 0x00000008,
        ERROR_ILLEGAL_HALL_STATE = 0x00000010,
        ERROR_INDEX_NOT_FOUND_YET = 0x00000020,
        ERROR_ABS_SPI_TIMEOUT = 0x00000040,
        ERROR_ABS_SPI_COM_FAIL = 0x00000080,
        ERROR_ABS_SPI_NOT_READY = 0x00000100
    };

    ODriveArduino(Stream &serial);

    // Commands
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);
    void SetCurrent(int motor_number, float current);
    void SetEncoderAbs(int motor_number, int32_t pos_abs);
    void SetLinearCount(int motor_number, int32_t count);

    void TrapezoidalMove(int motor_number, float position);
    void SetRequestedState(int motor_number, AxisState_t requested_state);
    void move_incremental(int motor_number, float displacement, bool from_input_pos);
    void start_anticogging_calibration(int motor_number);
    void watchdog_feed(int motor_number);
    void clear_errors(int motor_number);

    // Getters
    float GetVelocity(int motor_number);
    float GetPosition(int motor_number);
    float GetCurrent(int motor_number);
    float BusVoltage();
    AxisState_t GetCurrentState(int motor_number);
    AxisState_t GetRequestedState(int motor_number);

    float GetTemperature(int motor_number); //Temp of onboard thermistor

    bool GetMotorCalibrated(int motor_number);

    float GetPosSet(int motor_number);
    float GetVelSet(int motor_number);
    float GetCurrentSet(int motor_number);

    bool GetEncoderReady(int motor_number);

    int32_t GetEncoderAbs(int motor_number);

    int GetError(int motor_number);
    String GetErrorString(int motor_number);

    int GetMotorError(int motor_number);
    String GetMotorErrorString(int motor_number);
    int GetEncoderError(int motor_number);
    String GetEncoderErrorString(int motor_number);


    // General params
    float readFloat();
    int32_t readInt();

    // State helper
    bool run_state(int axis, AxisState_t requested_state, bool wait_for_idle, float timeout = 10.0f);

private:
    String readString();

    Stream &serial_;
};

#endif //ODriveArduino_h
