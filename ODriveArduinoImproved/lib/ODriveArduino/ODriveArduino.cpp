
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
    : serial_(serial) {}

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

void ODriveArduino::SetEncoderAbs(int motor_number, int32_t pos_abs);
void ODriveArduino::SetLinearCount(int32_t count);

void ODriveArduino::TrapezoidalMove(int motor_number, float position)
{
    serial_ << "t " << motor_number << " " << position << "\n";
}

void ODriveArduino::SetRequestedState(int motor_number, AxisState_t requested_state);
void ODriveArduino::move_incremental(float displacement, bool from_input_pos);
void ODriveArduino::start_anticogging_calibration();
void ODriveArduino::watchdog_feed();
void ODriveArduino::clear_errors();

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

ODriveArduino::AxisState_t ODriveArduino::GetCurrentState(int motor_number);
ODriveArduino::AxisState_t ODriveArduino::GetRequestedState(int motor_number);

float ODriveArduino::GetTemperature(int motor_number); //Temp of onboard thermistor

bool ODriveArduino::GetMotorCalibrated(int motor_number);

float ODriveArduino::GetPosSet(int motor_number);
float ODriveArduino::GetVelSet(int motor_number);
float ODriveArduino::GetCurrentSet(int motor_number);

bool ODriveArduino::GetEncoderReady(int motor_number);

int32_t ODriveArduino::GetEncoderAbs(int motor_number);

int ODriveArduino::GetErrors(int motor_number);
String ODriveArduino::GetErrorString(int motor_number);

bool ODriveArduino::run_state(int axis, int requested_state, bool wait_for_idle, float timeout)
{
    int timeout_ctr = (int)(timeout * 10.0f);
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle)
    {
        do
        {
            delay(100);
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
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
