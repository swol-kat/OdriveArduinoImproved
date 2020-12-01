#include "Motor.h"

Motor::Motor(int motor, ODriveArduino *odrive)
{
    axis = motor;
    odrv = odrive;
}

void Motor::setPosition(float position)
{
    setPosition(position, 0.0f, 0.0f);
}
void Motor::setPosition(float position, float vel_feed)
{
    setPosition(position, vel_feed, 0.0f);
}
void Motor::setPosition(float position, float vel_feed, float current_feed)
{
    odrv->SetPosition(axis, position, vel_feed, current_feed);
}

void Motor::setVelocity(float vel)
{
    setVelocity(vel, 0.0f);
}
void Motor::setVelocity(float vel, float current_feed)
{
    odrv->SetVelocity(axis, vel, current_feed);
}

void Motor::setCurrent(float current)
{
    odrv->SetCurrent(axis, current);
}