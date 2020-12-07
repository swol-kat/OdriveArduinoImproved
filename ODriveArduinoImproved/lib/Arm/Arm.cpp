#include "Arm.h"

Arm::Arm(Motor *m_shoulder, Motor *m_upper, Motor *m_lower){
    motors[SHOULDER] = m_shoulder;
}

void Arm::setPosition(motors_t target, float position){
    motors[target]->setPosition(position);
}
void Arm::setPosition(motors_t target, float position, float vel_feed){
    motors[target]->setPosition(position, vel_feed);
}
void  Arm::setPosition(motors_t target, float position, float vel_feed, float current_feed){
    motors[target]->setPosition(position, vel_feed, current_feed);
}

void  Arm::setVelocity(motors_t target, float vel){
    motors[target]->setVelocity(vel);
}
void  Arm::setVelocity(motors_t target, float vel, float current_feed){
    motors[target]->setVelocity(vel, current_feed);
}
    
void  Arm::setCurrent(motors_t target, float current){
    motors[target]->setCurrent(current);
}