#include "Motor.h"

class Arm
{
private:
    Motor *motors[3];
public:

    enum motors_t{
        SHOULDER = 0,
        UPPER = 1,
        LOWER = 2
    };

    Arm(Motor *m_shoulder, Motor *m_upper, Motor *m_lower);

    void setPosition(motors_t target, float position);
    void setPosition(motors_t target, float position, float vel_feed);
    void setPosition(motors_t target, float position, float vel_feed, float current_feed);

    void setVelocity(motors_t target, float vel);
    void setVelocity(motors_t target, float vel, float current_feed);
    
    void setCurrent(motors_t target, float current);
};
