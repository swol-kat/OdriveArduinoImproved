#include "ODriveArduino.h"

class Motor
{
private:
    int axis;
    ODriveArduino *odrv; 
public:
    Motor(int motor, ODriveArduino *odrive);

    void setPosition(float position);
    void setPosition(float position, float vel_feed);
    void setPosition(float position, float vel_feed, float current_feed);

    void setVelocity(float vel);
    void setVelocity(float vel, float current_feed);
    
    void setCurrent(float current);
};
