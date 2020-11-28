// includes
#include <HardwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
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

HardwareSerial &odrive_serial = Serial1;

ODriveArduino odrive(odrive_serial);

void setup()
{
  // ODrive uses 115200 baud
  odrive_serial.begin(250000);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial)
    ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  odrive_serial << "w axis0.controller.config.vel_limit " << 50.0f << '\n';

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
}

void loop()
{

  if (Serial.available())
  {
    char c = Serial.read();

    // Run calibration sequence
    if (c == '0')
    {
      int motornum = c - '0';
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if (!odrive.run_state(motornum, requested_state, true))
        return;

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if (!odrive.run_state(motornum, requested_state, true, 25.0f))
        return;

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if (!odrive.run_state(motornum, requested_state, false /*don't wait*/))
        return;
    }

    // Sinusoidal test move
    if (c == 's')
    {
      Serial.println("Executing test move");
      odrive.SetPosition(0, 50);
    }

    // Read bus voltage
    if (c == 'b')
    {
      odrive_serial << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p')
    {
      odrive_serial << "sr \n";
    }
  }
}
