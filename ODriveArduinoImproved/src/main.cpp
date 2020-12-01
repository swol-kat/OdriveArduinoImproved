// includes
#include <HardwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default Teensy one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive GND to Arduino GND.

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
// HardwareSerial& odrive_serial = Serial1;

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
// SoftwareSerial odrive_serial(8, 9);


// ODrive object
ODriveArduino odrive(odrive_serial);

void setup() {
  // ODrive uses 115200 baudS
  odrive_serial.begin(250000);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino - Swol Cat Edition");

for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 50.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 110.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }


  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
  Serial.println("Send the character 'e' to print errors");
  Serial.println("Send the character 'q' to clear errors");
  Serial.println("Send the character 'r' to reboot");
  Serial.println("Send the character 'v' for velocity control at 15 rps");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();

    // Run calibration sequence
    if (c == '0' || c == '1') {
      int motornum = c-'0';
      ODriveArduino::AxisState_t requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true)) return;

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if(!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
    }

    // Sinusoidal test move
    if (c == 's') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 62.8318530718f; ph += 0.1f) {
        float pos_m0 = 2.0f * cos(ph);
        float pos_m1 = 2.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        odrive.SetPosition(1, pos_m1);
        delay(5);
      }
    }

    // Read bus voltage
    if (c == 'b') {
      odrive_serial << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p') {
      unsigned int count = 0;
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
          count++;
          odrive_serial << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial << odrive.readFloat() << "\t" << count << '\t';
        }
        Serial << '\n';
      }
    }

    if (c == 'e'){
      Serial << odrive.GetError(0) << '\t' << odrive.GetErrorString(0) << '\n' << odrive.GetError(1) << '\t' << odrive.GetErrorString(1) << '\n';
    }

    if (c == 'q'){
      odrive.clear_errors(0);
      odrive.clear_errors(1);
      Serial << "Errors Cleared\n";
    }

    if (c == 'r'){
      odrive.reboot();
      Serial << "rebooting\n";
    }

    if (c == 'v'){
      odrive.SetVelocity(0, 15.0f);
      odrive.SetVelocity(1, 15.0f);
    }
  }
}
