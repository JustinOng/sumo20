#include <Arduino.h>
#include "drive.h"

Drive::Drive(Stream* serial) {
  _serial = serial;
}

void Drive::setSpeed(Motor_t motor, int8_t velocity) {
  // velocity: -100 to 100 for full reverse to full forward respectively
  _serial->print("v ");
  _serial->print((uint8_t) motor);
  _serial->print(" ");
  _serial->println(map(velocity, -100, 100, -MAX_SPEED, MAX_SPEED));
}
