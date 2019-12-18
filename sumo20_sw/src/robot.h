#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Servo.h>

#include "drive.h"

// scale left and right drive powers
#define LEFT_SCALE 1
#define RIGHT_SCALE -1

// scale turn power
#define TURN_SCALE -1

#define PIN_VACUUM 2

// throttle updates to the motor driver/vacuum to UPDATE_THROTTLE ms
#define UPDATE_THROTTLE 50

class Robot {
  public:
    Robot(void);
    void begin(void);
    void setSpeed(int8_t forward, int8_t turn);
    void setVacuum(uint8_t power);
  private:
    Drive *_drive;
    Servo *_vacuum;
};

#endif
