#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>

// max speed in encoder count per second
// scaling factor * max voltage * kV * encoder count per rev / seconds in a minute
// 0.90 * 12.6 * 1400 * 4000 / 60 = 1058400
#define MAX_SPEED 900000

class Drive {
  public:
    // axis on the odrive
    enum Motor_t {
      RIGHT = 0,
      LEFT = 1
    };

    Drive(Stream* serial);
    void setSpeed(Motor_t motor, int8_t velocity);
  private:
    Stream* _serial;
};

#endif
