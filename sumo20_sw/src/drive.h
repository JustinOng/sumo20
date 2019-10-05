#ifndef DRIVE_H
#define DRIVE_H

// max speed in encoder count per second
#define MAX_SPEED 400000

class Drive {
  public:
    // axis on the odrive
    enum Motor_t {
      LEFT = 1,
      RIGHT = 0
    };

    Drive(Stream& serial);
    setSpeed(Motor_t motor, int8_t velocity);
  private:
    Stream& _serial;
}

#endif
