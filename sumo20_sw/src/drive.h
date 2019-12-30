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
    void setPosition(Motor_t motor, int32_t target);
    void requestFeedback(void);

    int32_t getPos(Motor_t motor);

    void loop(void);
  private:
    // used to track what data we're expecting to receive on the serial port
    enum Read_State_t {
      NONE,
      READING_FEEDBACK
    };

    Read_State_t _read_state = NONE;
    // extra contextual data for current read state
    uint8_t _read_state_data;

    // buffer to hold serial data
    char _read_state_buffer[64];
    uint8_t _read_state_len = 0;

    float _pos[2];
    float _vel[2];

    Stream* _serial;
    
    void _requestFeedback(Motor_t motor);
};

#endif
