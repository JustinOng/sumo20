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
      LEFT = 0,
      RIGHT = 1
    };

    Drive(Stream* serial);
    void setVel(Motor_t motor, int8_t velocity);
    void incPosition(Motor_t motor, int32_t change);
    bool moveDone(Motor_t motor);
    void requestFeedback(void);
    void requestCtrlMode(void);

    int32_t getPos(Motor_t motor);
    uint8_t getCtrlMode(Motor_t motor);

    void loop(void);
  private:
    // used to track what data we're expecting to receive on the serial port
    enum Read_State_t {
      NONE,
      READING_FEEDBACK,
      READING_CTRL_MODE
    };

    Read_State_t _read_state = NONE;
    // extra contextual data for current read state
    uint8_t _read_state_data;

    // buffer to hold serial data
    char _read_state_buffer[64];
    uint8_t _read_state_len = 0;

    float _pos[2];
    float _vel[2];

    uint8_t _ctrl_mode[2] = {5, 5};

    int32_t target_left;
    int32_t target_right;

    Stream* _serial;
    
    void _requestFeedback(Motor_t motor);
    void _requestCtrlMode(Motor_t motor);
};

#endif
