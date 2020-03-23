#include <Arduino.h>
#include "drive.h"

Drive::Drive(Stream* serial) {
  _serial = serial;
}

void Drive::setVel(Motor_t motor, int8_t velocity) {
  // velocity: -100 to 100 for full reverse to full forward respectively
  _serial->print("v ");
  _serial->print((uint8_t) motor);
  _serial->print(" ");
  _serial->println(map(velocity, -100, 100, -MAX_SPEED, MAX_SPEED));

  _set_vel[motor] = velocity;
}

uint8_t Drive::getVel(Motor_t motor) {
  return _set_vel[motor];
}

bool Drive::moveDone(Motor_t motor) {
  return _ctrl_mode[motor] == 3;
}

void Drive::incPosition(Motor_t motor, int32_t change) {
  if (motor == LEFT) {
    change *= -1;
  }

  _serial->print("it ");
  _serial->print((uint8_t) motor);
  _serial->print(" ");
  _serial->println(change);

  // assume that control mode of the axis would have changed to 4
  // pre-emptively set it to 4 so autonomous does not see
  // the old value as move complete
  _ctrl_mode[motor] = 4;
}

void Drive::requestCtrlMode(void) {
  _requestCtrlMode(LEFT);
}

void Drive::_requestCtrlMode(Motor_t motor) {
  if (_read_state != NONE) return;

  _serial->print("r axis");
  _serial->print(motor);
  _serial->println(".controller.config.control_mode");

  _read_state = READING_CTRL_MODE;
  _read_state_data = (uint8_t) motor;
}

void Drive::requestFeedback(void) {
  _requestFeedback(LEFT);
}

void Drive::_requestFeedback(Motor_t motor) {
  if (_read_state != NONE) return;

  _serial->print("f ");
  _serial->println((uint8_t) motor);

  _read_state = READING_FEEDBACK;
  _read_state_data = (uint8_t) motor;
}

int32_t Drive::getPos(Motor_t motor) {
  return _pos[motor];
}

uint8_t Drive::getCtrlMode(Motor_t motor) {
  return _ctrl_mode[motor];
}

void Drive::loop(void) {
  static elapsedMillis last_message;
  char data;

  uint8_t tmp;

  while (_serial->available()) {
    last_message = 0;
    data = _serial->read();

    _read_state_buffer[_read_state_len++] = data;
    if (data == '\n') {
      if (_read_state == READING_FEEDBACK) {
        sscanf(_read_state_buffer, "%f %f", &_pos[_read_state_data], &_vel[_read_state_data]);
      
        _read_state_len = 0;
        _read_state = NONE;

        if (_read_state_data == LEFT) {
          _requestFeedback(RIGHT);
        }
      } else if (_read_state == READING_CTRL_MODE) {
        sscanf(_read_state_buffer, "%hhu", &tmp);
        _ctrl_mode[_read_state_data] = tmp;
      
        _read_state_len = 0;
        _read_state = NONE;

        if (_read_state_data == LEFT) {
          _requestCtrlMode(RIGHT);
        }
      } else {
        Serial1.print("Received data while in NONE: ");
        Serial1.write(_read_state_buffer, _read_state_len);
      
        _read_state_len = 0;
        _read_state = NONE;
      }
    }
  };

  if (last_message > 1000 && _read_state != NONE) {
    Serial1.println("Read timed out!");
    last_message = 0;
    _read_state = NONE;
  }
}
