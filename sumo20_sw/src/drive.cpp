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

void Drive::incPosition(int32_t change_left, int32_t change_right) {
  target_left = _pos[LEFT] + change_left;
  target_right = _pos[RIGHT] + change_right;

  _setPosition(LEFT, target_left);
  _setPosition(RIGHT, target_right);
}

bool Drive::moveDone(void) {
  return (abs(target_left - _pos[LEFT]) < POS_MOVE_THRESHOLD) &&
    (abs(target_right - _pos[RIGHT]) < POS_MOVE_THRESHOLD);
}

void Drive::_setPosition(Motor_t motor, int32_t target) {
  _serial->print("t ");
  _serial->print((uint8_t) motor);
  _serial->print(" ");
  _serial->println(target);
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

void Drive::loop(void) {
  static elapsedMillis last_message;
  char data;

  while (_serial->available()) {
    last_message = 0;
    data = _serial->read();

    if (_read_state == READING_FEEDBACK) {
      _read_state_buffer[_read_state_len++] = data;
      if (data == '\n') {
        sscanf(_read_state_buffer, "%f %f", &_pos[_read_state_data], &_vel[_read_state_data]);
        _read_state_len = 0;

        _read_state = NONE;

        if (_read_state_data == LEFT) {
          _requestFeedback(RIGHT);
        }
      }
    } else {
      Serial1.print("Received data while in NONE: ");
      Serial1.println(data);
    }
  };

  if (last_message > 1000 && _read_state != NONE) {
    Serial1.println("Read timed out!");
    _read_state = NONE;
  }
}
