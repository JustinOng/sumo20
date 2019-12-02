#include <Arduino.h>
#include "drive.h"

// #define LOG_CONTROLLER_VALUES

#define PIN_RC_CH0 23
#define PIN_RC_CH1 22
#define PIN_RC_CH2 21

// scale left and right drive powers
#define LEFT_SCALE 1
#define RIGHT_SCALE 1

// ms at which odrive should be updated
#define DRIVE_UPDATE_THROTTLE 50

// measured from actual output
const uint16_t MIN_PULSE_LEN[3] = {990, 1010};
const uint16_t MAX_PULSE_LEN[3] = {2010, 1992};

Drive drive(&Serial3);

volatile uint16_t pulse_len[3] = {1500};

template <uint8_t pin, uint8_t channel>
void ch_service() {
  static elapsedMicros ele_time;
  if(digitalReadFast(pin)) {
    ele_time = 0;
  } else {
    pulse_len[channel] = (uint16_t) ele_time;
  }
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  attachInterrupt(23, ch_service<23, 0>, CHANGE);
  attachInterrupt(22, ch_service<22, 1>, CHANGE);
  attachInterrupt(PIN_RC_CH0, ch_service<PIN_RC_CH0, 0>, CHANGE);
  attachInterrupt(PIN_RC_CH1, ch_service<PIN_RC_CH1, 1>, CHANGE);
  attachInterrupt(PIN_RC_CH2, ch_service<PIN_RC_CH2, 2>, CHANGE);

  pinMode(13, OUTPUT);
}

void loop() {
  static elapsedMillis driveDelay;

  if (driveDelay > DRIVE_UPDATE_THROTTLE) {
    int8_t forward = (int8_t) map(pulse_len[1], MIN_PULSE_LEN[1], MAX_PULSE_LEN[1], -100, 100);
    int8_t turn = (int8_t) map(pulse_len[0], MIN_PULSE_LEN[0], MAX_PULSE_LEN[0], -100, 100);

    // deadzone
    if (abs(forward) < 5) forward = 0;
    if (abs(turn) < 5) turn = 0;

    drive.setSpeed(Drive::LEFT, (forward + turn) * LEFT_SCALE);
    drive.setSpeed(Drive::RIGHT, (forward - turn) * RIGHT_SCALE);

    driveDelay = 0;

#ifdef LOG_CONTROLLER_VALUES
    Serial.print(pulse_len[0]);
    Serial.print(", ");
    Serial.println(pulse_len[1]);
#endif
  }
}
