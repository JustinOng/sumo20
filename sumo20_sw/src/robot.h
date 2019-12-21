#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Servo.h>

#include <FastLED.h>

#include "drive.h"

// scale left and right drive powers
#define LEFT_SCALE 1
#define RIGHT_SCALE -1

// scale turn power
#define TURN_SCALE -1

#define PIN_VACUUM 2

// throttle updates to the motor driver/vacuum to UPDATE_THROTTLE ms
#define UPDATE_THROTTLE 50

// ADC value below which to treat as line seen (white)
#define IR_THRESHOLD 300
#define NUM_IR 4

const uint8_t ir_pins[NUM_IR] = {23, 15, 14, 20};

enum {
  IR_FRONT_LEFT,
  IR_FRONT_RIGHT,
  IR_REAR_RIGHT,
  IR_REAR_LEFT
};

// number of internal LEDs
#define NUM_LED_INT 5
#define PIN_LED_INT 10

class Robot {
  public:
    Robot(void);
    void begin(void);
    void setSpeed(int8_t forward, int8_t turn);
    void setVacuum(uint8_t power);

    void loop(void);
  private:
    Drive *_drive;
    Servo *_vacuum;

    // true if line is seen
    bool ir[NUM_IR];
    CRGB leds_int[NUM_LED_INT];
};

#endif
