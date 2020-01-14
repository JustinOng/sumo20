#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Servo.h>

#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "drive.h"
#include "distance_sensors.h"

// scale left and right drive powers
#define LEFT_SCALE -1
#define RIGHT_SCALE 1

// scale turn power
#define TURN_SCALE 1

#define PIN_VACUUM 4
#define PIN_LIFTER 5

// throttle updates to the motor driver/vacuum to UPDATE_THROTTLE ms
#define UPDATE_THROTTLE 50

// ADC value below which to treat as line seen (white)
#define IR_THRESHOLD 300
#define NUM_IR 4

// ignore detections above this value
#define PROX_THRESHOLD 500

// configuration for autonomous states

// base speed, TRACK_TURN_SPEED is used to adjust
// so sensors 1 and 3 see the robot
#define TRACK_BASE_SPEED 10
// how fast to turn, 0 - 100
#define TRACK_TURN_SPEED 10

#define TRACK_ERROR_MAX 100
#define TRACK_ERROR_KP 0.2

// how fast to turn, 0 - 100
#define SEEK_SPEED 20

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

#define PIN_ISEN A3

class Robot {
  public:
    enum Modes_t {
      MODE_RC,
      MODE_AUTON
    };

    Robot(void);
    void begin(void);
    void setSpeed(int8_t forward, int8_t turn);
    void setVacuum(uint8_t power);
    void setLifter(uint16_t pulsewidth);

    void setMode(Modes_t mode);

    void loop(void);
  private:
    enum Auton_State_t {
      NONE,
      TRACK,
      SEEK,
      START_ST1_R_REV, // turn away from starting position
      START_ST2_FW,   // move forward
      DONE,
      INVALID
    };

    Drive *_drive;
    Servo *_vacuum;
    Servo *_lifter;
    Distance_Sensors *_distance_sensors;

    // true if line is seen
    bool ir[NUM_IR] = {0};
    CRGB leds_int[NUM_LED_INT];

    Adafruit_SSD1306 *_display;
    
    Modes_t _mode = MODE_RC;

    uint8_t _power_left = 0;
    uint8_t _power_right = 0;

    Auton_State_t _auton_state, _pAuton_state = NONE;

    void displayCurrent(void);
    void updateAutonState(void);
};

#endif
