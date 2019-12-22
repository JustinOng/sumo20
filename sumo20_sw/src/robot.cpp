#include "robot.h"

Robot::Robot(void) {
  _drive = new Drive(&Serial3);
  _vacuum = new Servo();
  _distance_sensors = new Distance_Sensors();

  _vacuum->attach(PIN_VACUUM);
}

void Robot::begin(void) {
  Serial3.begin(115200);
  LEDS.addLeds<WS2812B, PIN_LED_INT, GRB>(leds_int, NUM_LED_INT);

  // initialises 19/18 as SCL/SDA
  Wire.begin();
  Wire.setTimeout(1);
}

void Robot::setSpeed(int8_t forward, int8_t turn) {
  // throttle updates to max every UPDATE_THROTTLE ms
  static elapsedMillis lastUpdated;

  if (lastUpdated < UPDATE_THROTTLE) return;
  lastUpdated = 0;

  // deadzone
  if (abs(forward) < 5) forward = 0;
  if (abs(turn) < 5) turn = 0;

  _drive->setSpeed(Drive::LEFT, (forward + turn * TURN_SCALE) * LEFT_SCALE);
  _drive->setSpeed(Drive::RIGHT, (forward - turn * TURN_SCALE) * RIGHT_SCALE);
}

void Robot::setVacuum(uint8_t power) {
  // power: 0 - 100
  _vacuum->writeMicroseconds(map(power, 0, 100, 1000, 2000));
}

void Robot::loop(void) {
  _distance_sensors->loop();

  for (byte i = 0; i < NUM_IR; i++) {
    ir[i] = analogRead(ir_pins[i]) < IR_THRESHOLD;
  }
}
