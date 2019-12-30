#include "robot.h"

Robot::Robot(void) {
  _drive = new Drive(&Serial3);
  _vacuum = new Servo();
  _lifter = new Servo();
  _distance_sensors = new Distance_Sensors();

  _vacuum->attach(PIN_VACUUM);
  _lifter->attach(PIN_LIFTER);

  _display = new Adafruit_SSD1306(128, 64, &Wire, -1);
}

void Robot::begin(void) {
  Serial3.begin(115200);
  LEDS.addLeds<WS2812B, PIN_LED_INT, GRB>(leds_int, NUM_LED_INT);

  // initialises 19/18 as SCL/SDA
  Wire.begin();
  Wire.setTimeout(1);

  if(!_display->begin(SSD1306_SWITCHCAPVCC, 0x3c)) {
    Serial1.println(F("SSD1306 allocation failed"));
  }

  _display->setTextSize(1);
  _display->setTextColor(SSD1306_WHITE);
}

void Robot::setSpeed(int8_t forward, int8_t turn) {
  // throttle updates to max every UPDATE_THROTTLE ms
  static elapsedMillis lastUpdated;

  if (lastUpdated < UPDATE_THROTTLE) return;
  lastUpdated = 0;

  // deadzone
  if (abs(forward) < 3) forward = 0;
  if (abs(turn) < 3) turn = 0;

  _power_left = (forward + turn * TURN_SCALE) * LEFT_SCALE;
  _power_right = (forward - turn * TURN_SCALE) * RIGHT_SCALE;
}

void Robot::setVacuum(uint8_t power) {
  // power: 0 - 100
  _vacuum->writeMicroseconds(map(power, 0, 100, 1000, 2000));
}

void Robot::setLifter(uint16_t pulsewidth) {
  _lifter->writeMicroseconds(pulsewidth);
}

void Robot::displayCurrent(void) {
  static elapsedMillis last_update;

  if (last_update < 100) return;
  last_update = 0;

  // the ACS712 outputs a value from 0-5V which goes through a voltage divider
  // reducing it to 0-3.3V
  float raw_voltage = analogRead(PIN_ISEN) / 1024.0 * 3.3;
  float corrected_voltage = raw_voltage * 3 / 2;

  // value is centered around 2.5V, every ampere is 0.1V
  // multiply by 100 so we work with ints instead
  int8_t current = (corrected_voltage - 2.5) * 100;

  _display->clearDisplay();
  _display->setCursor(0, 0);
  _display->print("Current: ");
  _display->print(current / 10);
  _display->print('.');
  _display->print(abs(current % 10));

  _display->display();
}

void Robot::setMode(Modes_t mode) {
  _mode = mode;
}

void Robot::loop(void) {
  _distance_sensors->loop();

  for(byte i = 0; i < NUM_VL53L0X; i++) {
    // calculate correct index in the ws2812b
    byte led_index = (3-i) + 1;
    
    if (!_distance_sensors->initialised[i]) {
      leds_int[led_index] = CRGB::Black;
      continue;
    }

    if (_distance_sensors->distance[i] > 1000) {
      leds_int[led_index] = CRGB::Red;
      continue;
    }

    leds_int[led_index] = map(_distance_sensors->distance[i], 0, 1000, 0, 255) << 8;
  }

  for (byte i = 0; i < NUM_IR; i++) {
    ir[i] = analogRead(ir_pins[i]) < IR_THRESHOLD;
  }

  if (_mode == MODE_RC) {
    _drive->setSpeed(Drive::LEFT, _power_left);
    _drive->setSpeed(Drive::RIGHT, _power_right);
    leds_int[0] = CRGB::Green;
  } else {
    leds_int[0] = CRGB::Blue;
  }

  FastLED.show();

  displayCurrent();
}
