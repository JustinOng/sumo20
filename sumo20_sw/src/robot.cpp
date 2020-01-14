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
  if (_mode == mode) return;
  
  _mode = mode;

  if (_mode == MODE_AUTON) {
    _auton_state = NONE;
    Serial1.println("Entering auton");
  }
}

void Robot::loop(void) {
  static elapsedMillis last_feedback;

  _drive->loop();

  _distance_sensors->loop();

  for(byte i = 0; i < NUM_VL53L0X; i++) {
    // calculate correct index in the ws2812b
    byte led_index = (3-i) + 1;
    
    if (!_distance_sensors->initialised[i]) {
      leds_int[led_index] = CRGB::Black;
      continue;
    }

    if (_distance_sensors->distance[i] > PROX_THRESHOLD) {
      leds_int[led_index] = CRGB::Red;
      continue;
    }

    leds_int[led_index] = map(_distance_sensors->distance[i], PROX_THRESHOLD, 0, 0, 255) << 8;
  }

  for (byte i = 0; i < NUM_IR; i++) {
    ir[i] = analogRead(ir_pins[i]) < IR_THRESHOLD;
  }

  if (_mode == MODE_RC) {
    _drive->setSpeed(Drive::LEFT, _power_left);
    _drive->setSpeed(Drive::RIGHT, _power_right);
    leds_int[0] = CRGB::Green;
  } else {
    updateAutonState();
    leds_int[0] = CRGB::Blue;
  }

  FastLED.show();

  displayCurrent();

  if (last_feedback > 10) {
    last_feedback = 0;
    _drive->requestCtrlMode();
  }
}

void Robot::updateAutonState(void) {
  Auton_State_t new_state = INVALID;

  uint16_t *distance = _distance_sensors->distance;
  int16_t vel_left = 0, vel_right = 0, error = 0;

  switch(_auton_state) {
    case NONE:
      new_state = SEEK;
      Serial1.println("Entered NONE");
      break;
    case TRACK:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered TRACK");
      }
      /*
        sensors are arranged
          scoop
        0  1  2  3
           body
        physically from left to right
      */
      if (
        distance[0] > PROX_THRESHOLD &&
        distance[1] > PROX_THRESHOLD &&
        distance[2] > PROX_THRESHOLD &&
        distance[3] > PROX_THRESHOLD
      ) {
        // if none of the sensors see anything theres nothing to track, move to SEEK
        new_state = SEEK;
      }

      vel_left = TRACK_BASE_SPEED;
      vel_right = TRACK_BASE_SPEED;

      if (distance[1] > PROX_THRESHOLD && distance[2] < PROX_THRESHOLD) {
        // left sensor does not see, right sensor sees
        // spin left
        vel_right += TRACK_TURN_SPEED;
      } else if (distance[1] < PROX_THRESHOLD && distance[2] > PROX_THRESHOLD) {
        // left sensor sees, right sensor does not see
        // spin right
        vel_left += TRACK_TURN_SPEED;
      }

      // add the difference between s1 and s2 to power
      if (distance[1] < PROX_THRESHOLD && distance[2] < PROX_THRESHOLD) {
        error = distance[1] - distance[2];

        error = max(-TRACK_ERROR_MAX, min(TRACK_ERROR_MAX, error));

        vel_left -= error * TRACK_ERROR_KP;
        vel_right += error * TRACK_ERROR_KP;;
      }

      vel_left = max(0, min(vel_left, 100));
      vel_right = max(0, min(vel_right, 100));

      /*
      Serial1.print("Error: ");
      Serial1.print(error);
      Serial1.print(" Left: ");
      Serial1.print(vel_left);
      Serial1.print(" Right: ");
      Serial1.println(vel_right);
      */

      _drive->setSpeed(Drive::LEFT, -vel_left);
      _drive->setSpeed(Drive::RIGHT, vel_right);
      break;
    case SEEK:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered SEEK");
      }

      if (
        distance[0] < PROX_THRESHOLD ||
        distance[1] < PROX_THRESHOLD ||
        distance[2] < PROX_THRESHOLD ||
        distance[3] < PROX_THRESHOLD
      ) {
        new_state = TRACK;
      }

      _drive->setSpeed(Drive::LEFT, SEEK_SPEED);
      _drive->setSpeed(Drive::RIGHT, SEEK_SPEED);
      break;
    case START_ST1_R_REV:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered ST1_TURN");

        _drive->incPosition(_drive->LEFT, 50 * 4000);
        _drive->incPosition(_drive->RIGHT, -3 * 4000);
      }

      if (_drive->moveDone(_drive->RIGHT)) {
        new_state = START_ST2_FW;
      }
      break;
    case START_ST2_FW:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered ST2_FW");
        
        _drive->incPosition(_drive->RIGHT, 100 * 4000);
      }

      if (_drive->moveDone(_drive->LEFT) && _drive->moveDone(_drive->RIGHT)) {
        new_state = DONE;
      }
      break;
    case DONE:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered done");
      }
      break;
    case INVALID:
      new_state = NONE;
      break;
  }

  _pAuton_state = _auton_state;

  if (new_state != INVALID) {
    _auton_state = new_state;
  }
}
