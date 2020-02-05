#include "robot.h"

Robot::Robot(void) {
  _drive = new Drive(&Serial3);
  _vacuum = new Servo();
  _lifter = new Servo();
  _distance_sensors = new Distance_Sensors();

  _vacuum->attach(PIN_VACUUM);
  _lifter->attach(PIN_LIFTER);

  setVacuum(0);
  setLifter(1000);

  _display = new Adafruit_SSD1306(128, 64, &Wire, -1);
}

void Robot::begin(void) {
  Serial3.begin(921600);
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

void Robot::startVacuumOnMove(bool start) {
  _vacuum_on_move = start;
}

void Robot::setVacuum(uint8_t power) {
  // power: 0 - 100
  setVacuumRaw(map(power, 0, 100, 1000, 2000));
}

void Robot::setVacuumRaw(uint16_t power) {
  _vacuum->writeMicroseconds(power);
}

void Robot::setLifter(uint16_t pulsewidth) {
  if (pulsewidth < 1000 || pulsewidth > 2000) return;

  pulsewidth = max(pulsewidth, 1250);

  _lifter->writeMicroseconds(map(pulsewidth, 1000, 2000, 2000, 1000));
}

void Robot::updateDisplay(void) {
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
  _display->print(ir[0]);
  _display->print(" ");
  _display->print(ir_raw[0]);

  _display->setCursor(120, 0);
  _display->print(ir[1]);
  _display->setCursor(90, 0);
  _display->print(" ");
  _display->print(ir_raw[1]);

  _display->setCursor(120, 56);
  _display->print(ir[2]);
  _display->setCursor(90, 56);
  _display->print(" ");
  _display->print(ir_raw[2]);

  _display->setCursor(0, 56);
  _display->print(ir[3]);
  _display->print(" ");
  _display->print(ir_raw[3]);

  _display->setCursor(0, 15);
  _display->print(_distance_sensors->distance[3]);
  _display->setCursor(32, 15);
  _display->print(_distance_sensors->distance[2]);
  _display->setCursor(64, 15);
  _display->print(_distance_sensors->distance[1]);
  _display->setCursor(96, 15);
  _display->print(_distance_sensors->distance[0]);

  _display->setCursor(0, 30);
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

    dist_last_seen[i] = millis();

    leds_int[led_index] = map(_distance_sensors->distance[i], PROX_THRESHOLD, 0, 0, 255) << 8;
  }

  for (byte i = 0; i < NUM_IR; i++) {
    ir_raw[i] = IR_WEIGHT * analogRead(ir_pins[i]) + (1 - IR_WEIGHT) * ir_raw[i];
    ir[i] = ir_raw[i] IR_THRESHOLD;

    // explicitly ignore the rear sensors because they have been removed
    if (i == 2 || i == 3) {
      ir[i] = 0;
    }

    if (ir[i]) ir_last_seen[i] = 0;
  }

  if (_mode == MODE_RC) {
    /*if (
      (ir[IR_FRONT_LEFT] && _power_left < 0) ||  // if forward yet front sensor triggered
      (ir[IR_REAR_LEFT] && _power_left > 0)      // if reverse yet rear sensor triggered
    ) {
      _drive->setVel(Drive::LEFT, 0);
    } else {
      _drive->setVel(Drive::LEFT, _power_left);
    }

    if (
      (ir[IR_FRONT_RIGHT] && _power_right > 0) || // if forward yet front sensor triggered
      (ir[IR_REAR_RIGHT] && _power_right < 0)     // if reverse yet rear sensor triggered
    ) {
      _drive->setVel(Drive::RIGHT, 0);
    } else {
      _drive->setVel(Drive::RIGHT, _power_right);
    }*/

    _drive->setVel(Drive::LEFT, _power_left);
    _drive->setVel(Drive::RIGHT, _power_right);

    if (_vacuum_on_move && (millis() % 500 < 250)) {
      leds_int[0] = CRGB::Black;
    } else {
      leds_int[0] = CRGB::Green;
    }
  } else {
    updateAutonState();
    leds_int[0] = CRGB::Blue;
  }

  if (
    (_mode == MODE_RC && (_drive->getVel(Drive::LEFT) != 0 || _drive->getVel(Drive::RIGHT) != 0)) ||
    _mode == MODE_AUTON
  ) {
    if (_vacuum_on_move) {
      Serial1.println("Starting vacuum");
      _vacuum_on_move = false;
      setVacuum(VACUUM_ON);
    }
  }

  FastLED.show();

  updateDisplay();

  if (last_feedback > 10) {
    last_feedback = 0;
    _drive->requestCtrlMode();
  }
}

void Robot::updateAutonState(void) {
  static elapsedMillis last_state_change;

  Auton_State_t new_state = INVALID;

  uint16_t *distance = _distance_sensors->distance;
  int32_t vel_left = 0, vel_right = 0, error = 0;

  switch(_auton_state) {
    case NONE:
      new_state = START_ST1_R_REV;
      Serial1.println("Entered NONE");
      break;
    case TRACK:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered TRACK");
      }
      /*
        sensors are arranged
          scoop
        3  2  1  0
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
        new_state = SEEK_SPIN;
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

      _drive->setVel(Drive::LEFT, -vel_left);
      _drive->setVel(Drive::RIGHT, vel_right);
      break;
    case SEEK_SPIN:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered SEEK_SPIN");

        if (dist_last_seen[0] < dist_last_seen[3]) {
          // target was last seen on the right so spin right
          _drive->setVel(Drive::LEFT, -SEEK_SPIN_SPEED);
          _drive->setVel(Drive::RIGHT, -SEEK_SPIN_SPEED);
        } else {
          _drive->setVel(Drive::LEFT, SEEK_SPIN_SPEED);
          _drive->setVel(Drive::RIGHT, SEEK_SPIN_SPEED);
        }
      }

      if (
        distance[0] < PROX_THRESHOLD ||
        distance[1] < PROX_THRESHOLD ||
        distance[2] < PROX_THRESHOLD ||
        distance[3] < PROX_THRESHOLD
      ) {
        new_state = TRACK;
      }

      if (last_state_change > SEEK_SPIN_DURATION) {
        new_state = SEEK_FORWARD;
      }
      break;
    case SEEK_FORWARD:
      if (_auton_state != _pAuton_state) {
        _drive->setVel(Drive::LEFT, -SEEK_FORWARD_SPEED);
        _drive->setVel(Drive::RIGHT, SEEK_FORWARD_SPEED);
      }

      if (
        distance[0] < PROX_THRESHOLD ||
        distance[1] < PROX_THRESHOLD ||
        distance[2] < PROX_THRESHOLD ||
        distance[3] < PROX_THRESHOLD
      ) {
        new_state = TRACK;
      }

      break;
    case FLEE_LINE_REV:
      if (_auton_state != _pAuton_state) {
        _drive->incPosition(_drive->LEFT, FLEE_LINE_REV_POS);
        _drive->incPosition(_drive->RIGHT, FLEE_LINE_REV_POS);
      }
      
      // use OR because this does not need to be precise
      // and can assume both axis would have finished at same time
      if (_drive->moveDone(_drive->LEFT) || _drive->moveDone(_drive->RIGHT)) {
        new_state = FLEE_LINE_TURN;
      }
      break;
    case FLEE_LINE_TURN:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered FLEE_LINE");

        vel_right = random(FLEE_LINE_TURN_MIN, FLEE_LINE_TURN_MAX);
        vel_left = -vel_right;

        if (ir_last_seen[0] < 1000 || ir_last_seen[1] < 1000) {
          //if the front sensors activated recently, use them to determine
          // which way to spin ie if left last saw the line, spin right, else spin left
          if (ir_last_seen[0] < ir_last_seen[1]) {
            vel_left *= -1;
            vel_right *= -1;
          }
        } else {
          // pick a random direction
          if (random(0, 2)) {
            vel_left *= -1;
            vel_right *= -1;
          }
        }
        _drive->incPosition(_drive->LEFT, vel_left);
        _drive->incPosition(_drive->RIGHT, vel_right);
      }

      // use OR because this does not need to be precise
      // and can assume both axis would have finished at same time
      if (_drive->moveDone(_drive->LEFT) || _drive->moveDone(_drive->RIGHT)) {
        new_state = SEEK_FORWARD;
      }
      break;
    case START_ST1_R_REV:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered START_ST1_R_REV");

        _drive->incPosition(_drive->LEFT, 45 * 4000);
        _drive->incPosition(_drive->RIGHT, -1 * 4000);
      }

      if (_drive->moveDone(_drive->RIGHT)) {
        new_state = START_ST2_FW;
      }
      break;
    case START_ST2_FW:
      if (_auton_state != _pAuton_state) {
        Serial1.println("Entered START_ST2_FW");
        
        _drive->incPosition(_drive->RIGHT, 80 * 4000);
      }

      if (_drive->moveDone(_drive->LEFT) && _drive->moveDone(_drive->RIGHT)) {
        new_state = TRACK;
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

  if (ir[0] || ir[1] || ir[2] || ir[3]) {
    if (
      _auton_state != NONE &&
      _auton_state != START_ST1_R_REV &&
      _auton_state != START_ST2_FW &&
      _auton_state != FLEE_LINE_REV &&
      _auton_state != FLEE_LINE_TURN
    ) {
      new_state = FLEE_LINE_REV;
    }
  }

  if (new_state != INVALID) {
    _auton_state = new_state;
    last_state_change = 0;
  }
}
