#include "robot.h"

Robot::Robot(void) {
  _drive = new Drive(&Serial3);
  _vacuum = new Servo();

  _vacuum->attach(PIN_VACUUM);
}

void Robot::begin(void) {
  Serial3.begin(115200);
  LEDS.addLeds<WS2812B, PIN_LED_INT, RGB>(leds_int, NUM_LED_INT);

  // initialises 19/18 as SCL/SDA
  Wire.begin();
  Wire.setTimeout(1);
  
  begin_vl53l0x();
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

void Robot::begin_vl53l0x(void) {
  // P0 to P3 of the port expander is connected
  // to XSHUT of the sensors
  byte en_sensors = 0;
  byte error;

  for(byte i = 0; i < NUM_VL53L0X; i++) {
    en_sensors = (en_sensors << 1) | 1;
    Wire.beginTransmission(PORT_EXPANDER_ADDR);
    Wire.write(en_sensors);
    error = Wire.endTransmission();

    if (error != 0) {
      Serial1.print("Failed to write to port expander: "); Serial1.println(error);
    }

    if (!distance_sensors[i].init()) {
      Serial1.print("Failed to initialise distance sensor "); Serial1.println(i);
      continue;
    }

    Serial1.print("Initialised distance sensor "); Serial1.println(i);
    distance_sensors[i].setAddress(VL53L0X_BASE_ADDR | i);

    distance_sensors[i].setTimeout(1);
    distance_sensors[i].startContinuous();
  }
}

void Robot::loop(void) {
  for (byte i = 0; i < NUM_IR; i++) {
    ir[i] = analogRead(ir_pins[i]) < IR_THRESHOLD;
  }

  for (byte i = 0; i < NUM_VL53L0X; i++) {
    uint16_t val = distance_sensors[i].readRangeContinuousMillimeters();
    if (val != 65535) {
      distance[i] = val;
    }
  }
}
