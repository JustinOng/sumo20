#include <Arduino.h>
#include "FlySkyIBus.h"

#include "robot.h"

// #define LOG_CONTROLLER_VALUES

// power (0 - 100)
#define VACUUM_ON 60

Robot robot;

void setup() {
  Serial1.begin(115200);
  Serial1.println("Starting");

  IBus.begin(Serial1);

  robot.begin();

  FastLED.setBrightness(100);

  pinMode(13, OUTPUT);

  // seed with internal temperature sensor
  randomSeed(analogRead(38));
}

void loop() {
  IBus.loop();

  digitalWrite(13, IBus.is_alive());

  if (IBus.is_alive()) {
    int8_t forward = (int8_t) map(IBus.readChannel(1), 1000, 2000, -100, 100);
    int8_t turn = (int8_t) map(IBus.readChannel(0), 1000, 2000, -100, 100);

    robot.setSpeed(forward, turn);
    if (IBus.readChannel(6) > 1500) {
      if (IBus.readChannel(7) > 1500) {
        robot.setVacuumRaw(IBus.readChannel(4));
      } else {
        robot.setVacuum(VACUUM_ON);
      }
    } else {
      robot.setVacuum(0);
    }

    robot.setLifter(IBus.readChannel(2));
    
    if (IBus.readChannel(9) > 1500) {
      robot.setMode(Robot::MODE_AUTON);
    } else {
      robot.setMode(Robot::MODE_RC);
    }
  } else {
    robot.setSpeed(0, 0);
    robot.setMode(Robot::MODE_RC);
  }

#ifdef LOG_CONTROLLER_VALUES
    for (int i = 0; i < 10; i++) {
      Serial1.print(i);
      Serial1.print(": ");
      Serial1.print(IBus.readChannel(i));
      Serial1.print(" ");
    }
    Serial1.println();
#endif
  robot.loop();
}
