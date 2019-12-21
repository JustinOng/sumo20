#include <Arduino.h>
#include "FlySkyIBus.h"

#include "robot.h"

// #define LOG_CONTROLLER_VALUES

// power (0 - 100)
#define VACUUM_ON 60

Robot robot;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  IBus.begin(Serial2);

  robot.begin();

  pinMode(13, OUTPUT);
}

void loop() {
  IBus.loop();
  robot.loop();

  digitalWrite(13, IBus.is_alive());

  if (IBus.is_alive()) {
    int8_t forward = (int8_t) map(IBus.readChannel(1), 1000, 2000, -100, 100);
    int8_t turn = (int8_t) map(IBus.readChannel(0), 1000, 2000, -100, 100);

    robot.setSpeed(forward, turn);
    if (IBus.readChannel(6) > 1500) {
      robot.setVacuum(VACUUM_ON);
    } else {
      robot.setVacuum(0);
    }
  } else {
    robot.setSpeed(0, 0);
  }

#ifdef LOG_CONTROLLER_VALUES
    for (int i = 0; i < 10; i++) {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(IBus.readChannel(i));
      Serial.print(" ");
    }
    Serial.println();
#endif
}
