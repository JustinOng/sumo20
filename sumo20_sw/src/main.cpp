#include <Arduino.h>
#include <Servo.h>
#include "drive.h"
#include "FlySkyIBus.h"

// #define LOG_CONTROLLER_VALUES

#define PIN_VACUUM 2

// ms to send for vacuum on/off state
#define VACUUM_OFF 1000
#define VACUUM_ON 1600

// scale left and right drive powers
#define LEFT_SCALE 1
#define RIGHT_SCALE -1

// ms at which odrive should be updated
#define DRIVE_UPDATE_THROTTLE 50

Drive drive(&Serial3);
Servo vacuum;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  vacuum.attach(PIN_VACUUM);

  IBus.begin(Serial2);

  pinMode(13, OUTPUT);
}

void loop() {
  static elapsedMillis driveDelay;
  IBus.loop();

  digitalWrite(13, IBus.is_alive());

  if (driveDelay > DRIVE_UPDATE_THROTTLE) {
    int8_t forward = (int8_t) map(IBus.readChannel(1), 1000, 2000, -100, 100);
    int8_t turn = (int8_t) map(IBus.readChannel(0), 1000, 2000, -100, 100);

    // deadzone
    if (abs(forward) < 5) forward = 0;
    if (abs(turn) < 5) turn = 0;

    drive.setSpeed(Drive::LEFT, (forward + turn) * LEFT_SCALE);
    drive.setSpeed(Drive::RIGHT, (forward - turn) * RIGHT_SCALE);

    driveDelay = 0;

#ifdef LOG_CONTROLLER_VALUES
    for (int i = 0; i < 10; i++) {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(IBus.readChannel(i));
      Serial.print(" ");
    }
    Serial.println();
#endif

    if (IBus.readChannel(6) > 1500) {
      vacuum.writeMicroseconds(VACUUM_ON);
    } else {
      vacuum.writeMicroseconds(VACUUM_OFF);
    }
  }
}
