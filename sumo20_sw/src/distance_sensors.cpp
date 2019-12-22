#include "distance_sensors.h"

void Distance_Sensors::loop(void) {
  for(byte i = 0; i < NUM_VL53L0X; i++) {
    byte error;
    
    if (initialised[i]) continue;

    port_expander_state |= (1 << i);

    Wire.beginTransmission(PORT_EXPANDER_ADDR);
    Wire.write(port_expander_state);
    error = Wire.endTransmission();

    if (error != 0) {
      Serial1.print("Failed to write to port expander: "); Serial1.println(error);
      break;
    }

    // the vl53l0x takes up to 1.2ms to boot after XSHUT goes HIGH
    delay(2);

    if (!distance_sensors[i].init()) {
      Serial1.print("Failed to initialise distance sensor "); Serial1.println(i);

      // shut down the sensor
      port_expander_state &= ~(1 << i);
      Wire.beginTransmission(PORT_EXPANDER_ADDR);
      Wire.write(port_expander_state);
      Wire.endTransmission();

      continue;
    }

    initialised[i] = 1;

    Serial1.print("Initialised distance sensor "); Serial1.println(i);
    distance_sensors[i].setAddress(VL53L0X_BASE_ADDR | i);

    distance_sensors[i].setTimeout(1);
    distance_sensors[i].startContinuous();
  }

  for (byte i = 0; i < NUM_VL53L0X; i++) {
    uint16_t val = distance_sensors[i].readRangeContinuousMillimeters();
    if (val != 65535) {
      distance[i] = val;
    }
  }
}
