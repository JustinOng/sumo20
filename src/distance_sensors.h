#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include <Wire.h>
#include <VL53L0X.h>

#define PORT_EXPANDER_ADDR 0x20
#define NUM_VL53L0X 4
// base address to remap the sensors to
#define VL53L0X_BASE_ADDR 0x30

class Distance_Sensors {
  public:
    void loop(void);

    bool initialised[NUM_VL53L0X] = {0};
    uint16_t distance[NUM_VL53L0X] = {0};
  private:
    byte port_expander_state = 0;
    VL53L0X distance_sensors[NUM_VL53L0X];
};

#endif
