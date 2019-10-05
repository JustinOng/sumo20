#include <Arduino.h>
elapsedMicros ele_time;
volatile uint32_t pulse_len[3] = {0};

template <uint8_t pin, uint8_t channel>
void ch_service() {
  if(digitalReadFast(pin)) {
    ele_time = 0;
  } else {
    pulse_len[channel] = ele_time;
  }
}

void setup() {
  Serial.begin(115200);

  attachInterrupt(23, ch_service<23, 0>, CHANGE);
  attachInterrupt(22, ch_service<22, 1>, CHANGE);
  attachInterrupt(21, ch_service<21, 2>, CHANGE);

  pinMode(13, OUTPUT);
}

void loop() {
  Serial.print(pulse_len[0]);
  Serial.print(" ");
  Serial.print(pulse_len[1]);
  Serial.print(" ");
  Serial.println(pulse_len[2]);
}
