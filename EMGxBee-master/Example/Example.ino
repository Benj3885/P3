#include "EMGxBee.h"

EMGxBee xbee;
IntervalTimer xBeeTimer;

void xBeeRead(){
  xbee.updateData();
}

void setup() {
  Serial.begin(9600);
  xbee.begin(Serial1, 115200);
  xBeeTimer.begin(xBeeRead, 10000);
}

void loop() {
  Serial.print("Z = ");
  Serial.print(xbee.getAccZ());
  Serial.println();
  Serial.print("EMG1 = ");
  Serial.print(xbee.getEMG1());
  Serial.println();
  delay(250);
}
