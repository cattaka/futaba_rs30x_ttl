#include"futaba_rs30x_ttl.h"

FutabaRs30xTtl gFutabaRs30xTtl(Serial3, 2);

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  gFutabaRs30xTtl.setup(1, true);

  gFutabaRs30xTtl.setTorqueEnable(1, 0);
}

void loop() {
  gFutabaRs30xTtl.writeMemoryMap(1, 0x05, 0, 0, NULL);
//  gFutabaRs30xTtl.feedSerial();
  Serial.println((int16_t)gFutabaRs30xTtl.getPresentPosion(1));
  delay(100);
}
