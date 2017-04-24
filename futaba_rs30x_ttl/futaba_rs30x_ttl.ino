#include <SoftwareSerial.h>

#include"futaba_rs30x_ttl.h"


SoftwareSerial gSerial(2, 4);
FutabaRs30xTtl gFutabaRs30xTtl(gSerial, 3);

void setup() {
  Serial.begin(115200);
  gSerial.begin(115200);
  gFutabaRs30xTtl.setup(1, true);

  gFutabaRs30xTtl.setTorqueEnable(1, 0);
}

void loop() {
  gFutabaRs30xTtl.writeMemoryMap(1, 0x05, 0, 0, NULL);
  gFutabaRs30xTtl.feedSerial();
  Serial.println(gFutabaRs30xTtl.getPresentPosion(1));
  delay(100);
}
