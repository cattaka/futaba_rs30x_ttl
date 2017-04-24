#include"futaba_rs30x_ttl.h"

#define memoryMap(i) mMemoryMap[i-1]

FutabaRs30xTtl::FutabaRs30xTtl(Stream& stream, uint8_t pinNoRwSwitch, uint8_t maximumServoId, uint8_t returnPacketBufSize) {
  mStream = &stream;
  mPinNoRwSwitch = pinNoRwSwitch;
  mMaximumServoId = maximumServoId;
  mReturnPacketBufSize = returnPacketBufSize;
  mReturnPacketBuf = new uint8_t[mReturnPacketBufSize];
  memset(mReturnPacketBuf, 0, sizeof(uint8_t) * mReturnPacketBufSize);
  mMemoryMap = new uint8_t*[mMaximumServoId];
  memset(mMemoryMap, 0, sizeof(mMemoryMap) * mMaximumServoId);
}

FutabaRs30xTtl::~FutabaRs30xTtl() {
  delete mReturnPacketBuf;
  for (unsigned int i = 0; i < mMaximumServoId; i++) {
    if (mMemoryMap[i]) {
      delete mMemoryMap[i];
    }
  }
  delete mMemoryMap;
}

void FutabaRs30xTtl::setup(uint8_t id, bool readAllMemoryMap) {
  if (!memoryMap(id)) {
    memoryMap(id) = new uint8_t[mReturnPacketBufSize];
    memset(memoryMap(id), 0, sizeof(uint8_t) * mReturnPacketBufSize);
  }
  if (readAllMemoryMap) {
    // TODO
  }
}

void FutabaRs30xTtl::feedSerial() {
    // TODO
}

