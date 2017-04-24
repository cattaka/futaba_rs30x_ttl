#include<Arduino.h>
#include"futaba_rs30x_ttl.h"

// #define FUTABA_RS30X_TTL_DEBUG

#define memoryMap(id) (mMemoryMap[id-1])
#define read_uint8_t(id,addr) (memoryMap(id) ? memoryMap(id)[addr] : 0)
#define read_uint16_t(id,addr) (memoryMap(id) ? ((uint16_t)memoryMap(id)[addr] | ((uint16_t)memoryMap(id)[addr+1]<<8)) : 0)
#define write_uint8_t(id,addr,value,flashRom) {uint8_t buf[] = {value}; writeMemoryMap(id, flashRom?0x40:0x00, addr, 1, buf);}
#define write_uint16_t(id,addr,value,flashRom) {uint8_t buf[] = {(uint8_t)value, (uint8_t)(value>>8)}; writeMemoryMap(id, flashRom?0x40:0x00, addr, 2, buf);}

enum ReadState {
  RS_UNKNOWN,
  RS_HDR_2,
  RS_ID,
  RS_FLG,
  RS_ADR,
  RS_LEN,
  RS_CNT,
  RS_DAT,
  RS_SUM
};

uint8_t calcSum(uint8_t* buf, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum ^= buf[i];
  }
  return sum;
}


FutabaRs30xTtl::FutabaRs30xTtl(Stream& stream, uint8_t pinNoRwSwitch, uint8_t maximumServoId, uint8_t returnPacketBufSize, int waitTimeout) {
  mStream = &stream;
  mPinNoRwSwitch = pinNoRwSwitch;
  mMaximumServoId = maximumServoId;
  mReturnPacketBufSize = returnPacketBufSize;
  mReturnPacketBuf = new uint8_t[mReturnPacketBufSize];
  memset(mReturnPacketBuf, 0, sizeof(uint8_t) * mReturnPacketBufSize);
  mMemoryMap = new uint8_t*[mMaximumServoId];
  memset(mMemoryMap, 0, sizeof(mMemoryMap) * mMaximumServoId);
  mWaitTimeout = waitTimeout;
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
    this->writeMemoryMap(id, 0x03, 0, 0, NULL);
    this->writeMemoryMap(id, 0x05, 0, 0, NULL);
  }
}

bool FutabaRs30xTtl::waitAvailable(int timeout) {
  unsigned long t = millis();
  while (mStream->available() == 0) {
    if ((millis() - t) > timeout) {
      return false;
    }
  }
  return true;
}

void FutabaRs30xTtl::feedSerial() {
  while (mStream->available() > 0) {
    uint8_t ch = mStream->read();
    switch (mReadState) {
      case RS_UNKNOWN: {
          if (ch == 0xFD) {
            mReadState = RS_HDR_2;
          }
          break;
        }
      case RS_HDR_2: {
          if (ch == 0xDF) {
            mReadState = RS_ID;
          } else if (ch == 0xFD) {
            mReadState = RS_HDR_2;
          } else {
            mReadState = RS_UNKNOWN;
          }
          break;
        }
      case RS_ID: {
          mReadId = ch;
          mReadState = RS_FLG;
          break;
        }
      case RS_FLG: {
          mReadFlag = ch;
          mReadState = RS_ADR;
          break;
        }
      case RS_ADR: {
          mReadAddress = ch;
          mReadState = RS_LEN;
          break;
        }
      case RS_LEN: {
          mReadLength = ch;
          mReadState = RS_CNT;
          break;
        }
      case RS_CNT: {
          mReadCount = ch;
          mReadDataCount = 0;
          mReadState = (mReadLength > 0) ? RS_DAT : RS_SUM;
          break;
        }
      case RS_DAT: {
          if (mReadDataCount < mReturnPacketBufSize) {
            mReturnPacketBuf[mReadDataCount] = ch;
          }
          mReadDataCount++;
          if (mReadDataCount == mReadLength) {
            mReadState = RS_SUM;
          }
          break;
        }
      case RS_SUM: {
          uint8_t sum = mReadId ^ mReadFlag ^ mReadAddress ^ mReadLength ^ mReadCount ^ calcSum(mReturnPacketBuf, mReadDataCount);
          if (memoryMap(mReadId)
              && sum == ch
              && mReadAddress + mReadLength < mReturnPacketBufSize) {
            memcpy(memoryMap(mReadId) + mReadAddress, mReturnPacketBuf, mReadLength);
#ifdef FUTABA_RS30X_TTL_DEBUG
            Serial.println("succeed");
#endif
          } else {
#ifdef FUTABA_RS30X_TTL_DEBUG
            Serial.println("error.begin");
            Serial.println(mReadId, HEX);
            Serial.println(mReadFlag, HEX);
            Serial.println(mReadAddress, HEX);
            Serial.println(mReadLength, HEX);
            Serial.println(mReadDataCount, HEX);
            Serial.println(ch, HEX);
            Serial.println(sum, HEX);
            Serial.println(calcSum(mReturnPacketBuf, mReadDataCount), HEX);
            Serial.println("error.end");
#endif
          }
          mReadState = RS_UNKNOWN;
          break;
        }
    }
  }
}

void FutabaRs30xTtl::writeMemoryMap(uint8_t id, uint8_t flag, uint8_t address, uint8_t len, uint8_t* data) {
  uint8_t cnt = (flag != 0x0f) ? 1 : 0;
  uint8_t sum = id ^ flag ^ address ^ len ^ cnt;
  if (cnt && len > 0) {
    sum = sum ^ calcSum(data, len);
  }
#ifdef FUTABA_RS30X_TTL_DEBUG
  Serial.println(id, HEX);
  Serial.println(flag, HEX);
  Serial.println(address, HEX);
  Serial.println(len, HEX);
  Serial.println(cnt, HEX);
  Serial.println(sum, HEX);
  Serial.flush();
#endif

  digitalWrite(mPinNoRwSwitch, HIGH);
  mStream->write(0xFA);
  mStream->write(0xAF);
  mStream->write(id);
  mStream->write(flag);
  mStream->write(address);
  mStream->write(len);
  mStream->write(cnt);
  if (cnt && len > 0) {
    mStream->write(data, len);
  }
  mStream->write(sum);
  digitalWrite(mPinNoRwSwitch, LOW);

  if (flag != 0) {
    waitAvailable(mWaitTimeout);
    feedSerial();
  }
}

void FutabaRs30xTtl::readMemoryMap(uint8_t id, uint8_t address, uint8_t len) {
  uint8_t buf[0];
  this->writeMemoryMap(id, 0x0f, address, len, NULL);
}

uint8_t FutabaRs30xTtl::getServoId(uint8_t id) { return read_uint8_t(id,0x04); }
uint8_t FutabaRs30xTtl::getReverse(uint8_t id) { return read_uint8_t(id,0x05); }
uint8_t FutabaRs30xTtl::getBaudRate(uint8_t id) { return read_uint8_t(id,0x06); }
uint8_t FutabaRs30xTtl::getReturnDelay(uint8_t id) { return read_uint8_t(id,0x07); }
uint16_t FutabaRs30xTtl::getCwAngleLimit(uint8_t id) { return read_uint16_t(id,0x08); }
uint16_t FutabaRs30xTtl::getCcwAngleLimit(uint8_t id) { return read_uint16_t(id,0x0A); }
uint16_t FutabaRs30xTtl::getTemperatureLimit(uint8_t id) { return read_uint16_t(id,0x0E); }
uint8_t FutabaRs30xTtl::getTorqueInSilence(uint8_t id) { return read_uint8_t(id,0x16); }
uint8_t FutabaRs30xTtl::getWarmupTime(uint8_t id) { return read_uint8_t(id,0x17); }
uint8_t FutabaRs30xTtl::getCwComplianceMargin(uint8_t id) { return read_uint8_t(id,0x18); }
uint8_t FutabaRs30xTtl::getCcwComplianceMargin(uint8_t id) { return read_uint8_t(id,0x19); }
uint8_t FutabaRs30xTtl::getCwComplianceSlope(uint8_t id) { return read_uint8_t(id,0x1A); }
uint8_t FutabaRs30xTtl::getCcwComplianceSlope(uint8_t id) { return read_uint8_t(id,0x1B); }
uint8_t FutabaRs30xTtl::getPunch(uint8_t id) { return read_uint8_t(id,0x1C); }
uint16_t FutabaRs30xTtl::getGoalPosition(uint8_t id) { return read_uint16_t(id,0x1E); }
uint16_t FutabaRs30xTtl::getGoalTime(uint8_t id) { return read_uint16_t(id,0x20); }
uint8_t FutabaRs30xTtl::getMaxTorque(uint8_t id) { return read_uint8_t(id,0x23); }
uint8_t FutabaRs30xTtl::getTorqueEnable(uint8_t id) { return read_uint8_t(id,0x24); }
uint16_t FutabaRs30xTtl::getPresentPosion(uint8_t id) { return read_uint16_t(id,0x2A); }
uint16_t FutabaRs30xTtl::getPresentTime(uint8_t id) { return read_uint16_t(id,0x2C); }
uint16_t FutabaRs30xTtl::getPresentSpeed(uint8_t id) { return read_uint16_t(id,0x2E); }
uint16_t FutabaRs30xTtl::getPresentCurrent(uint8_t id) { return read_uint16_t(id,0x30); }
uint16_t FutabaRs30xTtl::getPresentTemperature(uint8_t id) { return read_uint16_t(id,0x32); }
uint16_t FutabaRs30xTtl::getPresentVolts(uint8_t id) { return read_uint16_t(id,0x34); }

void FutabaRs30xTtl::setServoId(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x04,value,flashRom); }
void FutabaRs30xTtl::setReverse(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x05,value,flashRom); }
void FutabaRs30xTtl::setBaudRate(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x06,value,flashRom); }
void FutabaRs30xTtl::setReturnDelay(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x07,value,flashRom); }
void FutabaRs30xTtl::setCwAngleLimit(uint8_t id, uint16_t value, bool flashRom) { write_uint16_t(id,0x08,value,flashRom); }
void FutabaRs30xTtl::setCcwAngleLimit(uint8_t id, uint16_t value, bool flashRom) { write_uint16_t(id,0x0A,value,flashRom); }
void FutabaRs30xTtl::setTorqueInSilence(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x16,value,flashRom); }
void FutabaRs30xTtl::setWarmupTime(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x17,value,flashRom); }
void FutabaRs30xTtl::setCwComplianceMargin(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x18,value,flashRom); }
void FutabaRs30xTtl::setCcwComplianceMargin(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x19,value,flashRom); }
void FutabaRs30xTtl::setCwComplianceSlope(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x1A,value,flashRom); }
void FutabaRs30xTtl::setCcwComplianceSlope(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x1B,value,flashRom); }
void FutabaRs30xTtl::setPunch(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x1C,value,flashRom); }
void FutabaRs30xTtl::setGoalPosition(uint8_t id, uint16_t value, bool flashRom) { write_uint16_t(id,0x1E,value,flashRom); }
void FutabaRs30xTtl::setGoalTime(uint8_t id, uint16_t value, bool flashRom) { write_uint16_t(id,0x20,value,flashRom); }
void FutabaRs30xTtl::setMaxTorque(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x23,value,flashRom); }
void FutabaRs30xTtl::setTorqueEnable(uint8_t id, uint8_t value, bool flashRom) { write_uint8_t(id,0x24,value,flashRom); }

