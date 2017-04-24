#ifndef FUTABA_RS30X_TTL_H
#define FUTABA_RS30X_TTL_H

#include<Stream.h>

class FutabaRs30xTtl {
  private:
    Stream* mStream;
    uint8_t mPinNoRwSwitch;
    uint8_t mMaximumServoId;
    uint8_t mReturnPacketBufSize;
    uint8_t** mMemoryMap;

    uint8_t mReadState;
    uint8_t mReadId;
    uint8_t mReadFlag;
    uint8_t mReadAddress;
    uint8_t mReadLength;
    uint8_t mReadCount;
    uint8_t mReadDataCount;
    uint8_t* mReturnPacketBuf;
    int mWaitTimeout;

    bool waitAvailable(int timeout);

  public:
    FutabaRs30xTtl(Stream& stream, uint8_t pinNoRwSwitch, uint8_t maximumServoId = 24, uint8_t returnPacketBufSize = 0x40, int waitTimeout = 100);
    virtual ~FutabaRs30xTtl();
    void setup(uint8_t id, bool readAllMemoryMap);

    // general-purpose methods
    void feedSerial();
    void writeMemoryMap(uint8_t id, uint8_t flag, uint8_t address, uint8_t len, uint8_t* data);
    void readMemoryMap(uint8_t id, uint8_t address, uint8_t len);

    // getter methods
    uint8_t getServoId(uint8_t id);
    uint8_t getReverse(uint8_t id);
    uint8_t getBaudRate(uint8_t id);
    uint8_t getReturnDelay(uint8_t id);
    uint16_t getCwAngleLimit(uint8_t id);
    uint16_t getCcwAngleLimit(uint8_t id);
    uint16_t getTemperatureLimit(uint8_t id);
    uint8_t getTorqueInSilence(uint8_t id);
    uint8_t getWarmupTime(uint8_t id);
    uint8_t getCwComplianceMargin(uint8_t id);
    uint8_t getCcwComplianceMargin(uint8_t id);
    uint8_t getCwComplianceSlope(uint8_t id);
    uint8_t getCcwComplianceSlope(uint8_t id);
    uint8_t getPunch(uint8_t id);
    uint16_t getGoalPosition(uint8_t id);
    uint16_t getGoalTime(uint8_t id);
    uint8_t getMaxTorque(uint8_t id);
    uint8_t getTorqueEnable(uint8_t id);
    uint16_t getPresentPosion(uint8_t id);
    uint16_t getPresentTime(uint8_t id);
    uint16_t getPresentSpeed(uint8_t id);
    uint16_t getPresentCurrent(uint8_t id);
    uint16_t getPresentTemperature(uint8_t id);
    uint16_t getPresentVolts(uint8_t id);

    void setServoId(uint8_t id, uint8_t value, bool flashRom = false);
    void setReverse(uint8_t id, uint8_t value, bool flashRom = false);
    void setBaudRate(uint8_t id, uint8_t value, bool flashRom = false);
    void setReturnDelay(uint8_t id, uint8_t value, bool flashRom = false);
    void setCwAngleLimit(uint8_t id, uint16_t value, bool flashRom = false);
    void setCcwAngleLimit(uint8_t id, uint16_t value, bool flashRom = false);
    void setTorqueInSilence(uint8_t id, uint8_t value, bool flashRom = false);
    void setWarmupTime(uint8_t id, uint8_t value, bool flashRom = false);
    void setCwComplianceMargin(uint8_t id, uint8_t value, bool flashRom = false);
    void setCcwComplianceMargin(uint8_t id, uint8_t value, bool flashRom = false);
    void setCwComplianceSlope(uint8_t id, uint8_t value, bool flashRom = false);
    void setCcwComplianceSlope(uint8_t id, uint8_t value, bool flashRom = false);
    void setPunch(uint8_t id, uint8_t value, bool flashRom = false);
    void setGoalPosition(uint8_t id, uint16_t value, bool flashRom = false);
    void setGoalTime(uint8_t id, uint16_t value, bool flashRom = false);
    void setMaxTorque(uint8_t id, uint8_t value, bool flashRom = false);
    void setTorqueEnable(uint8_t id, uint8_t value, bool flashRom = false);
};

#endif // FUTABA_RS30X_TTL_H

