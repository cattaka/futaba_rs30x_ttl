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
    uint8_t mReadAddress;
    uint8_t mReadLength;
    uint8_t mReadCount;
    uint8_t* mReturnPacketBuf;

  public:
    FutabaRs30xTtl(Stream& stream, uint8_t pinNoRwSwitch, uint8_t maximumServoId = 24, uint8_t returnPacketBufSize = 0x40);
    virtual ~FutabaRs30xTtl();
    void setup(uint8_t id, bool readAllMemoryMap);
  
    // general-purpose methods
    void feedSerial();
    void readMemoryMap(uint8_t id, uint16_t address, uint8_t len);
    void writeMemoryMap(uint8_t id, bool flashRom, uint16_t address, uint8_t len, uint8_t* data);
    
    // getter-setter methods
    uint8_t getServoId(bool requestPacket);
    void setServoId(bool requestPacket, uint8_t value, bool flashRom = false);
    bool getReverse(bool requestPacket);
    void setReverse(bool requestPacket, bool value, bool flashRom = false);
    uint8_t getBaudRate(bool requestPacket);
    void setBaudRate(bool requestPacket, uint8_t value, bool flashRom = false);
    uint8_t getReturnDelay(bool requestPacket);
    void setReturnDelay(bool requestPacket, uint8_t value, bool flashRom = false);
    uint16_t getCwAngleLimit(bool requestPacket);
    void setCwAngleLimit(bool requestPacket, uint16_t value, bool flashRom = false);
    uint16_t getCcwAngleLimit(bool requestPacket);
    void setCcwAngleLimit(bool requestPacket, uint16_t value, bool flashRom = false);
    uint16_t getTemperatureLimit(bool requestPacket);
    uint8_t getTorqueInSilence(bool requestPacket);
    void setTorqueInSilence(bool requestPacket, uint8_t value, bool flashRom = false);
    uint8_t getWarmupTime(bool requestPacket);
    void setWarmupTime(bool requestPacket, uint8_t value, bool flashRom = false);
    uint8_t getCwComplianceMargin(bool requestPacket);
    void setCwComplianceMargin(bool requestPacket, uint8_t value, bool flashRom = false);
    uint8_t getCcwComplianceMargin(bool requestPacket);
    void setCcwComplianceMargin(bool requestPacket, uint8_t value, bool flashRom = false);
    uint8_t getCwComplianceSlope(bool requestPacket);
    void setCwComplianceSlope(bool requestPacket, uint8_t value, bool flashRom = false);
    uint8_t getCcwComplianceSlope(bool requestPacket);
    void setCcwComplianceSlope(bool requestPacket, uint8_t value, bool flashRom = false);
    uint8_t getPunch(bool requestPacket); void setPunch(bool requestPacket, uint8_t value, bool flashRom = false);
    uint16_t getGoalPosition(bool requestPacket);
    void setGoalPosition(bool requestPacket, uint16_t value, bool flashRom = false);
    uint16_t getGoalTime(bool requestPacket);
    void setGoalTime(bool requestPacket, uint16_t value, bool flashRom = false);
    uint8_t getMaxTorque(bool requestPacket);
    void setMaxTorque(bool requestPacket, uint8_t value, bool flashRom = false);
    uint8_t getTorqueEnable(bool requestPacket);
    void setTorqueEnable(bool requestPacket, uint8_t value, bool flashRom = false);
    uint16_t getPresentPosion(bool requestPacket);
    uint16_t getPresentTime(bool requestPacket);
    uint16_t getPresentSpeed(bool requestPacket);
    uint16_t getPresentCurrent(bool requestPacket);
    uint16_t getPresentTemperature(bool requestPacket);
    uint16_t getPresentVolts(bool requestPacket);
};

#endif // FUTABA_RS30X_TTL_H

