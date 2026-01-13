// Helper class for Vaisala HMP110 Humidity and Temperature Probe
// Provides convenient methods to read sensor data via MODBUS RTU

#ifndef HMP110_H
#define HMP110_H

#include <Arduino.h>
#include "ModbusRTU.h"

class HMP110 {
public:
  HMP110(ModbusRTU* modbus, uint8_t slaveAddress = 0xF0);
  
  // Read both humidity and temperature at once (most efficient)
  bool readAllData(float& humidity, float& temperature, uint16_t timeout_ms = 1000);
  
  // Read individual parameters
  bool readHumidity(float& humidity, uint16_t timeout_ms = 1000);
  bool readTemperature(float& temperature, uint16_t timeout_ms = 1000);
  bool readDewPoint(float& dewPoint, uint16_t timeout_ms = 1000);
  
  // Get last error from ModbusRTU
  uint8_t getLastError() { return _modbus->getLastError(); }
  
private:
  ModbusRTU* _modbus;
  uint8_t _slaveAddress;
  
  // HMP110 MODBUS register addresses (from HMP110 User Guide Appendix A)
  // Floating point values (32-bit IEEE 754)
  static const uint16_t REG_HUMIDITY = 0x0000;     // Relative humidity (LSB at 0x0000, MSB at 0x0001)
  static const uint16_t REG_TEMPERATURE = 0x0002;  // Temperature in °C (LSB at 0x0002, MSB at 0x0003)
  static const uint16_t REG_DEW_POINT = 0x0008;    // Dew/frost point temperature (LSB at 0x0008, MSB at 0x0009)
};

#endif // HMP110_H
