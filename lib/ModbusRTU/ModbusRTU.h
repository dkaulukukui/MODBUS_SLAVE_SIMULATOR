// Generic MODBUS RTU client for RS-485 sensors
// Supports multiple devices on the same bus with different addresses

#ifndef MODBUSRTU_H
#define MODBUSRTU_H

#include <Arduino.h>

class ModbusRTU {
public:
  ModbusRTU(Stream* serial);
  
  // Read single 32-bit float from holding registers
  bool readFloat(uint8_t slaveAddress, uint16_t registerAddress, float& value, uint16_t timeout_ms = 1000);
  
  // Read multiple 32-bit floats from holding registers
  bool readMultipleFloats(uint8_t slaveAddress, uint16_t startAddress, uint8_t numFloats, float* values, uint16_t timeout_ms = 1000);
  
  // Read single 16-bit register
  bool readRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t& value, uint16_t timeout_ms = 1000);
  
  // Read multiple 16-bit registers
  bool readMultipleRegisters(uint8_t slaveAddress, uint16_t startAddress, uint8_t numRegisters, uint16_t* values, uint16_t timeout_ms = 1000);
  
  // Write single 16-bit register (function code 0x06)
  bool writeSingleRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t value, uint16_t timeout_ms = 1000);
  
  // Write multiple 16-bit registers (function code 0x10)
  bool writeMultipleRegisters(uint8_t slaveAddress, uint16_t startAddress, uint8_t numRegisters, uint16_t* values, uint16_t timeout_ms = 1000);
  
  // Set inter-query delay (milliseconds between consecutive queries)
  void setInterQueryDelay(uint16_t delay_ms) { _interQueryDelay = delay_ms; }
  
  // Get last error code
  uint8_t getLastError() { return _lastError; }
  
private:
  Stream* _serial;
  uint16_t _interQueryDelay;  // Delay between queries (default 100ms)
  unsigned long _lastQueryTime;
  uint8_t _lastError;
  
  // MODBUS function codes
  static const uint8_t FUNCTION_READ_HOLDING_REGISTERS = 0x03;
  static const uint8_t FUNCTION_WRITE_SINGLE_REGISTER = 0x06;
  static const uint8_t FUNCTION_WRITE_MULTIPLE_REGISTERS = 0x10;
  
  // Error codes
  static const uint8_t ERROR_NONE = 0x00;
  static const uint8_t ERROR_TIMEOUT = 0xE0;
  static const uint8_t ERROR_CRC = 0xE1;
  static const uint8_t ERROR_EXCEPTION = 0xE2;
  static const uint8_t ERROR_INVALID_LENGTH = 0xE3;
  
  // Calculate MODBUS CRC16
  uint16_t calculateCRC(uint8_t* buffer, uint8_t length);
  
  // Send MODBUS request
  void sendRequest(uint8_t* request, uint8_t length);
  
  // Receive MODBUS response
  bool receiveResponse(uint8_t* response, uint8_t expectedLength, uint16_t timeout_ms);
  
  // Convert two 16-bit registers to IEEE 754 float (little-endian word order)
  float registersToFloat(uint16_t lsw, uint16_t msw);
  
  // Enforce inter-query delay
  void enforceDelay();
};

#endif // MODBUSRTU_H
