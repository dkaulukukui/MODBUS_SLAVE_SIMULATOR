#include "ModbusRTU.h"

ModbusRTU::ModbusRTU(Stream* serial) 
  : _serial(serial), _interQueryDelay(100), _lastQueryTime(0), _lastError(ERROR_NONE) {
}

uint16_t ModbusRTU::calculateCRC(uint8_t* buffer, uint8_t length) {
  uint16_t crc = 0xFFFF;
  
  for (uint8_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    
    for (uint8_t i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

void ModbusRTU::sendRequest(uint8_t* request, uint8_t length) {
  // Enforce inter-query delay
  enforceDelay();
  
  // Clear any pending data in receive buffer
  while (_serial->available()) {
    _serial->read();
  }
  
  // Calculate and append CRC
  uint16_t crc = calculateCRC(request, length);
  request[length] = crc & 0xFF;         // CRC low byte
  request[length + 1] = (crc >> 8) & 0xFF; // CRC high byte
  
  // Send request
  _serial->write(request, length + 2);
  _serial->flush();
  
  // Update last query time
  _lastQueryTime = millis();
}

bool ModbusRTU::receiveResponse(uint8_t* response, uint8_t expectedLength, uint16_t timeout_ms) {
  unsigned long startTime = millis();
  uint8_t bytesRead = 0;
  
  // Wait for response with timeout
  while (bytesRead < expectedLength && (millis() - startTime) < timeout_ms) {
    if (_serial->available()) {
      response[bytesRead++] = _serial->read();
    }
  }
  
  if (bytesRead < expectedLength) {
    _lastError = ERROR_TIMEOUT;
    return false;
  }
  
  // Verify CRC
  uint16_t receivedCRC = response[expectedLength - 2] | (response[expectedLength - 1] << 8);
  uint16_t calculatedCRC = calculateCRC(response, expectedLength - 2);
  
  if (receivedCRC != calculatedCRC) {
    _lastError = ERROR_CRC;
    return false;
  }
  
  // Check for MODBUS exception
  if (response[1] & 0x80) {
    _lastError = ERROR_EXCEPTION;
    return false;
  }
  
  _lastError = ERROR_NONE;
  return true;
}

float ModbusRTU::registersToFloat(uint16_t lsw, uint16_t msw) {
  // Combine into 32-bit value (LSW first = little-endian word order)
  uint32_t combined = ((uint32_t)msw << 16) | lsw;
  
  // Reinterpret as float
  float result;
  memcpy(&result, &combined, sizeof(float));
  
  return result;
}

void ModbusRTU::enforceDelay() {
  unsigned long now = millis();
  unsigned long elapsed = now - _lastQueryTime;
  
  if (elapsed < _interQueryDelay) {
    delay(_interQueryDelay - elapsed);
  }
}

bool ModbusRTU::readFloat(uint8_t slaveAddress, uint16_t registerAddress, float& value, uint16_t timeout_ms) {
  // Build MODBUS request to read 2 registers (one 32-bit float)
  uint8_t request[8];
  request[0] = slaveAddress;                         // Slave address
  request[1] = FUNCTION_READ_HOLDING_REGISTERS;      // Function code
  request[2] = (registerAddress >> 8) & 0xFF;        // Register address high byte
  request[3] = registerAddress & 0xFF;               // Register address low byte
  request[4] = 0x00;                                 // Number of registers high byte
  request[5] = 0x02;                                 // Number of registers low byte (2 for one float)
  
  sendRequest(request, 6);
  
  // Wait for response
  // Response format: [Address][Function][ByteCount][Data...][CRC_L][CRC_H]
  // For 2 registers: 1 + 1 + 1 + 4 + 2 = 9 bytes
  uint8_t response[9];
  
  if (!receiveResponse(response, 9, timeout_ms)) {
    return false;
  }
  
  // DEBUG: Print raw response bytes
  Serial.print("  [DEBUG] Modbus Response: ");
  for (int i = 0; i < 9; i++) {
    if (response[i] < 0x10) Serial.print("0");
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Extract 32-bit float from response
  // Data starts at byte 3
  // Modbus sends each register as big-endian (MSB first)
  uint16_t lsw = (response[3] << 8) | response[4]; // LSW: byte3=high, byte4=low
  uint16_t msw = (response[5] << 8) | response[6]; // MSW: byte5=high, byte6=low
  
  value = registersToFloat(lsw, msw);
  
  return true;
}

bool ModbusRTU::readMultipleFloats(uint8_t slaveAddress, uint16_t startAddress, uint8_t numFloats, float* values, uint16_t timeout_ms) {
  uint8_t numRegisters = numFloats * 2; // 2 registers per float
  
  // Build MODBUS request
  uint8_t request[8];
  request[0] = slaveAddress;
  request[1] = FUNCTION_READ_HOLDING_REGISTERS;
  request[2] = (startAddress >> 8) & 0xFF;
  request[3] = startAddress & 0xFF;
  request[4] = (numRegisters >> 8) & 0xFF;
  request[5] = numRegisters & 0xFF;
  
  sendRequest(request, 6);
  
  // Calculate expected response length
  // [Address][Function][ByteCount][Data...][CRC_L][CRC_H]
  uint8_t expectedLength = 3 + numRegisters * 2 + 2;
  
  uint8_t response[256]; // Max response size
  
  if (!receiveResponse(response, expectedLength, timeout_ms)) {
    return false;
  }
  
  // Extract floats from response
  // Modbus sends each register as big-endian (MSB first)
  for (uint8_t i = 0; i < numFloats; i++) {
    uint8_t dataOffset = 3 + (i * 4); // Data starts at byte 3, 4 bytes per float
    uint16_t lsw = (response[dataOffset] << 8) | response[dataOffset + 1];
    uint16_t msw = (response[dataOffset + 2] << 8) | response[dataOffset + 3];
    values[i] = registersToFloat(lsw, msw);
  }
  
  return true;
}

bool ModbusRTU::readRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t& value, uint16_t timeout_ms) {
  // Build MODBUS request to read 1 register
  uint8_t request[8];
  request[0] = slaveAddress;
  request[1] = FUNCTION_READ_HOLDING_REGISTERS;
  request[2] = (registerAddress >> 8) & 0xFF;
  request[3] = registerAddress & 0xFF;
  request[4] = 0x00;                                 // Number of registers high byte
  request[5] = 0x01;                                 // Number of registers low byte (1)
  
  sendRequest(request, 6);
  
  // Response format: [Address][Function][ByteCount][Data...][CRC_L][CRC_H]
  // For 1 register: 1 + 1 + 1 + 2 + 2 = 7 bytes
  uint8_t response[7];
  
  if (!receiveResponse(response, 7, timeout_ms)) {
    return false;
  }
  
  // Extract 16-bit value from response (big-endian)
  value = (response[3] << 8) | response[4];
  
  return true;
}

bool ModbusRTU::readMultipleRegisters(uint8_t slaveAddress, uint16_t startAddress, uint8_t numRegisters, uint16_t* values, uint16_t timeout_ms) {
  // Build MODBUS request
  uint8_t request[8];
  request[0] = slaveAddress;
  request[1] = FUNCTION_READ_HOLDING_REGISTERS;
  request[2] = (startAddress >> 8) & 0xFF;
  request[3] = startAddress & 0xFF;
  request[4] = (numRegisters >> 8) & 0xFF;
  request[5] = numRegisters & 0xFF;
  
  sendRequest(request, 6);
  
  // Calculate expected response length
  uint8_t expectedLength = 3 + numRegisters * 2 + 2;
  
  uint8_t response[256];
  
  if (!receiveResponse(response, expectedLength, timeout_ms)) {
    return false;
  }
  
  // Extract registers from response
  for (uint8_t i = 0; i < numRegisters; i++) {
    uint8_t dataOffset = 3 + (i * 2);
    values[i] = (response[dataOffset] << 8) | response[dataOffset + 1];
  }
  
  return true;
}

bool ModbusRTU::writeSingleRegister(uint8_t slaveAddress, uint16_t registerAddress, uint16_t value, uint16_t timeout_ms) {
  // Build MODBUS request (function code 0x06)
  uint8_t request[8];
  request[0] = slaveAddress;
  request[1] = FUNCTION_WRITE_SINGLE_REGISTER;
  request[2] = (registerAddress >> 8) & 0xFF;
  request[3] = registerAddress & 0xFF;
  request[4] = (value >> 8) & 0xFF;
  request[5] = value & 0xFF;
  
  sendRequest(request, 6);
  
  // Response should echo the request
  uint8_t response[8];
  
  if (!receiveResponse(response, 8, timeout_ms)) {
    return false;
  }
  
  // Verify response matches request
  for (uint8_t i = 0; i < 6; i++) {
    if (response[i] != request[i]) {
      _lastError = ERROR_INVALID_LENGTH;
      return false;
    }
  }
  
  return true;
}

bool ModbusRTU::writeMultipleRegisters(uint8_t slaveAddress, uint16_t startAddress, uint8_t numRegisters, uint16_t* values, uint16_t timeout_ms) {
  // Build MODBUS request (function code 0x10)
  uint8_t request[256];
  uint8_t byteCount = numRegisters * 2;
  
  request[0] = slaveAddress;
  request[1] = FUNCTION_WRITE_MULTIPLE_REGISTERS;
  request[2] = (startAddress >> 8) & 0xFF;
  request[3] = startAddress & 0xFF;
  request[4] = (numRegisters >> 8) & 0xFF;
  request[5] = numRegisters & 0xFF;
  request[6] = byteCount;
  
  // Add register values
  for (uint8_t i = 0; i < numRegisters; i++) {
    request[7 + (i * 2)] = (values[i] >> 8) & 0xFF;
    request[8 + (i * 2)] = values[i] & 0xFF;
  }
  
  sendRequest(request, 7 + byteCount);
  
  // Response format: [Address][Function][StartAddr_H][StartAddr_L][NumReg_H][NumReg_L][CRC_L][CRC_H]
  uint8_t response[8];
  
  if (!receiveResponse(response, 8, timeout_ms)) {
    return false;
  }
  
  // Verify response
  if (response[0] != slaveAddress || 
      response[1] != FUNCTION_WRITE_MULTIPLE_REGISTERS) {
    _lastError = ERROR_INVALID_LENGTH;
    return false;
  }
  
  return true;
}