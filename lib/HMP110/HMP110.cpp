#include "HMP110.h"

HMP110::HMP110(ModbusRTU* modbus, uint8_t slaveAddress)
  : _modbus(modbus), _slaveAddress(slaveAddress) {
}

bool HMP110::readAllData(float& humidity, float& temperature, uint16_t timeout_ms) {
  // Read both humidity and temperature in one transaction (most efficient)
  // This reads 4 registers starting from REG_HUMIDITY (2 floats)
  float values[2];
  
  if (_modbus->readMultipleFloats(_slaveAddress, REG_HUMIDITY, 2, values, timeout_ms)) {
    humidity = values[0];
    temperature = values[1];
    return true;
  }
  
  return false;
}

bool HMP110::readHumidity(float& humidity, uint16_t timeout_ms) {
  return _modbus->readFloat(_slaveAddress, REG_HUMIDITY, humidity, timeout_ms);
}

bool HMP110::readTemperature(float& temperature, uint16_t timeout_ms) {
  return _modbus->readFloat(_slaveAddress, REG_TEMPERATURE, temperature, timeout_ms);
}

bool HMP110::readDewPoint(float& dewPoint, uint16_t timeout_ms) {
  return _modbus->readFloat(_slaveAddress, REG_DEW_POINT, dewPoint, timeout_ms);
}
