#include "Young41342VC.h"

Young41342VC::Young41342VC(Stream* serial, char address)
  : _serial(serial), _address(address), _lastError(ERROR_NONE) {
}

void Young41342VC::sendPollCommand() {
  // Clear any pending data in receive buffer
  while (_serial->available()) {
    _serial->read();
  }
  
  // Send polling command: Ma!
  // Example: MA! for address 'A', MB! for address 'B'
  _serial->print('M');
  _serial->print(_address);
  _serial->print('!');
  _serial->flush();
}

bool Young41342VC::receiveResponse(char* buffer, size_t bufferSize, uint16_t timeout_ms) {
  unsigned long startTime = millis();
  size_t index = 0;
  bool foundCR = false;
  
  // Read until we get <CR> or timeout
  while ((millis() - startTime) < timeout_ms) {
    if (_serial->available()) {
      char c = _serial->read();
      
      // Check for carriage return (end of message)
      if (c == '\r' || c == '\n') {
        if (index > 0) {  // Only mark as found if we have data
          foundCR = true;
          break;
        }
        continue;  // Skip leading CR/LF
      }
      
      // Store character if buffer has space
      if (index < bufferSize - 1) {
        buffer[index++] = c;
      } else {
        // Buffer overflow
        _lastError = ERROR_INVALID_RESPONSE;
        return false;
      }
    }
  }
  
  // Null terminate the string
  buffer[index] = '\0';
  
  if (!foundCR && index == 0) {
    _lastError = ERROR_TIMEOUT;
    return false;
  }
  
  _lastError = ERROR_NONE;
  return true;
}

bool Young41342VC::parseResponse(const char* response, float& temperature,
                                  uint16_t& vin1, uint16_t& vin2, uint16_t& vin3, uint16_t& vin4) {
  // Expected format: a,T,VIN1,VIN2,VIN3,VIN4
  // Example: A,22.50,1234,2345,3456,4000
  
  // Verify first character matches our address
  if (response[0] != _address) {
    _lastError = ERROR_INVALID_RESPONSE;
    return false;
  }
  
  // Parse comma-separated values
  // Skip first character (address) and comma
  const char* ptr = response + 1;
  if (*ptr == ',') ptr++;  // Skip comma after address
  
  // Parse temperature
  temperature = atof(ptr);
  
  // Find next comma
  ptr = strchr(ptr, ',');
  if (!ptr) {
    _lastError = ERROR_PARSE;
    return false;
  }
  ptr++;  // Skip comma
  
  // Parse VIN1
  vin1 = atoi(ptr);
  
  // Find next comma
  ptr = strchr(ptr, ',');
  if (!ptr) {
    _lastError = ERROR_PARSE;
    return false;
  }
  ptr++;  // Skip comma
  
  // Parse VIN2
  vin2 = atoi(ptr);
  
  // Find next comma
  ptr = strchr(ptr, ',');
  if (!ptr) {
    _lastError = ERROR_PARSE;
    return false;
  }
  ptr++;  // Skip comma
  
  // Parse VIN3
  vin3 = atoi(ptr);
  
  // Find next comma
  ptr = strchr(ptr, ',');
  if (!ptr) {
    _lastError = ERROR_PARSE;
    return false;
  }
  ptr++;  // Skip comma
  
  // Parse VIN4
  vin4 = atoi(ptr);
  
  _lastError = ERROR_NONE;
  return true;
}

bool Young41342VC::readAllData(float& temperature,
                                uint16_t& vin1, uint16_t& vin2, uint16_t& vin3, uint16_t& vin4,
                                uint16_t timeout_ms) {
  // Send polling command
  sendPollCommand();
  
  // Wait for 32400 turnaround (2ms minimum per manual)
  delay(5);
  
  // Receive response
  char buffer[128];
  if (!receiveResponse(buffer, sizeof(buffer), timeout_ms)) {
    return false;
  }
  
  // Parse response
  return parseResponse(buffer, temperature, vin1, vin2, vin3, vin4);
}

bool Young41342VC::readTemperature(float& temperature, uint16_t timeout_ms) {
  // Simplified version - just get temperature, ignore voltage inputs
  uint16_t vin1, vin2, vin3, vin4;
  return readAllData(temperature, vin1, vin2, vin3, vin4, timeout_ms);
}