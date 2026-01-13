// Helper class for Young 41342VC Temperature Sensor (Modified for Temperature Only)
// Provides convenient methods to read temperature data via ASCII polling over RS-485
// Polling command: Ma! where 'a' is the address (A-Z, 0-9)
// Response format: a,T,VIN1,VIN2,VIN3,VIN4<CR><LF>

#ifndef YOUNG_41342VC_H
#define YOUNG_41342VC_H

#include <Arduino.h>

class Young41342VC {
public:
  Young41342VC(Stream* serial, char address = 'A');
  
  // Read all sensor data by polling
  // Returns temperature (°C) and voltage inputs
  bool readAllData(float& temperature, 
                   uint16_t& vin1, uint16_t& vin2, uint16_t& vin3, uint16_t& vin4,
                   uint16_t timeout_ms = 1000);
  
  // Simplified read - just temperature
  bool readTemperature(float& temperature, uint16_t timeout_ms = 1000);
  
  // Get last error code
  uint8_t getLastError() { return _lastError; }
  
  // Get address character
  char getAddress() { return _address; }
  
  // Set address character (A-Z, 0-9)
  void setAddress(char address) { _address = address; }
  
private:
  Stream* _serial;
  char _address;      // Polling address (A-Z, 0-9)
  uint8_t _lastError;
  
  // Error codes
  static const uint8_t ERROR_NONE = 0x00;
  static const uint8_t ERROR_TIMEOUT = 0xE0;
  static const uint8_t ERROR_INVALID_RESPONSE = 0xE1;
  static const uint8_t ERROR_PARSE = 0xE2;
  
  // Send polling command: Ma!
  void sendPollCommand();
  
  // Receive and parse ASCII response
  bool receiveResponse(char* buffer, size_t bufferSize, uint16_t timeout_ms);
  
  // Parse the ASCII response string
  bool parseResponse(const char* response, float& temperature,
                     uint16_t& vin1, uint16_t& vin2, uint16_t& vin3, uint16_t& vin4);
};

#endif // YOUNG_41342VC_H