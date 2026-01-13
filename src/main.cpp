/**
 * Multi-Sensor Simulator - Fixed Version
 * 
 * Simulates 3 sensors on shared RS-485 bus:
 * 1. HMP110 - MODBUS RTU at address 0xF0 (humidity & temperature)
 * 2. Young 41342VC #1 - ASCII polling at address 'A' (wind data)
 * 3. Young 41342VC #2 - ASCII polling at address 'B' (wind data)
 * 
 * KEY FIX: Manually handles Modbus requests instead of using ArduinoModbus
 *          library so we can also detect ASCII commands on the same serial line
 */

#include <Arduino.h>

// LED for visual feedback
#define LED_PIN 13

// Sensor addresses
const uint8_t HMP110_SLAVE_ID = 0xF0;    // Modbus address (240)
const char YOUNG1_ADDRESS = 'A';
const char YOUNG2_ADDRESS = 'B';

// Serial configuration
const uint32_t SERIAL_BAUD = 19200;
const uint32_t SERIAL_CONFIG = SERIAL_8N1;

// Modbus register addresses for HMP110
const uint16_t HMP110_RH_ADDR = 0x0000;    // Humidity (2 registers)
const uint16_t HMP110_T_ADDR = 0x0002;     // Temperature (2 registers)

// Simulated sensor data
struct HMP110Data {
  float humidity;     // %RH
  float temperature;  // °C
} hmp110 = {50.5, 25.2};

struct YoungData {
  float windSpeed;       // m/s
  float windDirection;   // degrees
  uint16_t vin1, vin2, vin3, vin4;  // Voltage inputs
} young1 = {5.2, 180.0, 1250, 2350, 0, 0};
struct YoungData young2 = {3.8, 90.0, 1150, 2200, 0, 0};

// Request counters
unsigned long modbusRequestCount = 0;
unsigned long asciiRequestCount = 0;

// ============================================================================
// MODBUS CRC CALCULATION
// ============================================================================
uint16_t calculateModbusCRC(uint8_t* buffer, uint8_t length) {
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

// ============================================================================
// FLOAT TO MODBUS REGISTERS
// ============================================================================
void floatToModbusRegisters(float value, uint16_t& lsw, uint16_t& msw) {
  union {
    float f;
    uint32_t i;
  } converter;
  
  converter.f = value;
  lsw = (uint16_t)(converter.i & 0xFFFF);         // Low word
  msw = (uint16_t)((converter.i >> 16) & 0xFFFF); // High word
}

// ============================================================================
// HANDLE MODBUS REQUEST
// ============================================================================
void handleModbusRequest(uint8_t* request, uint8_t requestLength) {
  // Verify it's for our address
  if (request[0] != HMP110_SLAVE_ID) {
    return;  // Not for us
  }
  
  // Check CRC
  uint16_t receivedCRC = request[requestLength - 2] | (request[requestLength - 1] << 8);
  uint16_t calculatedCRC = calculateModbusCRC(request, requestLength - 2);
  
  if (receivedCRC != calculatedCRC) {
    Serial.println("⚠ Modbus CRC error - ignoring request");
    return;
  }
  
  uint8_t functionCode = request[1];
  
  // We only support function 0x03 (Read Holding Registers)
  if (functionCode != 0x03) {
    Serial.print("⚠ Unsupported Modbus function: 0x");
    Serial.println(functionCode, HEX);
    return;
  }
  
  // Extract register address and count
  uint16_t startAddress = (request[2] << 8) | request[3];
  uint16_t numRegisters = (request[4] << 8) | request[5];
  
  // Build response
  uint8_t response[256];
  uint8_t responseLength = 0;
  
  response[responseLength++] = HMP110_SLAVE_ID;  // Slave address
  response[responseLength++] = 0x03;              // Function code
  response[responseLength++] = numRegisters * 2;  // Byte count
  
  // Add register data
  for (uint16_t i = 0; i < numRegisters; i++) {
    uint16_t regAddress = startAddress + i;
    uint16_t regValue = 0;
    
    // Get register value based on address
    uint16_t lsw_rh, msw_rh, lsw_t, msw_t;
    floatToModbusRegisters(hmp110.humidity, lsw_rh, msw_rh);
    floatToModbusRegisters(hmp110.temperature, lsw_t, msw_t);
    
    if (regAddress == 0x0000) {
      regValue = lsw_rh;
    } else if (regAddress == 0x0001) {
      regValue = msw_rh;
    } else if (regAddress == 0x0002) {
      regValue = lsw_t;
    } else if (regAddress == 0x0003) {
      regValue = msw_t;
    }
    
    // Add register value (big-endian - MSB first)
    response[responseLength++] = (regValue >> 8) & 0xFF;  // High byte
    response[responseLength++] = regValue & 0xFF;         // Low byte
  }
  
  // Add CRC
  uint16_t crc = calculateModbusCRC(response, responseLength);
  response[responseLength++] = crc & 0xFF;        // CRC low byte
  response[responseLength++] = (crc >> 8) & 0xFF; // CRC high byte
  
  // Send response
  Serial1.write(response, responseLength);
  Serial1.flush();
  
  // Log
  Serial.print("📡 Modbus Request #");
  Serial.print(++modbusRequestCount);
  Serial.print(" - HMP110 (0xF0) - RH=");
  Serial.print(hmp110.humidity, 2);
  Serial.print(" %RH, T=");
  Serial.print(hmp110.temperature, 2);
  Serial.println(" °C");
  
  // Flash LED
  digitalWrite(LED_PIN, HIGH);
  delay(30);
  digitalWrite(LED_PIN, LOW);
}

// ============================================================================
// HANDLE ASCII POLL
// ============================================================================
void handleASCIIPoll(char address) {
  YoungData* sensor = nullptr;
  
  // Select which sensor
  if (address == YOUNG1_ADDRESS) {
    sensor = &young1;
    Serial.print("📡 ASCII Poll #");
    Serial.print(++asciiRequestCount);
    Serial.print(" - Young #1 ('A')");
  } else if (address == YOUNG2_ADDRESS) {
    sensor = &young2;
    Serial.print("📡 ASCII Poll #");
    Serial.print(++asciiRequestCount);
    Serial.print(" - Young #2 ('B')");
  } else {
    return;  // Unknown address
  }
  
  // Build response: a,WS,WD,VIN1,VIN2,VIN3,VIN4<CR><LF>
  String response = "";
  response += address;
  response += ",";
  response += String(sensor->windSpeed, 1);
  response += ",";
  response += String(sensor->windDirection, 1);
  response += ",";
  response += String(sensor->vin1);
  response += ",";
  response += String(sensor->vin2);
  response += ",";
  response += String(sensor->vin3);
  response += ",";
  response += String(sensor->vin4);
  response += "\r\n";
  
  // Send response
  Serial1.print(response);
  Serial1.flush();
  
  // Log
  Serial.print(" - WS=");
  Serial.print(sensor->windSpeed, 1);
  Serial.print(" m/s, WD=");
  Serial.print(sensor->windDirection, 1);
  Serial.println("°");
  
  // Flash LED
  digitalWrite(LED_PIN, HIGH);
  delay(30);
  digitalWrite(LED_PIN, LOW);
}

// ============================================================================
// CHECK INCOMING DATA
// ============================================================================
void checkIncomingData() {
  static uint8_t buffer[256];
  static uint8_t bufferIndex = 0;
  static unsigned long lastByteTime = 0;
  
  // Read available bytes
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    unsigned long now = millis();
    
    // Reset buffer if gap > 10ms (new message)
    if (bufferIndex > 0 && (now - lastByteTime) > 10) {
      bufferIndex = 0;
    }
    
    lastByteTime = now;
    buffer[bufferIndex++] = b;
    
    // Check for ASCII command: Ma! (3 bytes)
    if (bufferIndex >= 3 && buffer[0] == 'M' && buffer[2] == '!') {
      char address = (char)buffer[1];
      handleASCIIPoll(address);
      bufferIndex = 0;
      return;
    }
    
    // Check for potential Modbus request (minimum 8 bytes)
    if (bufferIndex >= 8) {
      // Modbus request format: [Addr][Func][Data...][CRC_L][CRC_H]
      // Read Holding Registers: [Addr][0x03][Start_H][Start_L][Count_H][Count_L][CRC_L][CRC_H]
      if (buffer[0] == HMP110_SLAVE_ID && buffer[1] == 0x03) {
        if (bufferIndex == 8) {
          handleModbusRequest(buffer, bufferIndex);
          bufferIndex = 0;
          return;
        }
      }
    }
    
    // Prevent buffer overflow
    if (bufferIndex >= 250) {
      bufferIndex = 0;
    }
  }
}

// ============================================================================
// UPDATE SIMULATED VALUES
// ============================================================================
void updateSimulatedValues() {
  // HMP110: Simulate realistic drift
  hmp110.humidity += (random(-10, 11) / 10.0);
  hmp110.temperature += (random(-5, 6) / 10.0);
  hmp110.humidity = constrain(hmp110.humidity, 30.0, 70.0);
  hmp110.temperature = constrain(hmp110.temperature, 20.0, 30.0);
  
  // Young #1: Simulate wind changes
  young1.windSpeed += (random(-20, 21) / 10.0);
  young1.windDirection += (random(-10, 11));
  young1.windSpeed = constrain(young1.windSpeed, 0.0, 20.0);
  if (young1.windDirection < 0) young1.windDirection += 360;
  if (young1.windDirection >= 360) young1.windDirection -= 360;
  
  // Young #2: Simulate wind changes
  young2.windSpeed += (random(-15, 16) / 10.0);
  young2.windDirection += (random(-15, 16));
  young2.windSpeed = constrain(young2.windSpeed, 0.0, 15.0);
  if (young2.windDirection < 0) young2.windDirection += 360;
  if (young2.windDirection >= 360) young2.windDirection -= 360;
  
  // Update voltage inputs
  young1.vin1 = constrain(young1.vin1 + random(-50, 51), 1000, 1500);
  young1.vin2 = constrain(young1.vin2 + random(-50, 51), 2000, 2500);
  young2.vin1 = constrain(young2.vin1 + random(-30, 31), 1000, 1300);
  young2.vin2 = constrain(young2.vin2 + random(-30, 31), 2100, 2400);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  // USB Serial for monitoring
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  // RS-485 Serial
  Serial1.begin(SERIAL_BAUD, SERIAL_CONFIG);
  
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║   MULTI-SENSOR SIMULATOR (FIXED)               ║");
  Serial.println("║   Modbus RTU + ASCII Polling on ONE bus        ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.println();
  
  Serial.println("Serial Configuration:");
  Serial.println("  Port: Serial1 (TX=1, RX=0)");
  Serial.println("  Baud: 19200");
  Serial.println("  Config: 8N1");
  Serial.println("  Connection: TX1->RX(master), RX1->TX(master)");
  Serial.println();
  
  Serial.println("Simulated Sensors:");
  Serial.println("  HMP110:   Modbus addr 0xF0 - BINARY MODBUS");
  Serial.println("  Young #1: ASCII addr 'A'  - ASCII POLLING");
  Serial.println("  Young #2: ASCII addr 'B'  - ASCII POLLING");
  Serial.println();
  
  Serial.println("Initial Values:");
  Serial.print("  HMP110:   RH=");
  Serial.print(hmp110.humidity, 2);
  Serial.print(" %RH, T=");
  Serial.print(hmp110.temperature, 2);
  Serial.println(" °C");
  Serial.print("  Young #1: WS=");
  Serial.print(young1.windSpeed, 1);
  Serial.print(" m/s, WD=");
  Serial.print(young1.windDirection, 1);
  Serial.println("°");
  Serial.print("  Young #2: WS=");
  Serial.print(young2.windSpeed, 1);
  Serial.print(" m/s, WD=");
  Serial.print(young2.windDirection, 1);
  Serial.println("°");
  Serial.println();
  
  Serial.println("════════════════════════════════════════════════");
  Serial.println("✓ Ready! Listening for requests...");
  Serial.println("════════════════════════════════════════════════");
  Serial.println();
  
  // Flash LED to indicate ready
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// ============================================================================
// LOOP
// ============================================================================
unsigned long lastUpdate = 0;
unsigned long lastHeartbeat = 0;

void loop() {
  // Check for incoming Modbus or ASCII requests
  checkIncomingData();
  
  // Update simulated values every 2 seconds
  if (millis() - lastUpdate >= 2000) {
    lastUpdate = millis();
    updateSimulatedValues();
  }
  
  // Heartbeat every 10 seconds
  if (millis() - lastHeartbeat >= 10000) {
    lastHeartbeat = millis();
    Serial.println();
    Serial.println("──────────── HEARTBEAT ────────────");
    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
    Serial.print("Requests: Modbus=");
    Serial.print(modbusRequestCount);
    Serial.print(", ASCII=");
    Serial.println(asciiRequestCount);
    Serial.print("Current: RH=");
    Serial.print(hmp110.humidity, 1);
    Serial.print("%, T=");
    Serial.print(hmp110.temperature, 1);
    Serial.print("C, WS1=");
    Serial.print(young1.windSpeed, 1);
    Serial.print("m/s, WS2=");
    Serial.print(young2.windSpeed, 1);
    Serial.println("m/s");
    Serial.println("───────────────────────────────────");
    Serial.println();
  }
}