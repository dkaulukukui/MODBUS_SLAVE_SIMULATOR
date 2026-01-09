/**
 * HMP110 MODBUS RTU Slave Simulator
 * Direct UART Connection (No RS485)
 * 
 * Connect TX1 -> RX of master
 *        RX1 -> TX of master
 *        GND -> GND
 */

#include <Arduino.h>
#include <ArduinoModbus.h>

// ============================================================================
// HMP110 MODBUS Configuration
// ============================================================================
const int SLAVE_ID = 240;                    // HMP110 default address (0xF0)
const long SERIAL_BAUD = 115200;              // HMP110 baud rate
const uint16_t SERIAL_CONFIG = SERIAL_8N2;   // 8 data, No parity, 2 stop bits

const int LED_PIN = 13;

// Register addresses
const int HMP110_RH_ADDR = 0x0000;     // RH: registers 0x0000-0x0001
const int HMP110_T_ADDR = 0x0002;      // T:  registers 0x0002-0x0003
const int NUM_HOLDING_REGISTERS = 4;

// Simulated sensor values
float currentHumidity = 50.0;
float currentTemperature = 25.0;

// ============================================================================
// IEEE 754 Float Encoding (Little-Endian)
// ============================================================================
void floatToModbusRegisters(float value, uint16_t &lsw, uint16_t &msw) {
  union {
    float f;
    uint32_t i;
  } converter;
  
  converter.f = value;
  lsw = (uint16_t)(converter.i & 0xFFFF);         // Low word first (little-endian)
  msw = (uint16_t)((converter.i >> 16) & 0xFFFF); // High word second
}

// ============================================================================
// Update HMP110 Registers
// ============================================================================
void updateHMP110Registers() {
  uint16_t lsw, msw;
  
  // Update RH registers (0x0000-0x0001)
  floatToModbusRegisters(currentHumidity, lsw, msw);
  ModbusRTUServer.holdingRegisterWrite(HMP110_RH_ADDR, lsw);
  ModbusRTUServer.holdingRegisterWrite(HMP110_RH_ADDR + 1, msw);
  
  // Update Temperature registers (0x0002-0x0003)
  floatToModbusRegisters(currentTemperature, lsw, msw);
  ModbusRTUServer.holdingRegisterWrite(HMP110_T_ADDR, lsw);
  ModbusRTUServer.holdingRegisterWrite(HMP110_T_ADDR + 1, msw);
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  // USB Serial for monitoring
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.println("║   HMP110 MODBUS RTU SLAVE (Direct UART)      ║");
  Serial.println("╚═══════════════════════════════════════════════╝");
  Serial.println();
  Serial.print("Slave Address: ");
  Serial.print(SLAVE_ID);
  Serial.println(" (0xF0)");
  Serial.print("Baud Rate: ");
  Serial.println(SERIAL_BAUD);
  Serial.println("Config: 8N2");
  Serial.println("Connection: TX1->RX(master), RX1->TX(master)");
  Serial.println();
  
  // Start MODBUS RTU Server on Serial1
  Serial.println("Starting MODBUS RTU Server on Serial1...");
  
  if (!ModbusRTUServer.begin(SLAVE_ID, SERIAL_BAUD, SERIAL_CONFIG)) {
    Serial.println("✗ Failed to start MODBUS server!");
    while(1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(100);
    }
  }
  
  Serial.println("✓ MODBUS server started");
  
  // Configure holding registers
  ModbusRTUServer.configureHoldingRegisters(0x0000, NUM_HOLDING_REGISTERS);
  
  // Set initial values
  updateHMP110Registers();
  
  Serial.println();
  Serial.println("Register Map:");
  Serial.println("  0x0000-0x0001: Humidity (32-bit float, little-endian)");
  Serial.println("  0x0002-0x0003: Temperature (32-bit float, little-endian)");
  Serial.println();
  Serial.print("Initial: RH=");
  Serial.print(currentHumidity, 2);
  Serial.print(" %RH, T=");
  Serial.print(currentTemperature, 2);
  Serial.println(" °C");
  Serial.println();
  Serial.println("═══════════════════════════════════════════════");
  Serial.println("✓ Ready! Listening for MODBUS requests...");
  Serial.println("═══════════════════════════════════════════════");
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
// Loop
// ============================================================================
unsigned long lastUpdate = 0;
unsigned long lastHeartbeat = 0;
int requestCount = 0;

void loop() {
  // Poll for MODBUS requests
  int pollResult = ModbusRTUServer.poll();
  
  // Show when we get a request
  if (pollResult > 0) {
    requestCount++;
    Serial.print("📡 Request #");
    Serial.print(requestCount);
    Serial.print(" @ ");
    Serial.print(millis());
    Serial.print(" ms - RH=");
    Serial.print(currentHumidity, 2);
    Serial.print(" %RH, T=");
    Serial.print(currentTemperature, 2);
    Serial.println(" °C");
    
    // Flash LED
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }
  
  // Update simulated values every 2 seconds
  if (millis() - lastUpdate >= 2000) {
    lastUpdate = millis();
    
    // Simulate realistic drift
    currentHumidity += (random(-10, 11) / 10.0);
    currentTemperature += (random(-5, 6) / 10.0);
    
    // Keep in range
    currentHumidity = constrain(currentHumidity, 30.0, 70.0);
    currentTemperature = constrain(currentTemperature, 20.0, 30.0);
    
    // Update registers
    updateHMP110Registers();
  }
  
  // Heartbeat every 10 seconds to show we're alive
  if (millis() - lastHeartbeat >= 10000) {
    lastHeartbeat = millis();
    Serial.print("Heartbeat - Uptime: ");
    Serial.print(millis() / 1000);
    Serial.print("s, Requests: ");
    Serial.print(requestCount);
    Serial.print(", RH=");
    Serial.print(currentHumidity, 2);
    Serial.print(" %RH, T=");
    Serial.print(currentTemperature, 2);
    Serial.println(" °C");
  }
}