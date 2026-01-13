/**
 * Master Test Program for Multi-Sensor Simulator
 * 
 * Tests all three sensors:
 * - HMP110 at Modbus address 0xF0 (Binary Modbus RTU)
 * - Young #1 at ASCII address 'A' (ASCII polling)
 * - Young #2 at ASCII address 'B' (ASCII polling)
 * 
 * Connect to simulator:
 *   TX2 (pin 11) -> RX1 of simulator
 *   RX2 (pin 12) -> TX1 of simulator
 *   GND -> GND
 */

#include <Arduino.h>
#include "wiring_private.h"  // For pinPeripheral()
#include "ModbusRTU.h"
#include "HMP110.h"
#include "Young41342VC.h"

// Create Serial2 on SERCOM1 (pins 11/12)
Uart Serial2(&sercom1, 12, 11, SERCOM_RX_PAD_3, UART_TX_PAD_0);

// Interrupt handler for SERCOM1
void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

// SENSOR ADDRESSES
const uint8_t HMP110_ADDRESS = 0xF0;    // HMP110 Modbus address (240)
const char YOUNG1_ADDRESS = 'A';        // Young sensor #1 ASCII address
const char YOUNG2_ADDRESS = 'B';        // Young sensor #2 ASCII address

// Sensor data structures
struct HMP110Data {
  bool valid;
  float humidity;
  float temperature;
  unsigned long lastReadTime;
} hmp110;

struct YoungData {
  bool valid;
  float windSpeed;
  float windDirection;
  uint16_t vin1, vin2, vin3, vin4;
  unsigned long lastReadTime;
} young1, young2;

// Create MODBUS RTU instance for HMP110
ModbusRTU modbus(&Serial2);

// Create sensor wrapper objects
HMP110 hmp110Sensor(&modbus, HMP110_ADDRESS);
Young41342VC young1Sensor(&Serial2, YOUNG1_ADDRESS);
Young41342VC young2Sensor(&Serial2, YOUNG2_ADDRESS);

// Timing
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 2000;  // Read all sensors every 2 seconds

// Forward declarations
void readAllSensors();
void displayAllData();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║   MULTI-SENSOR TEST PROGRAM (MASTER)           ║");
  Serial.println("║   Reading from Simulator                       ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize Serial2 for RS-485 communication
  Serial2.begin(19200, SERIAL_8N1);
  pinPeripheral(11, PIO_SERCOM);  // TX
  pinPeripheral(12, PIO_SERCOM);  // RX
  
  Serial.println("Serial2 initialized:");
  Serial.println("  Pins: 11(TX) / 12(RX)");
  Serial.println("  Baud: 19200");
  Serial.println("  Config: 8N1");
  Serial.println();
  
  // Configure inter-query delay for Modbus
  modbus.setInterQueryDelay(100);
  
  Serial.println("Sensor Configuration:");
  Serial.println("  HMP110:   Modbus addr 0xF0 - BINARY MODBUS");
  Serial.println("  Young #1: ASCII addr 'A'  - ASCII POLLING");
  Serial.println("  Young #2: ASCII addr 'B'  - ASCII POLLING");
  Serial.println();
  
  Serial.println("Connection to Simulator:");
  Serial.println("  TX2 (pin 11) -> RX1 of simulator");
  Serial.println("  RX2 (pin 12) -> TX1 of simulator");
  Serial.println("  GND -> GND");
  Serial.println();
  
  // Initialize data structures
  hmp110.valid = false;
  young1.valid = false;
  young2.valid = false;
  
  Serial.println("════════════════════════════════════════════════");
  Serial.println("✓ Starting sensor reads in 2 seconds...");
  Serial.println("════════════════════════════════════════════════");
  Serial.println();
  
  delay(2000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read all sensors periodically
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    readAllSensors();
    displayAllData();
    lastReadTime = currentTime;
  }
}

void readAllSensors() {
  Serial.println("\n──────────── READING SENSORS ────────────");
  
  // Read HMP110 (humidity and temperature via MODBUS)
  Serial.print("Reading HMP110 (Modbus 0xF0)...");
  if (hmp110Sensor.readAllData(hmp110.humidity, hmp110.temperature)) {
    hmp110.valid = true;
    hmp110.lastReadTime = millis();
    Serial.println(" ✓ SUCCESS");
  } else {
    hmp110.valid = false;
    Serial.print(" ✗ FAILED - Error: 0x");
    Serial.println(hmp110Sensor.getLastError(), HEX);
  }
  
  // IMPORTANT: Longer delay between Modbus and ASCII protocols
  // This ensures the simulator has time to process the Modbus response
  // and clear its serial buffer before ASCII polling starts
  delay(250);
  
  // Clear any residual data from serial buffer before ASCII polling
  while (Serial2.available()) {
    Serial2.read();
  }
  
  // Read Young sensor #1 (wind data via ASCII polling)
  Serial.print("Reading Young #1 (ASCII 'A')...");
  if (young1Sensor.readAllData(young1.windSpeed, young1.windDirection,
                                young1.vin1, young1.vin2, young1.vin3, young1.vin4)) {
    young1.valid = true;
    young1.lastReadTime = millis();
    Serial.println(" ✓ SUCCESS");
  } else {
    young1.valid = false;
    Serial.print(" ✗ FAILED - Error: 0x");
    Serial.println(young1Sensor.getLastError(), HEX);
  }
  
  delay(150);
  
  // Read Young sensor #2 (wind data via ASCII polling)
  Serial.print("Reading Young #2 (ASCII 'B')...");
  if (young2Sensor.readAllData(young2.windSpeed, young2.windDirection,
                                young2.vin1, young2.vin2, young2.vin3, young2.vin4)) {
    young2.valid = true;
    young2.lastReadTime = millis();
    Serial.println(" ✓ SUCCESS");
  } else {
    young2.valid = false;
    Serial.print(" ✗ FAILED - Error: 0x");
    Serial.println(young2Sensor.getLastError(), HEX);
  }
  
  Serial.println("──────────────────────────────────────────");
}

void displayAllData() {
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║             SENSOR DATA                        ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  
  // HMP110 Data (from Modbus)
  Serial.println("\n┌─ HMP110 (Humidity/Temperature) [MODBUS] ──────┐");
  if (hmp110.valid) {
    Serial.print("│  Humidity:    ");
    Serial.print(hmp110.humidity, 2);
    Serial.println(" %RH");
    Serial.print("│  Temperature: ");
    Serial.print(hmp110.temperature, 2);
    Serial.println(" °C");
  } else {
    Serial.println("│  ⚠ NO DATA");
  }
  Serial.println("└────────────────────────────────────────────────┘");
  
  // Young Sensor #1 Data (from ASCII polling)
  Serial.println("\n┌─ Young #1 (Wind Sensor) [ASCII 'A'] ──────────┐");
  if (young1.valid) {
    Serial.print("│  Wind Speed:     ");
    Serial.print(young1.windSpeed, 2);
    Serial.println(" m/s");
    Serial.print("│  Wind Direction: ");
    Serial.print(young1.windDirection, 1);
    Serial.println(" degrees");
    Serial.print("│  VIN1: ");
    Serial.print(young1.vin1);
    Serial.print("  VIN2: ");
    Serial.print(young1.vin2);
    Serial.print("  VIN3: ");
    Serial.print(young1.vin3);
    Serial.print("  VIN4: ");
    Serial.println(young1.vin4);
  } else {
    Serial.println("│  ⚠ NO DATA");
  }
  Serial.println("└────────────────────────────────────────────────┘");
  
  // Young Sensor #2 Data (from ASCII polling)
  Serial.println("\n┌─ Young #2 (Wind Sensor) [ASCII 'B'] ──────────┐");
  if (young2.valid) {
    Serial.print("│  Wind Speed:     ");
    Serial.print(young2.windSpeed, 2);
    Serial.println(" m/s");
    Serial.print("│  Wind Direction: ");
    Serial.print(young2.windDirection, 1);
    Serial.println(" degrees");
    Serial.print("│  VIN1: ");
    Serial.print(young2.vin1);
    Serial.print("  VIN2: ");
    Serial.print(young2.vin2);
    Serial.print("  VIN3: ");
    Serial.print(young2.vin3);
    Serial.print("  VIN4: ");
    Serial.println(young2.vin4);
  } else {
    Serial.println("│  ⚠ NO DATA");
  }
  Serial.println("└────────────────────────────────────────────────┘");
  
  // Summary
  int validCount = (hmp110.valid ? 1 : 0) + (young1.valid ? 1 : 0) + (young2.valid ? 1 : 0);
  Serial.println();
  Serial.print("✓ Successfully read ");
  Serial.print(validCount);
  Serial.println(" of 3 sensors");
  Serial.println();
}