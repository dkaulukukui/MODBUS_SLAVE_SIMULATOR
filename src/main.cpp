/**
 * MODBUS RTU Slave Simulator for Adafruit Feather M0 Express
 * 
 * This project implements a MODBUS RTU slave device that responds to all
 * supported MODBUS functions from the modbus-arduino library.
 * 
 * Supported MODBUS Functions:
 * - 0x01: Read Coils
 * - 0x02: Read Discrete Inputs
 * - 0x03: Read Holding Registers
 * - 0x04: Read Input Registers
 * - 0x05: Write Single Coil
 * - 0x06: Write Single Register
 * - 0x0F: Write Multiple Coils
 * - 0x10: Write Multiple Registers
 * - 0x11: Report Server ID
 * 
 * Hardware: Adafruit Feather M0 Express
 * Library: modbus-arduino by epsilonrt
 */

#include <Arduino.h>
#include <ModbusSerial.h>

// MODBUS Configuration
const int SLAVE_ID = 1;           // Modbus Slave ID
const long SERIAL_BAUD = 9600;    // Serial baud rate
const int SERIAL_CONFIG = SERIAL_8E1;  // 8 data bits, Even parity, 1 stop bit
const int TXEN_PIN = -1;          // Transmit enable pin for RS485 (-1 if not used)

// Create Modbus Serial instance
ModbusSerial mb;

// Define the number of each register type
const int NUM_COILS = 10;              // Digital outputs (read/write)
const int NUM_DISCRETE_INPUTS = 10;    // Digital inputs (read only)
const int NUM_HOLDING_REGISTERS = 10;  // Analog outputs/storage (read/write)
const int NUM_INPUT_REGISTERS = 10;    // Analog inputs (read only)

// Simulated data arrays
bool coilData[NUM_COILS];
bool discreteInputData[NUM_DISCRETE_INPUTS];
uint16_t holdingRegisterData[NUM_HOLDING_REGISTERS];
uint16_t inputRegisterData[NUM_INPUT_REGISTERS];

// LED pin for visual feedback
const int LED_PIN = 13;
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 1000; // Update simulated data every second

/**
 * Initialize simulated data with meaningful values
 */
void initializeSimulatedData() {
  // Initialize coils (digital outputs)
  for (int i = 0; i < NUM_COILS; i++) {
    coilData[i] = (i % 2 == 0); // Alternate pattern
  }
  
  // Initialize discrete inputs (digital inputs)
  for (int i = 0; i < NUM_DISCRETE_INPUTS; i++) {
    discreteInputData[i] = (i % 3 == 0); // Different pattern
  }
  
  // Initialize holding registers (analog outputs/storage)
  for (int i = 0; i < NUM_HOLDING_REGISTERS; i++) {
    holdingRegisterData[i] = 1000 + (i * 100); // 1000, 1100, 1200, etc.
  }
  
  // Initialize input registers (analog inputs - simulated sensor data)
  for (int i = 0; i < NUM_INPUT_REGISTERS; i++) {
    inputRegisterData[i] = 2000 + (i * 50); // 2000, 2050, 2100, etc.
  }
}

/**
 * Update simulated sensor data (input registers and discrete inputs)
 * This simulates changing sensor readings over time
 */
void updateSimulatedData() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentTime;
    
    // Simulate changing input register values (e.g., temperature, humidity sensors)
    for (int i = 0; i < NUM_INPUT_REGISTERS; i++) {
      // Add small random variation to simulate real sensors
      int variation = random(-10, 11);
      inputRegisterData[i] = constrain(inputRegisterData[i] + variation, 0, 65535);
      mb.Ireg(i, inputRegisterData[i]);
    }
    
    // Simulate changing discrete input states (e.g., switches, buttons)
    for (int i = 0; i < NUM_DISCRETE_INPUTS; i++) {
      // Occasionally toggle discrete inputs
      if (random(100) < 10) { // 10% chance to toggle
        discreteInputData[i] = !discreteInputData[i];
        mb.Ists(i, discreteInputData[i]);
      }
    }
    
    // Toggle LED to show activity
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void setup() {
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Seed random number generator for sensor simulation
  randomSeed(analogRead(0));
  
  // Initialize serial communication for MODBUS
  Serial.begin(SERIAL_BAUD, SERIAL_CONFIG);
  
  // Wait for serial port to be ready
  delay(100);
  
  // Configure MODBUS slave
  mb.config(&Serial, SERIAL_BAUD, SERIAL_CONFIG, TXEN_PIN);
  mb.setSlaveId(SLAVE_ID);
  
  // Initialize simulated data
  initializeSimulatedData();
  
  // Add Coils (Function codes 0x01, 0x05, 0x0F)
  // Coils are digital outputs that can be read and written
  for (int i = 0; i < NUM_COILS; i++) {
    mb.addCoil(i, coilData[i]);
  }
  
  // Add Discrete Inputs (Function code 0x02)
  // Discrete inputs are digital inputs that are read-only
  for (int i = 0; i < NUM_DISCRETE_INPUTS; i++) {
    mb.addIsts(i, discreteInputData[i]);
  }
  
  // Add Holding Registers (Function codes 0x03, 0x06, 0x10)
  // Holding registers are 16-bit registers that can be read and written
  for (int i = 0; i < NUM_HOLDING_REGISTERS; i++) {
    mb.addHreg(i, holdingRegisterData[i]);
  }
  
  // Add Input Registers (Function code 0x04)
  // Input registers are 16-bit read-only registers
  for (int i = 0; i < NUM_INPUT_REGISTERS; i++) {
    mb.addIreg(i, inputRegisterData[i]);
  }
  
  // Flash LED three times to indicate successful initialization
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  // Note: Function 0x11 (Report Server ID) is automatically supported
  // by the modbus-arduino library when configured
}

void loop() {
  // Process MODBUS requests
  // This must be called regularly to handle incoming requests
  mb.task();
  
  // Update simulated sensor data periodically
  updateSimulatedData();
  
  // Optional: Sync any changed coil or holding register values back to arrays
  // This would be needed if you want to act on changes made via MODBUS
  for (int i = 0; i < NUM_COILS; i++) {
    coilData[i] = mb.Coil(i);
  }
  
  for (int i = 0; i < NUM_HOLDING_REGISTERS; i++) {
    holdingRegisterData[i] = mb.Hreg(i);
  }
}
