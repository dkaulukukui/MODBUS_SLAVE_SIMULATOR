/**
 * MODBUS RTU Slave Simulator for Adafruit Feather M0 Express
 * 
 * This project implements a MODBUS RTU slave device that responds to all
 * supported MODBUS functions from the modbus-arduino library.
 * 
 * Features:
 * - Full MODBUS RTU Slave implementation
 * - RS-485 Bus Monitor: Captures and displays ALL data on the bus
 * - Monitors both Modbus and non-Modbus data (e.g., polled ASCII)
 * - Echoes all bus traffic to USB Serial for debugging
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
 * Hardware Connections:
 * - Serial (USB): Debug/monitoring output at 115200 baud
 * - Serial1 (TX/RX pins): RS-485 connection at 9600 baud, 8E1
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

// Bus Monitoring Configuration
const bool BUS_MONITOR_ENABLED = true;  // Enable RS-485 bus monitoring
const int MAX_FRAME_SIZE = 256;         // Maximum size of captured frame
const unsigned long FRAME_TIMEOUT = 10; // Timeout in ms to detect end of frame

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
 * Print a byte array in hex format to USB serial
 */
void printHexData(const byte* data, int length, const char* prefix = "[RS-485]") {
  Serial.print(prefix);
  Serial.print(" ");
  Serial.print(millis());
  Serial.print(" ms | ");
  Serial.print(length);
  Serial.print(" bytes: ");
  
  for (int i = 0; i < length; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  
  // Also print ASCII representation
  Serial.print(" | ASCII: ");
  for (int i = 0; i < length; i++) {
    if (data[i] >= 32 && data[i] <= 126) {
      Serial.write(data[i]);
    } else {
      Serial.print(".");
    }
  }
  Serial.println();
}

/**
 * MonitoredSerial - A Stream wrapper that logs all reads to USB Serial
 * This class wraps Serial1 and logs all data read from the RS-485 bus
 */
class MonitoredSerial : public Stream {
private:
  Stream* _serial;
  byte _frameBuffer[MAX_FRAME_SIZE];
  int _frameLength;
  unsigned long _lastByteTime;
  byte _txFrameBuffer[MAX_FRAME_SIZE];
  int _txFrameLength;
  unsigned long _lastTxByteTime;
  
  void checkFrameTimeout() {
    unsigned long currentTime = millis();
    if (_frameLength > 0 && (currentTime - _lastByteTime) >= FRAME_TIMEOUT) {
      // RX Frame complete, print it
      if (BUS_MONITOR_ENABLED) {
        printHexData(_frameBuffer, _frameLength);
      }
      _frameLength = 0;
    }
    
    // Also check TX frame timeout
    if (_txFrameLength > 0 && (currentTime - _lastTxByteTime) >= FRAME_TIMEOUT) {
      // TX Frame complete, print it
      if (BUS_MONITOR_ENABLED) {
        printHexData(_txFrameBuffer, _txFrameLength, "[TX]");
      }
      _txFrameLength = 0;
    }
  }
  
  void flushFrame(byte* buffer, int& length, const char* prefix) {
    if (length > 0) {
      if (BUS_MONITOR_ENABLED) {
        printHexData(buffer, length, prefix);
      }
      length = 0;
    }
  }
  
  void logByte(byte b) {
    if (!BUS_MONITOR_ENABLED) return;
    
    unsigned long currentTime = millis();
    
    // Check if previous frame has timed out
    if (_frameLength > 0 && (currentTime - _lastByteTime) >= FRAME_TIMEOUT) {
      flushFrame(_frameBuffer, _frameLength, "[RS-485]");
    }
    
    // Add byte to frame buffer
    if (_frameLength < MAX_FRAME_SIZE) {
      _frameBuffer[_frameLength++] = b;
      _lastByteTime = currentTime;
    } else {
      // Buffer overflow - flush and start new frame
      Serial.println("[WARNING] RX frame buffer overflow, flushing...");
      flushFrame(_frameBuffer, _frameLength, "[RS-485]");
      _frameBuffer[0] = b;
      _frameLength = 1;
      _lastByteTime = currentTime;
    }
  }
  
  void logTxByte(byte b) {
    if (!BUS_MONITOR_ENABLED) return;
    
    unsigned long currentTime = millis();
    
    // Check if previous TX frame has timed out and flush it
    if (_txFrameLength > 0 && (currentTime - _lastTxByteTime) >= FRAME_TIMEOUT) {
      flushFrame(_txFrameBuffer, _txFrameLength, "[TX]");
    }
    
    // Add byte to TX frame buffer
    if (_txFrameLength < MAX_FRAME_SIZE) {
      _txFrameBuffer[_txFrameLength++] = b;
      _lastTxByteTime = currentTime;
    } else {
      // Buffer overflow - flush and start new frame
      Serial.println("[WARNING] TX frame buffer overflow, flushing...");
      flushFrame(_txFrameBuffer, _txFrameLength, "[TX]");
      _txFrameBuffer[0] = b;
      _txFrameLength = 1;
      _lastTxByteTime = currentTime;
    }
  }
  
public:
  MonitoredSerial(Stream* serial) : _serial(serial), _frameLength(0), _lastByteTime(0), 
                                     _txFrameLength(0), _lastTxByteTime(0) {}
  
  // Stream methods that must be implemented
  int available() override {
    checkFrameTimeout();
    return _serial->available();
  }
  
  int read() override {
    int b = _serial->read();
    if (b >= 0) {
      logByte((byte)b);
    }
    return b;
  }
  
  int peek() override {
    return _serial->peek();
  }
  
  // Print methods for writing
  size_t write(uint8_t b) override {
    logTxByte(b);
    return _serial->write(b);
  }
  
  // Additional methods for compatibility
  void begin(unsigned long baud) {
    if (_serial == &Serial1) {
      Serial1.begin(baud, SERIAL_CONFIG);
    }
  }
  
  void begin(unsigned long baud, uint16_t config) {
    if (_serial == &Serial1) {
      Serial1.begin(baud, config);
    }
  }
  
  void flush() {
    checkFrameTimeout();
  }
};

// Create monitored serial instance for RS-485 bus monitoring
MonitoredSerial monitoredSerial(&Serial1);

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
  
  // Initialize USB Serial for monitoring output
  Serial.begin(115200);  // Higher baud rate for USB debugging
  while (!Serial && millis() < 3000); // Wait up to 3 seconds for USB Serial
  
  Serial.println();
  Serial.println("=== MODBUS RTU Slave Simulator with Bus Monitor ===");
  Serial.print("Slave ID: ");
  Serial.println(SLAVE_ID);
  Serial.print("Bus Monitor: ");
  Serial.println(BUS_MONITOR_ENABLED ? "ENABLED" : "DISABLED");
  Serial.println("================================================");
  Serial.println();
  
  // Initialize Serial1 (Hardware UART) for RS-485/MODBUS
  Serial1.begin(SERIAL_BAUD, SERIAL_CONFIG);
  
  // Wait for serial port to be ready
  delay(100);
  
  // Configure MODBUS slave to use monitored Serial1 for RS-485
  mb.config(&monitoredSerial, SERIAL_BAUD, SERIAL_CONFIG, TXEN_PIN);
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
  
  // Flush monitored serial to print any completed frames
  monitoredSerial.flush();
  
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
