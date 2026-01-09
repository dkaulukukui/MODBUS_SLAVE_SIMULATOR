# HMP110 MODBUS RTU Slave Simulator

A PlatformIO project that simulates a Vaisala HMP110 temperature and humidity sensor on an Adafruit Feather M0 Express microcontroller. The device responds to MODBUS RTU requests with simulated temperature and humidity data, making it ideal for testing MODBUS master devices and applications that work with HMP110 sensors.

## Features

- **HMP110 Sensor Emulation**: Accurately simulates Vaisala HMP110 temperature and humidity sensor
- **IEEE 754 Float Encoding**: Uses proper 32-bit floating-point format (little-endian) for sensor values
- **Direct UART Connection**: Simple TX/RX connection, no RS-485 transceiver needed
- **Realistic Simulation**: Temperature and humidity values drift over time to mimic real sensor behavior
- **USB Serial Monitoring**: Displays request count, uptime, and current sensor values
- **Visual Feedback**: Built-in LED flashes on each MODBUS request

## Hardware Requirements

- **Adafruit Feather M0 Express** (ATSAMD21G18 ARM Cortex M0 processor)
- **Direct UART Connection** to MODBUS master
  - Connect Feather TX1 (pin 1) to master RX
  - Connect Feather RX1 (pin 0) to master TX
  - Connect GND to GND
- **USB Cable** for programming and viewing monitoring output
- **No RS-485 transceiver needed** - uses direct UART communication

## Supported MODBUS Functions

The HMP110 simulator supports standard MODBUS holding register operations:

| Function Code | Function Name | Description |
|--------------|---------------|-------------|
| 0x03 | Read Holding Registers | Read temperature and humidity values |
| 0x06 | Write Single Register | Write to holding register (register values preserved) |
| 0x10 | Write Multiple Registers | Write to multiple holding registers |

## HMP110 Register Map

The simulator implements the HMP110's register layout with IEEE 754 32-bit floating-point values in little-endian byte order:

| Register Address | Parameter | Data Type | Units | Range |
|-----------------|-----------|-----------|-------|-------|
| 0x0000-0x0001 | Relative Humidity | 32-bit float (little-endian) | %RH | 30.0 - 70.0 |
| 0x0002-0x0003 | Temperature | 32-bit float (little-endian) | °C | 20.0 - 30.0 |

**Note**: Each parameter occupies 2 consecutive 16-bit registers (32 bits total for the IEEE 754 float).

### Reading Float Values

When reading from MODBUS, you'll receive two 16-bit registers per parameter. To decode:
1. Read 2 registers (e.g., 0x0000 and 0x0001 for humidity)
2. Combine as: `value = (MSW << 16) | LSW` (where LSW is first register, MSW is second)
3. Interpret the 32-bit value as an IEEE 754 float

## Simulated Sensor Behavior

The simulator provides realistic sensor readings:

- **Initial Values**: 
  - Humidity: 50.0 %RH
  - Temperature: 25.0 °C

- **Dynamic Updates**: Values change every 2 seconds with realistic drift
  - Humidity varies randomly by ±1.0 %RH per update
  - Temperature varies randomly by ±0.5 °C per update
  - Values are constrained to realistic ranges (30-70 %RH, 20-30 °C)

This mimics the behavior of a real HMP110 sensor in a stable environment with small natural variations.

## Configuration

HMP110 MODBUS settings (configured in `src/main.cpp`):

```cpp
const int SLAVE_ID = 240;                    // HMP110 default address (0xF0)
const long SERIAL_BAUD = 115200;             // HMP110 baud rate
const uint16_t SERIAL_CONFIG = SERIAL_8N2;   // 8 data bits, No parity, 2 stop bits
```

Serial ports:
- **Serial1 (TX1/RX1 pins)**: MODBUS RTU communication at 115200 baud, 8N2
- **USB Serial**: Monitoring output at 115200 baud

## USB Serial Monitoring

The simulator outputs status information to the USB Serial port (at 115200 baud). This includes:

- Startup information and configuration
- Register map layout
- Current sensor values at startup
- Request counter and sensor values for each MODBUS request received
- Heartbeat messages every 10 seconds showing uptime, request count, and current values

### Monitoring Output Format

```
╔═══════════════════════════════════════════════╗
║   HMP110 MODBUS RTU SLAVE (Direct UART)      ║
╚═══════════════════════════════════════════════╝

Slave Address: 240 (0xF0)
Baud Rate: 115200
Config: 8N2
Connection: TX1->RX(master), RX1->TX(master)

Starting MODBUS RTU Server on Serial1...
✓ MODBUS server started

Register Map:
  0x0000-0x0001: Humidity (32-bit float, little-endian)
  0x0002-0x0003: Temperature (32-bit float, little-endian)

Initial: RH=50.00 %RH, T=25.00 °C

═══════════════════════════════════════════════
✓ Ready! Listening for MODBUS requests...
═══════════════════════════════════════════════

📡 Request #1 @ 1234 ms - RH=50.12 %RH, T=25.23 °C
📡 Request #2 @ 3456 ms - RH=50.34 %RH, T=25.18 °C
Heartbeat - Uptime: 10s, Requests: 2, RH=50.45 %RH, T=25.31 °C
```

### Using the Monitor

To view the monitoring output, connect to the USB Serial port at 115200 baud:
```bash
pio device monitor --baud 115200
```

Or use any serial terminal:
```bash
# Linux/Mac
screen /dev/ttyACM0 115200

# Or using minicom
minicom -D /dev/ttyACM0 -b 115200
```

**Note**: The USB Serial port shows monitoring output (typically `/dev/ttyACM0`), while MODBUS communication happens on Serial1 through direct TX1/RX1 connections to your MODBUS master device.

## Installation & Setup

### Prerequisites

1. Install [PlatformIO](https://platformio.org/install) (standalone or as VS Code extension)
2. Install the PlatformIO CLI or use the PlatformIO IDE

### Building the Project

```bash
# Clone the repository
git clone https://github.com/dkaulukukui/MODBUS_SLAVE_SIMULATOR.git
cd MODBUS_SLAVE_SIMULATOR

# Build the project
pio run

# Upload to the Feather M0 Express
pio run --target upload

# Monitor serial output (view status and sensor values)
pio device monitor --baud 115200
```

**Wiring**:
- Connect Feather TX1 (pin 1) to your MODBUS master's RX
- Connect Feather RX1 (pin 0) to your MODBUS master's TX  
- Connect GND to GND
- USB cable provides power and monitoring output

### VS Code with PlatformIO Extension

1. Open the project folder in VS Code
2. PlatformIO will automatically detect the project
3. Click the "Build" button (checkmark) in the status bar
4. Click the "Upload" button (arrow) to flash the device
5. Open Serial Monitor at 115200 baud to view bus monitoring

## Testing the HMP110 Simulator

### Important: Connection Setup

The Feather M0 Express connects directly to your MODBUS master via UART:
- **Serial1 (TX1/RX1 pins)**: MODBUS RTU communication at 115200 baud, 8N2
- **USB Serial**: Monitoring output only

**Wiring**:
1. Connect Feather TX1 (pin 1) to MODBUS master RX
2. Connect Feather RX1 (pin 0) to MODBUS master TX
3. Connect GND to GND

### Using mbpoll (Command Line)

Install [mbpoll](https://github.com/epsilonrt/mbpoll) and test the HMP110 simulator:

```bash
# Configure your MODBUS master serial port for:
# - 115200 baud
# - 8 data bits, No parity, 2 stop bits (8N2)
# - Slave address 240 (0xF0)

# Read humidity (2 registers starting at address 0x0000)
mbpoll -m rtu -b 115200 -a 240 -P none -s 2 -t 4 -r 0 -c 2 /dev/ttyUSB0

# Read temperature (2 registers starting at address 0x0002)
mbpoll -m rtu -b 115200 -a 240 -P none -s 2 -t 4 -r 2 -c 2 /dev/ttyUSB0

# Read both humidity and temperature (4 registers starting at address 0x0000)
mbpoll -m rtu -b 115200 -a 240 -P none -s 2 -t 4 -r 0 -c 4 /dev/ttyUSB0
```

**Note**: Replace `/dev/ttyUSB0` with your actual MODBUS master serial port. The `-P none -s 2` flags specify No parity and 2 stop bits (8N2).

While running these commands, watch the USB Serial output (on the Feather) to see request counts and current sensor values!

### Using QModMaster (GUI)

1. Download and install [QModMaster](https://sourceforge.net/projects/qmodmaster/)
2. Configure connection:
   - Mode: RTU
   - Port: Select your MODBUS master serial port
   - Baud: 115200
   - Parity: None
   - Data bits: 8
   - Stop bits: 2
   - Slave ID: 240
3. Read holding registers:
   - Start address: 0
   - Number of registers: 4
   - You'll see the raw register values (need to decode as IEEE 754 floats)
4. Monitor the Feather's USB Serial (at 115200 baud) to see request count and decoded sensor values

### Using Python pymodbus

```python
from pymodbus.client import ModbusSerialClient
import struct

# Create client for HMP110 simulator
client = ModbusSerialClient(
    port='/dev/ttyUSB0',  # Your MODBUS master serial port
    baudrate=115200,
    parity='N',           # No parity
    stopbits=2,           # 2 stop bits
    bytesize=8
)

# Connect
client.connect()

# Read all 4 holding registers (humidity + temperature)
result = client.read_holding_registers(address=0, count=4, slave=240)

if result.isError():
    print("Error reading registers")
else:
    registers = result.registers
    
    # Decode humidity (registers 0-1, little-endian)
    humidity_bytes = struct.pack('<HH', registers[0], registers[1])
    humidity = struct.unpack('<f', humidity_bytes)[0]
    
    # Decode temperature (registers 2-3, little-endian)
    temp_bytes = struct.pack('<HH', registers[2], registers[3])
    temperature = struct.unpack('<f', temp_bytes)[0]
    
    print(f"Humidity: {humidity:.2f} %RH")
    print(f"Temperature: {temperature:.2f} °C")

# Close connection
client.close()
```

## Troubleshooting

### Upload Issues
- Ensure the Feather M0 is properly connected via USB
- Try double-pressing the reset button to enter bootloader mode
- Check that the correct port is selected

### Communication Issues
- Verify baud rate is 115200 with 8N2 (8 data bits, no parity, 2 stop bits)
- Check slave ID is 240 (0xF0)
- Ensure direct UART connections: TX1->RX(master), RX1->TX(master), GND->GND
- Verify the MODBUS master is configured for the correct serial port

### LED Behavior
- **Five quick flashes at startup**: Successful initialization
- **Brief flash on each MODBUS request**: Request received and processed
- **No LED activity**: Check power, upload, or MODBUS configuration

## Library Credits

This project uses the [ArduinoModbus](https://github.com/arduino-libraries/ArduinoModbus) library by Arduino.

## License

This project follows the same license as the ArduinoModbus library.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
