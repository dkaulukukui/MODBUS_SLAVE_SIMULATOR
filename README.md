# MODBUS_SLAVE_SIMULATOR

A PlatformIO project that implements a MODBUS RTU slave device on an Adafruit Feather M0 Express microcontroller. The device responds to all supported MODBUS functions with simulated data, making it ideal for testing MODBUS master devices and applications.

## Features

- **Full MODBUS RTU Slave Support**: Implements all 9 MODBUS functions supported by the modbus-arduino library
- **RS-485 Bus Monitor**: Captures and displays ALL data on the RS-485 bus in real-time
  - Monitors Modbus frames addressed to ANY slave (not just this device)
  - Monitors non-Modbus data types (e.g., polled ASCII protocols)
  - Echoes all bus traffic to USB Serial for debugging
  - Displays data in both hex and ASCII formats with timestamps
- **Simulated Data**: Provides realistic simulated sensor and control data
- **Dynamic Updates**: Simulated input values change over time to mimic real sensors
- **Visual Feedback**: Built-in LED indicates system activity

## Hardware Requirements

- **Adafruit Feather M0 Express** (ATSAMD21G18 ARM Cortex M0 processor)
- **RS-485 Module** (required for bus monitoring)
  - Connect RS-485 module to Serial1 pins (TX/RX on Feather M0)
  - If using RS-485, connect the module's TX enable pin and update `TXEN_PIN` in the code
- **USB Cable** for programming and viewing bus monitoring output

## Supported MODBUS Functions

The slave device responds to all standard MODBUS RTU functions:

| Function Code | Function Name | Description | Register Type |
|--------------|---------------|-------------|---------------|
| 0x01 | Read Coils | Read digital outputs (R/W) | Coils (10 available) |
| 0x02 | Read Discrete Inputs | Read digital inputs (R/O) | Discrete Inputs (10 available) |
| 0x03 | Read Holding Registers | Read analog outputs/storage (R/W) | Holding Registers (10 available) |
| 0x04 | Read Input Registers | Read analog inputs (R/O) | Input Registers (10 available) |
| 0x05 | Write Single Coil | Write one digital output | Coils |
| 0x06 | Write Single Register | Write one analog output | Holding Registers |
| 0x0F | Write Multiple Coils | Write multiple digital outputs | Coils |
| 0x10 | Write Multiple Registers | Write multiple analog outputs | Holding Registers |
| 0x11 | Report Server ID | Get slave identification | - |

## Simulated Data

### Coils (Address 0-9)
Digital outputs that can be read and written. Initially set in an alternating pattern (ON, OFF, ON, OFF...).

### Discrete Inputs (Address 0-9)
Read-only digital inputs simulating buttons or switches. Values change randomly over time. Initially set in a pattern where every third input is ON.

### Holding Registers (Address 0-9)
16-bit registers that can be read and written. Initial values: 1000, 1100, 1200, 1300, etc.

### Input Registers (Address 0-9)
Read-only 16-bit registers simulating sensor data. Initial values: 2000, 2050, 2100, 2150, etc. Values change slightly over time to simulate real sensor readings.

## Configuration

Default MODBUS settings (can be modified in `src/main.cpp`):

```cpp
const int SLAVE_ID = 1;           // MODBUS Slave ID
const long SERIAL_BAUD = 9600;    // Serial baud rate (for RS-485/Serial1)
const int TXEN_PIN = -1;          // RS-485 TX enable pin (-1 if not used)
const bool BUS_MONITOR_ENABLED = true;  // Enable/disable bus monitoring
```

Serial configuration:
- **RS-485 (Serial1)**: 9600 baud, 8 data bits, Even parity, 1 stop bit (8E1)
- **USB (Serial)**: 115200 baud for monitoring output

## Bus Monitoring

The bus monitor captures ALL data transmitted on the RS-485 bus and echoes it to the USB Serial port in real-time. This includes:

- **Modbus frames addressed to this slave** - Will be processed normally AND monitored
- **Modbus frames addressed to other slaves** - Captured for monitoring only (not processed)
- **Non-Modbus data** - Any data on the bus (e.g., ASCII protocols)

### Monitoring Output Format

```
=== MODBUS RTU Slave Simulator with Bus Monitor ===
Slave ID: 1
Bus Monitor: ENABLED
================================================

[RS-485] 1234 ms | 8 bytes: 01 03 00 00 00 0A C5 CD | ASCII: ........
[TX] 1245 ms | 23 bytes: 01 03 14 00 00 00 01 03 E8 ... | ASCII: ..........
[RS-485] 2300 ms | 8 bytes: 02 03 00 00 00 05 85 F9 | ASCII: ........
[RS-485] 3456 ms | 9 bytes: 3E 30 30 31 0D 0A | ASCII: >001..
```

Each line shows:
- **[RS-485]** or **[TX]** prefix (received vs transmitted data)
- **Timestamp** in milliseconds since startup
- **Byte count** of the captured frame
- **Hex representation** of all bytes (space separated)
- **ASCII representation** (printable characters shown, others as '.')

### Example Scenarios

#### Scenario 1: Modbus Master Querying This Slave
```
[RS-485] 5123 ms | 8 bytes: 01 03 00 00 00 0A C5 CD | ASCII: ........
[TX] 5156 ms | 23 bytes: 01 03 14 03 E8 04 4C ... | ASCII: ........
```
The slave receives a Modbus Read Holding Registers request (function 0x03) and responds with data.

#### Scenario 2: Multi-Drop Network with Multiple Slaves
```
[RS-485] 8001 ms | 8 bytes: 01 03 00 00 00 0A C5 CD | ASCII: ........
[TX] 8034 ms | 23 bytes: 01 03 14 03 E8 04 4C ... | ASCII: ........
[RS-485] 8200 ms | 8 bytes: 02 03 00 00 00 05 85 F9 | ASCII: ........
```
Master queries slave 1 (this device responds), then queries slave 2 (captured but not processed).

#### Scenario 3: Non-Modbus ASCII Protocol
```
[RS-485] 12456 ms | 6 bytes: 3E 30 30 31 0D 0A | ASCII: >001..
[RS-485] 12500 ms | 15 bytes: 30 30 31 2C 31 32 33 2E 34 35 0D 0A | ASCII: 001,123.45..
```
An ASCII polling protocol is captured and displayed alongside Modbus traffic.

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

**Note**: The monitoring output is on the Feather's USB port (typically `/dev/ttyACM0`), while Modbus communication happens on the RS-485 adapter port (typically `/dev/ttyUSB0`).

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

# Monitor serial output (view bus monitoring data)
pio device monitor --baud 115200
```

**Note**: The Modbus communication happens on Serial1 (TX/RX pins), while monitoring output is on USB Serial. You'll need two connections:
1. RS-485 module connected to TX/RX pins for Modbus communication
2. USB cable for monitoring output

### VS Code with PlatformIO Extension

1. Open the project folder in VS Code
2. PlatformIO will automatically detect the project
3. Click the "Build" button (checkmark) in the status bar
4. Click the "Upload" button (arrow) to flash the device
5. Open Serial Monitor at 115200 baud to view bus monitoring

## Testing the MODBUS Slave

### Important: Serial Port Configuration

The Feather M0 Express has two serial ports:
- **Serial1 (TX/RX pins)**: Used for RS-485/Modbus communication
- **USB Serial**: Used for monitoring output only

When using Modbus tools like `mbpoll`, you need to:
1. Connect your RS-485 adapter to the Feather's TX/RX pins (Serial1)
2. Connect your RS-485 adapter to your computer
3. Use the RS-485 adapter's serial port (NOT the Feather's USB port) with mbpoll

### Using mbpoll (Command Line)

Install [mbpoll](https://github.com/epsilonrt/mbpoll) and test the slave:

```bash
# Replace /dev/ttyUSB0 with your RS-485 adapter's port
# (NOT /dev/ttyACM0 which is the Feather's USB port)

# Read 5 coils starting at address 0
mbpoll -m rtu -b 9600 -a 1 -t 0 -r 1 -c 5 /dev/ttyUSB0

# Read 5 holding registers starting at address 0
mbpoll -m rtu -b 9600 -a 1 -t 4 -r 1 -c 5 /dev/ttyUSB0

# Write value 1234 to holding register at address 0
mbpoll -m rtu -b 9600 -a 1 -t 4 -r 1 /dev/ttyUSB0 1234

# Read 5 input registers starting at address 0
mbpoll -m rtu -b 9600 -a 1 -t 3 -r 1 -c 5 /dev/ttyUSB0

# Read 5 discrete inputs starting at address 0
mbpoll -m rtu -b 9600 -a 1 -t 1 -r 1 -c 5 /dev/ttyUSB0
```

While running these commands, watch the USB Serial output to see all bus traffic being captured and displayed!

### Using QModMaster (GUI)

1. Download and install [QModMaster](https://sourceforge.net/projects/qmodmaster/)
2. Configure connection:
   - Mode: RTU
   - Port: Select your RS-485 adapter port (NOT the Feather USB port)
   - Baud: 9600
   - Parity: Even
   - Data bits: 8
   - Stop bits: 1
   - Slave ID: 1
3. Test reading and writing different register types
4. Monitor the Feather's USB Serial (at 115200 baud) to see all captured bus traffic

### Using Python pymodbus

```python
from pymodbus.client import ModbusSerialClient

# Create client - use your RS-485 adapter's port
client = ModbusSerialClient(
    port='/dev/ttyUSB0',  # Your RS-485 adapter, NOT /dev/ttyACM0
    baudrate=9600,
    parity='E',
    stopbits=1,
    bytesize=8
)

# Connect
client.connect()

# Read 5 holding registers starting at address 0
result = client.read_holding_registers(address=0, count=5, slave=1)
print(result.registers)

# Write to holding register
client.write_register(address=0, value=5000, slave=1)

# Close connection
client.close()
```

## Troubleshooting

### Upload Issues
- Ensure the Feather M0 is properly connected via USB
- Try double-pressing the reset button to enter bootloader mode
- Check that the correct port is selected

### Communication Issues
- Verify baud rate, parity, and stop bits match on master and slave
- Check slave ID matches (default is 1)
- Ensure proper RS-485 connections if using external transceiver
- For RS-485, verify TX enable pin is correctly configured

### LED Behavior
- **Three quick flashes at startup**: Successful initialization
- **Slow blinking**: Normal operation (updates every second)
- **No LED activity**: Check power and upload

## Library Credits

This project uses the [modbus-arduino](https://github.com/epsilonrt/modbus-arduino) library by epsilonrt.

## License

This project follows the same license as the modbus-arduino library (BSD License).

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
