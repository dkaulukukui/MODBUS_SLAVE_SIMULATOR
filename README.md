# MODBUS_SLAVE_SIMULATOR

A PlatformIO project that implements a MODBUS RTU slave device on an Adafruit Feather M0 Express microcontroller. The device responds to all supported MODBUS functions with simulated data, making it ideal for testing MODBUS master devices and applications.

## Features

- **Full MODBUS RTU Slave Support**: Implements all 9 MODBUS functions supported by the modbus-arduino library
- **Simulated Data**: Provides realistic simulated sensor and control data
- **Dynamic Updates**: Simulated input values change over time to mimic real sensors
- **Visual Feedback**: Built-in LED indicates system activity

## Hardware Requirements

- **Adafruit Feather M0 Express** (ATSAMD21G18 ARM Cortex M0 processor)
- **RS-485 Module** (optional, for proper industrial MODBUS networks)
  - If using RS-485, connect the module's TX enable pin and update `TXEN_PIN` in the code
- **USB Cable** for programming and serial communication

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
const long SERIAL_BAUD = 9600;    // Serial baud rate
const int TXEN_PIN = -1;          // RS-485 TX enable pin (-1 if not used)
```

Serial configuration: **9600 baud, 8 data bits, Even parity, 1 stop bit (8E1)**

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

# Monitor serial output (optional)
pio device monitor
```

### VS Code with PlatformIO Extension

1. Open the project folder in VS Code
2. PlatformIO will automatically detect the project
3. Click the "Build" button (checkmark) in the status bar
4. Click the "Upload" button (arrow) to flash the device

## Testing the MODBUS Slave

### Using mbpoll (Command Line)

Install [mbpoll](https://github.com/epsilonrt/mbpoll) and test the slave:

```bash
# Read 5 coils starting at address 0
mbpoll -m rtu -b 9600 -a 1 -t 0 -r 1 -c 5 /dev/ttyACM0

# Read 5 holding registers starting at address 0
mbpoll -m rtu -b 9600 -a 1 -t 4 -r 1 -c 5 /dev/ttyACM0

# Write value 1234 to holding register at address 0
mbpoll -m rtu -b 9600 -a 1 -t 4 -r 1 /dev/ttyACM0 1234

# Read 5 input registers starting at address 0
mbpoll -m rtu -b 9600 -a 1 -t 3 -r 1 -c 5 /dev/ttyACM0

# Read 5 discrete inputs starting at address 0
mbpoll -m rtu -b 9600 -a 1 -t 1 -r 1 -c 5 /dev/ttyACM0
```

**Note**: Replace `/dev/ttyACM0` with your actual serial port (check `pio device list`).

### Using QModMaster (GUI)

1. Download and install [QModMaster](https://sourceforge.net/projects/qmodmaster/)
2. Configure connection:
   - Mode: RTU
   - Port: Select your Feather M0 port
   - Baud: 9600
   - Parity: Even
   - Data bits: 8
   - Stop bits: 1
   - Slave ID: 1
3. Test reading and writing different register types

### Using Python pymodbus

```python
from pymodbus.client import ModbusSerialClient

# Create client
client = ModbusSerialClient(
    port='/dev/ttyACM0',
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
