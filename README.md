This Arduino sketch is designed for an ESP32-based automotive monitoring system, specifically for a Hyundai Terracan, integrating multiple sensors and communication protocols to display real-time vehicle data on an SSD1322 OLED display. Key features include:

- **Sensor Monitoring**: Reads oil pressure, MAP (Manifold Absolute Pressure), battery voltage, oil temperature, intake air temperature (IAT), and engine temperature using analog inputs and a DallasTemperature sensor via OneWire.
- **Display Management**: Uses an SSD1322 OLED to show data across multiple pages (overview, oil pressure graph, MAP graph, battery graph, TPMS, and ECU fault codes), with a splash screen featuring a Hyundai logo and dynamic brightness adjustment based on day/night mode.
- **TPMS (Tire Pressure Monitoring System)**: Utilizes BLE (Bluetooth Low Energy) to scan for tire sensor data (pressure, temperature, voltage) and displays it with a car graphic indicating tire positions.
- **K-Line Communication**: Interfaces with the vehicle's ECU via K-Line (ISO 9141) to read and clear diagnostic trouble codes (DTCs), with descriptions stored in PROGMEM for memory efficiency.
- **User Interaction**: Incorporates five buttons for navigation (page switching, brightness control, buzzer mute, graph reset, DTC read/clear) and a serial interface for configuration (e.g., setting thresholds, TPMS addresses).
- **Data Storage**: Uses EEPROM to store settings like brightness levels, sensor calibration values, and TPMS addresses, with validation to ensure values stay within defined limits.
- **Power Management**: Supports deep sleep mode, triggered by a button, with wakeup on button press, and includes a watchdog timer to prevent system hangs.
- **Data Visualization**: Displays real-time sensor data, min/max values, and graphical trends for oil pressure, MAP, and battery voltage, with non-blocking updates and error handling for sensor faults.

The system is designed for reliability with a watchdog timer, non-blocking K-Line reads, and efficient memory use via PROGMEM for static data. It provides a comprehensive interface for monitoring critical vehicle parameters and diagnosing issues.
