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

---

### Sensor Configuration

**Identifying Temperature Sensors**

This project uses Dallas Temperature sensors (DS18B20), which have unique hardware addresses. To ensure the system reads the correct sensor for oil temperature, intake air temperature (IAT), and engine temperature, you must first identify each sensor's unique address.

A utility sketch is provided in the `FindSensorAddresses` directory to help you with this process.

**Instructions:**

1.  **Open the Utility Sketch**: In the Arduino IDE, navigate to the `FindSensorAddresses` folder and open the `FindAddresses.ino` sketch.
2.  **Upload to Your Device**: Upload this sketch to your ESP32.
3.  **Open the Serial Monitor**: With the device connected, open the Serial Monitor (baud rate 115200). The monitor will list the unique address of every connected Dallas Temperature sensor.
4.  **Identify Each Sensor**: To determine which address belongs to which sensor (e.g., the oil temperature sensor), you can gently heat one of the physical sensors and observe which address reports a rising temperature in the Serial Monitor. Note down the address for each sensor (oil, IAT, engine).
5.  **Update the Main Sketch**: Open the main project sketch (`V6_3_DTC_page/V6_3_DTC_page.ino`). Find the "SENSOR ADDRESSES" section and replace the placeholder addresses with the actual addresses you discovered.

By following these steps, you will ensure that the temperature readings are accurate and assigned to the correct functions in the display.
