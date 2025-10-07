#include <OneWire.h>
#include <DallasTemperature.h>

// --- CONFIGURATION ---
// Pin where the DS18B20 data line is connected
const int ONE_WIRE_BUS_PIN = 25;

// --- INITIALIZATION ---
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS_PIN);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// Variable to store device addresses
DeviceAddress tempDeviceAddress;

void setup(void) {
  // Start serial communication for output
  Serial.begin(115200);
  Serial.println("Dallas Temperature Sensor Address Finder");

  // Start up the Dallas Temperature library
  sensors.begin();

  // Locate devices on the bus and report how many were found.
  int deviceCount = sensors.getDeviceCount();
  Serial.print("Found ");
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");

  if (deviceCount == 0) {
    Serial.println("No devices found! Please check your wiring.");
  } else {
    Serial.println("Sensor addresses:");
    // Loop through each device and print its address
    for (int i = 0; i < deviceCount; i++) {
      if (sensors.getAddress(tempDeviceAddress, i)) {
        Serial.print("  Sensor ");
        Serial.print(i + 1);
        Serial.print(": ");
        printAddress(tempDeviceAddress);
        Serial.println();
      }
    }
  }
}

void loop(void) {
  // Request temperatures from all sensors on the bus
  sensors.requestTemperatures();

  int deviceCount = sensors.getDeviceCount();

  // Loop through each device, print its address and temperature
  for (int i = 0; i < deviceCount; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      float tempC = sensors.getTempC(tempDeviceAddress);

      Serial.print("Address: ");
      printAddress(tempDeviceAddress);

      // Check if the sensor could be read
      if (tempC == DEVICE_DISCONNECTED_C) {
        Serial.println(" | Could not read temperature");
      } else {
        Serial.print(" | Temp C: ");
        Serial.println(tempC);
      }
    }
  }

  Serial.println("------------------------------------");
  delay(5000); // Wait 5 seconds before the next scan
}

// Helper function to print a device address in a readable format
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    // Add a "0" if the byte is less than 16
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}