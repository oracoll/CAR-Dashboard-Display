#include <Arduino.h>
#include <SSD1322_for_Adafruit_GFX.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_sleep.h>
#include <Bounce2.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>
#include <HardwareSerial.h> // For K-Line communication

// —— CONFIG ——
#define WDT_TIMEOUT 10000 // 10 seconds
constexpr float ADC_MAX = 4095.0;
constexpr float VREF    = 3.3;
const long SCREEN_UPDATE_INTERVAL = 100;
const long SENSOR_UPDATE_INTERVAL = 100;
const long TEMP_REQUEST_INTERVAL  = 800;
const int  DEBOUNCE_DELAY         = 50;

// —— RESISTOR DIVIDER CONFIG (in kOhms) ——
const float OIL_R1 = 2.0;  // Resistor from oil pressure sensor to ADC pin (kΩ)
const float OIL_R2 = 3.3;  // Resistor from ADC pin to ground (kΩ)
const float MAP_R1 = 2.0;  // Resistor from MAP sensor to ADC pin (kΩ)
const float MAP_R2 = 3.3;  // Resistor from ADC pin to ground (kΩ)

// —— BRIGHTNESS CONFIG ——
#define DAY_NIGHT_PIN 13 // GPIO pin to detect day (HIGH) or night (LOW)
const uint8_t brightnessLevels[] = {25, 50, 75, 100, 125, 150, 175, 200, 225, 250}; // Levels 1–10
const int numBrightnessLevels = 10;
int dayBrightnessIndex = 5; // Default to level 6 (150) for day
int nightBrightnessIndex = 2; // Default to level 3 (75) for night
bool isDayMode = true; // Track current mode (true for day, false for night)

// —— EEPROM CONFIG ——
const int EEPROM_SIZE = 256;
const int TPMS_MAX_LEN = 18; // Max length for TPMS addresses (17 chars + null)
const uint8_t EEPROM_MAGIC = 0xAA; // Magic byte to check initialization

// —— K-LINE (OBD) CONFIG ——
#define K_LINE_RX_PIN 19 // Connect to K-line transceiver RX
#define K_LINE_TX_PIN 21 // Connect to K-line transceiver TX
HardwareSerial KLine(2); // Use UART2 for K-Line

// EEPROM Addresses
const int ADDR_MAGIC = 0;
const int ADDR_DAY_BRIGHTNESS = ADDR_MAGIC + sizeof(uint8_t);
const int ADDR_NIGHT_BRIGHTNESS = ADDR_DAY_BRIGHTNESS + sizeof(uint8_t);
const int ADDR_OIL_WARN = ADDR_NIGHT_BRIGHTNESS + sizeof(uint8_t);
const int ADDR_OIL_CRIT = ADDR_OIL_WARN + sizeof(float);
const int ADDR_OIL_MIN_V = ADDR_OIL_CRIT + sizeof(float);
const int ADDR_OIL_MAX_V = ADDR_OIL_MIN_V + sizeof(float);
const int ADDR_OIL_MIN_P = ADDR_OIL_MAX_V + sizeof(float);
const int ADDR_OIL_MAX_P = ADDR_OIL_MIN_P + sizeof(float);
const int ADDR_MAP_MIN_V = ADDR_OIL_MAX_P + sizeof(float);
const int ADDR_MAP_MAX_V = ADDR_MAP_MIN_V + sizeof(float);
const int ADDR_MAP_MIN_P = ADDR_MAP_MAX_V + sizeof(float);
const int ADDR_MAP_MAX_P = ADDR_MAP_MIN_P + sizeof(float);
const int ADDR_OIL_TEMP_OFFSET = ADDR_MAP_MAX_P + sizeof(float);
const int ADDR_IAT_OFFSET = ADDR_OIL_TEMP_OFFSET + sizeof(float);
const int ADDR_ENG_TEMP_OFFSET = ADDR_IAT_OFFSET + sizeof(float);
const int ADDR_TPMS_FL = ADDR_ENG_TEMP_OFFSET + sizeof(float);
const int ADDR_TPMS_FR = ADDR_TPMS_FL + TPMS_MAX_LEN;
const int ADDR_TPMS_RL = ADDR_TPMS_FR + TPMS_MAX_LEN;
const int ADDR_TPMS_RR = ADDR_TPMS_RL + TPMS_MAX_LEN;

// —— SENSOR LIMITS ——
#define OIL_MIN_P_LIMIT 0.00
#define OIL_MAX_P_LIMIT 20.00
#define MAP_MIN_P_LIMIT 0.00
#define MAP_MAX_P_LIMIT 5.00
#define TEMP_MIN_OFFSET 0.00
#define TEMP_MAX_OFFSET 150.00

// —— SENSOR VARIABLES ——
float oil_pressure_warn_threshold = 0.3;
float oil_pressure_crit_threshold = 0.1;
float oil_min_v = 0.5;
float oil_max_v = 4.5;
float oil_min_p = 0.0;
float oil_max_p = 10.0;
float map_min_v = 0.4;
float map_max_v = 4.65;
float map_min_p = 0.2;
float map_max_p = 2.5;
float oil_temp_offset = 0.0;
float iat_offset = 0.0;
float engine_temp_offset = 0.0;
String tpms_fl = "4a:20:00:00:22:2b";
String tpms_fr = "4a:6c:00:00:9c:13";
String tpms_rl = "4a:23:00:00:5f:00";
String tpms_rr = "4a:24:00:00:8e:5b";

// —— OLED PINS ——
#define OLED_CLK    18
#define OLED_MOSI   23
#define OLED_CS     26
#define OLED_DC     14
#define OLED_RESET  27

// —— BUTTONS & BUZZER ——
const int buttonPins[5] = {15, 2, 4, 16, 17};
Bounce buttons[5];
#define BUZZER_PIN       5

// —— TEMP SENSOR ——
#define TEMP_SENSOR_PIN 25
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);

// —— ANALOG SENSORS ——
#define OIL_PRESSURE_PIN 36
#define MAP_PIN          39
#define BATTERY_PIN      34

// —— SPLASH SCREEN ——
#define logoWidth  128
#define logoHeight 64
// Placeholder 128x64 Hyundai logo bitmap (replace with actual resized bitmap)
const uint8_t hyundai_logo_bits[] PROGMEM = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xf0,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0xf8,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xff,0xc0,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff,0xff,0xff,0xfe,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0x00,
0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0x80,0x03,0xff,0xff,0xf8,0x00,0x00,
0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xfe,0x00,0x00,0x03,0xff,0xfe,0x00,0x00,0x00,
0x00,0x00,0x03,0xff,0xfc,0x3f,0xfc,0x00,0x00,0x00,0x1f,0xff,0xc0,0x00,0x00,0x00,
0x00,0x0f,0xff,0xc0,0x7f,0xf8,0x00,0x00,0x00,0x01,0xff,0xf0,0x00,0x00,0x00,0x00,
0x1f,0xfc,0x00,0xff,0xf0,0x00,0x00,0x00,0x00,0x7f,0xfc,0x00,0x00,0x00,0x00,0x7f,
0xe0,0x00,0xff,0xf0,0x00,0x00,0x00,0x00,0x7f,0xfe,0x00,0x00,0x00,0x01,0xff,0x80,
0x01,0xff,0xe0,0x00,0x00,0x00,0x00,0xff,0xff,0x80,0x00,0x00,0x03,0xfe,0x00,0x03,
0xff,0xe0,0x00,0x00,0x00,0x03,0xff,0xff,0xc0,0x00,0x00,0x07,0xf8,0x00,0x07,0xff,
0xc0,0x00,0x00,0x00,0x07,0xff,0xff,0xe0,0x00,0x00,0x0f,0xe0,0x00,0x07,0xff,0xc0,
0x00,0x00,0x00,0x1f,0xff,0xe7,0xf8,0x00,0x00,0x1f,0xc0,0x00,0x0f,0xff,0x80,0x00,
0x00,0x00,0x7f,0xff,0xc3,0xfc,0x00,0x00,0x3f,0x00,0x00,0x1f,0xff,0x80,0x00,0x00,
0x00,0xff,0xff,0xc0,0xfe,0x00,0x00,0x7e,0x00,0x00,0x3f,0xff,0x80,0x00,0x00,0x07,
0xff,0xff,0xc0,0x7f,0x00,0x00,0xfc,0x00,0x00,0x3f,0xff,0x80,0x00,0x00,0x1f,0xff,
0xff,0x80,0x3f,0x00,0x01,0xf8,0x00,0x00,0x7f,0xff,0x00,0x00,0x00,0x7f,0xff,0xff,
0x80,0x1f,0x80,0x01,0xf0,0x00,0x00,0x7f,0xff,0x00,0x00,0x03,0xff,0xff,0xff,0x00,
0x0f,0xc0,0x03,0xf0,0x00,0x00,0xff,0xff,0x00,0x00,0x0f,0xff,0xff,0xff,0x00,0x07,
0xc0,0x07,0xe0,0x00,0x01,0xff,0xff,0x80,0x00,0x7f,0xff,0xff,0xfe,0x00,0x07,0xe0,
0x07,0xc0,0x00,0x01,0xff,0xff,0x80,0x03,0xff,0xff,0xff,0xfe,0x00,0x03,0xe0,0x07,
0xc0,0x00,0x03,0xff,0xff,0xe0,0x3f,0xff,0xff,0xff,0xfc,0x00,0x03,0xe0,0x07,0xc0,
0x00,0x03,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc,0x00,0x01,0xf0,0x0f,0x80,0x00,
0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf8,0x00,0x01,0xf0,0x0f,0x80,0x00,0x07,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf8,0x00,0x01,0xf0,0x0f,0x80,0x00,0x0f,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xf0,0x00,0x01,0xf0,0x0f,0x80,0x00,0x0f,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xf0,0x00,0x01,0xf0,0x0f,0x80,0x00,0x1f,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xe0,0x00,0x01,0xf0,0x0f,0x80,0x00,0x1f,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xe0,0x00,0x01,0xf0,0x07,0xc0,0x00,0x3f,0xff,0xff,0xff,0xff,0x07,
0xff,0xff,0xc0,0x00,0x03,0xf0,0x07,0xc0,0x00,0x3f,0xff,0xff,0xff,0xf0,0x01,0xff,
0xff,0xc0,0x00,0x03,0xe0,0x07,0xe0,0x00,0x7f,0xff,0xff,0xff,0x80,0x01,0xff,0xff,
0x80,0x00,0x03,0xe0,0x03,0xe0,0x00,0x7f,0xff,0xff,0xf8,0x00,0x00,0xff,0xff,0x00,
0x00,0x07,0xc0,0x03,0xf0,0x00,0xff,0xff,0xff,0xe0,0x00,0x00,0xff,0xff,0x00,0x00,
0x0f,0xc0,0x01,0xf8,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xfe,0x00,0x00,0x1f,
0x80,0x00,0xfe,0x01,0xff,0xff,0xfc,0x00,0x00,0x00,0xff,0xfe,0x00,0x00,0x3f,0x80,
0x00,0x7f,0x03,0xff,0xff,0xf0,0x00,0x00,0x00,0xff,0xfc,0x00,0x00,0x7f,0x00,0x00,
0x3f,0x83,0xff,0xff,0xc0,0x00,0x00,0x01,0xff,0xf8,0x00,0x00,0xfe,0x00,0x00,0x1f,
0xe7,0xff,0xfe,0x00,0x00,0x00,0x01,0xff,0xf8,0x00,0x01,0xfc,0x00,0x00,0x0f,0xff,
0xff,0xf0,0x00,0x00,0x00,0x03,0xff,0xe0,0x00,0x07,0xf8,0x00,0x00,0x03,0xff,0xff,
0xe0,0x00,0x00,0x00,0x03,0xff,0xe0,0x00,0x3f,0xe0,0x00,0x00,0x01,0xff,0xff,0x80,
0x00,0x00,0x00,0x07,0xff,0xc0,0x00,0xff,0x80,0x00,0x00,0x00,0xff,0xff,0x00,0x00,
0x00,0x00,0x07,0xff,0x80,0x03,0xff,0x00,0x00,0x00,0x00,0x3f,0xfe,0x00,0x00,0x00,
0x00,0x0f,0xff,0x00,0x1f,0xfc,0x00,0x00,0x00,0x00,0x0f,0xff,0x00,0x00,0x00,0x00,
0x0f,0xfe,0x00,0xff,0xf0,0x00,0x00,0x00,0x00,0x03,0xff,0xf0,0x00,0x00,0x00,0x1f,
0xfe,0x0f,0xff,0xc0,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0x00,0x00,0x00,0x3f,0xfe,
0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xfe,0x00,0x00,0xff,0xff,0xff,
0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xff,0xff,0xf0,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xff,0xfe,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

const unsigned long splashDuration = 2000;
const int splashTextY = 60;

// —— DISPLAY ——
Adafruit_SSD1322 display(256, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// —— DATA BUFFERS & STATS ——
const int GRAPH_WIDTH = 256;
float oilPressureBuffer[GRAPH_WIDTH], mapBuffer[GRAPH_WIDTH], batteryBuffer[GRAPH_WIDTH];
int bufferIndex = 0;

float oilPressureMin = INFINITY, oilPressureMax = -INFINITY;
float mapMin = INFINITY, mapMax = -INFINITY;
float oilTempMin = INFINITY, oilTempMax = -INFINITY;
float iatMin = INFINITY, iatMax = -INFINITY;
float batteryMin = INFINITY, batteryMax = -INFINITY;
float engineTempMin = INFINITY, engineTempMax = -INFINITY;

bool buzzerMuted = false;
unsigned long lastBeepTime = 0;
unsigned long lastUpdate = 0, lastSensorUpdate = 0, lastTempRequest = 0;

// —— FAULT CODE VARIABLES ——
const int MAX_FAULTS = 10;
char faultCodes[MAX_FAULTS][30]; // Use char array to avoid String fragmentation
int numFaults = 0;
int faultScrollIndex = 0;
unsigned long button3PressTime = 0;
bool button3LongPress = false;

// For non-blocking messages
bool messageMode = false;
unsigned long messageEndTime = 0;

// —— TPMS DATA ——
int front_left_updated = 0;
float front_left_voltage = 0.0;
int front_left_temperature = 0;
float front_left_pressure_psi = 0.0;

int front_right_updated = 0;
float front_right_voltage = 0.0;
int front_right_temperature = 0;
float front_right_pressure_psi = 0.0;

int rear_left_updated = 0;
float rear_left_voltage = 0.0;
int rear_left_temperature = 0;
float rear_left_pressure_psi = 0.0;

int rear_right_updated = 0;
float rear_right_voltage = 0.0;
int rear_right_temperature = 0;
float rear_right_pressure_psi = 0.0;

BLEScan *pBLEScan;

// —— TPMS GRAPHICS ——
static const uint8_t image_arrow_FL_bits[] PROGMEM =
{0xff, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xc0, 0x03};

static const uint8_t image_arrow_FR_bits[] PROGMEM =
{0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff,
 0x03, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00};

static const uint8_t image_arrow_RL_bits[] PROGMEM =
{0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0,
0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x18, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0xff, 0xff, 0xff, 0xff,
0xff, 0x07, 0x00};

static const uint8_t image_arrow_RR_bits[] PROGMEM =
{0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0x80, 0xff, 0xff, 0xff,
0xff, 0xff, 0x03};

static const uint8_t image_car_bits[] PROGMEM =
{0x00,0x03,0xf8,0x00,0x00,0x7f,0xf7,0x80,0x01,0x1f,0xfe,0x60,0x06,0xff,0xff,
0x90,0x09,0xff,0xff,0xe8,0x0b,0xff,0xff,0xf4,0x07,0xff,0xff,0xfc,0x0f,0xff,
0xff,0xfc,0x0f,0xff,0xff,0xfc,0x0f,0xff,0xff,0xfc,0x1f,0xff,0xff,0xfc,0x0f,
0xff,0xff,0xfc,0x1f,0xff,0xff,0xfc,0x0f,0xff,0xff,0xfc,0x1f,0xff,0xff,0xfc,
0x0f,0xff,0xff,0xfc,0x1f,0xff,0xff,0xfc,0x07,0xff,0xff,0xfc,0x1f,0xfc,0x0f,
0xfc,0x0f,0x80,0x00,0x7c,0x1e,0x00,0x00,0x1c,0x0e,0x00,0x00,0x1c,0x1e,0x00,
0x00,0x1c,0x1e,0x00,0x00,0x1e,0x3e,0x00,0x00,0x3f,0x0b,0x00,0x00,0x25,0x19,
0x00,0x00,0x24,0x09,0x1f,0xfe,0x24,0x19,0xff,0xff,0xc4,0x09,0xff,0xff,0xc4,
0x08,0xff,0xff,0xc4,0x18,0xff,0xff,0xc4,0x18,0xff,0xff,0xc6,0x18,0xff,0xff,
0xc6,0x18,0xff,0xff,0xc4,0x08,0xff,0xff,0xc4,0x0e,0xff,0xff,0xfc,0x0f,0xff,
0xff,0xfc,0x08,0xff,0xff,0xc4,0x08,0xff,0xff,0xc4,0x08,0xff,0xff,0xc4,0x08,
0xff,0xff,0xc4,0x08,0xff,0xff,0xcc,0x08,0xff,0xff,0xc4,0x08,0xff,0xff,0xcc,
0x18,0xff,0xff,0xc4,0x18,0xff,0xff,0xce,0x18,0xff,0xff,0xc6,0x19,0xff,0xff,
0xec,0x0e,0xff,0xff,0xdc,0x08,0xff,0xff,0xc4,0x08,0xff,0xff,0xcc,0x08,0xff,
0xff,0xcc,0x08,0xff,0xff,0xcc,0x08,0xff,0xff,0xcc,0x08,0xff,0xff,0xcc,0x08,
0xff,0xff,0xcc,0x08,0xff,0xff,0xcc,0x0d,0xff,0xff,0xec,0x0f,0xff,0xff,0xfc,
0x0f,0xff,0xff,0xfc,0x0f,0xff,0xff,0xfc,0x0f,0xff,0xff,0x7c,0x0f,0x00,0x00,
0x3c,0x0e,0x00,0x00,0x14,0x0b,0xe0,0x00,0xf4,0x04,0x7f,0xff,0x88,0x03,0xff,
0xff,0xf0};

// —— PAGES ——
enum PageID { OVERVIEW, OIL_GRAPH, MAP_GRAPH, BATT_GRAPH, TPMS_PAGE, FAULT_CODES_PAGE };
PageID currentPage = OVERVIEW;

const char* labels0[6] = {"OILP","MAP","OILT","IAT","BATT","ENGT"};
float values0[6]      = {0,0,0,0,0,0};
const char* units0[6] = {"Bar","Bar","C","C","V","C"};
float min0[6]         = {0.0, 0.2, -40, -40, 0, -40};
float max0[6]         = {10.0, 2.5, 120, 120, 16, 120};

const char* labels1[3] = {"OILP","OILT","ENGT"};
float values1[3]      = {0,0,0};
const char* units1[3] = {"Bar","C","C"};

const char* labels2[3] = {"MAP","IAT","OILP"};
float values2[3]      = {0,0,0};
const char* units2[3] = {"Bar","C","Bar"};

const char* labels3[3] = {"BATT","OILT","ENGT"};
float values3[3]      = {0,0,0};
const char* units3[3] = {"V","C","C"};

struct Page {
  const char** labels;
  float*      values;
  const char** units;
  int         count;
  bool        graph;
  float*      buffer;
  float       bufMin, bufMax;
};

Page pages[6] = {
  { labels0, values0, units0, 6, false, nullptr,           0,    0      },
  { labels1, values1, units1, 3, true,  oilPressureBuffer, 0.0,  10.0   },
  { labels2, values2, units2, 3, true,  mapBuffer,         0.2,  2.5    },
  { labels3, values3, units3, 3, true,  batteryBuffer,     0,    16     },
  { nullptr, nullptr, nullptr, 0, false, nullptr,           0,    0      }, // TPMS_PAGE placeholder
  { nullptr, nullptr, nullptr, 0, false, nullptr,           0,    0      }  // FAULT_CODES_PAGE placeholder
};

// —— BLE CALLBACK ——
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveName() && advertisedDevice.getName() == "BR") {
      String address = advertisedDevice.getAddress().toString();
      int tireIndex = -1;
      if (address == tpms_fl) tireIndex = 0; // F.L
      else if (address == tpms_fr) tireIndex = 1; // F.R
      else if (address == tpms_rl) tireIndex = 2; // R.L
      else if (address == tpms_rr) tireIndex = 3; // R.R
      if (tireIndex != -1 && advertisedDevice.haveManufacturerData()) {
        String strManufacturerData = advertisedDevice.getManufacturerData();
        byte cManufacturerData[100];
        memcpy(cManufacturerData, strManufacturerData.c_str(), strManufacturerData.length());
        if (strManufacturerData.length() >= 7) {
          float voltage = (float)cManufacturerData[1] / 10.0;
          int temperature = cManufacturerData[2];
          uint16_t pressure_psi_x10 = (uint16_t)cManufacturerData[3] << 8 | cManufacturerData[4];
          float pressure_psi = (float)pressure_psi_x10 / 10.0;
          if (tireIndex == 0) {
            front_left_voltage = voltage;
            front_left_temperature = temperature;
            front_left_pressure_psi = pressure_psi;
            front_left_updated = 1;
          } else if (tireIndex == 1) {
            front_right_voltage = voltage;
            front_right_temperature = temperature;
            front_right_pressure_psi = pressure_psi;
            front_right_updated = 1;
          } else if (tireIndex == 2) {
            rear_left_voltage = voltage;
            rear_left_temperature = temperature;
            rear_left_pressure_psi = pressure_psi;
            rear_left_updated = 1;
          } else if (tireIndex == 3) {
            rear_right_voltage = voltage;
            rear_right_temperature = temperature;
            rear_right_pressure_psi = pressure_psi;
            rear_right_updated = 1;
          }
        }
      }
    }
  }
};

// —— DTC DESCRIPTION LOOKUP TABLE ——
// Stored in PROGMEM to save RAM
struct DTC {
  const char* code;
  const char* description;
};

const DTC dtc_descriptions[] PROGMEM = {
  {"P1100", "Map Sensor - Malfunction"},
  {"P1101", "Map Sensor - Abnormal"},
  {"P1102", "Map Sensor - Low Input"},
  {"P1103", "Map Sensor - High Input"},
  {"P1104", "Air Flow Sensor Fault"},
  {"P1112", "VGT Actuator - Malfunction"},
  {"P1115", "Coolant Temperature Input - Abnormal"},
  {"P1116", "Boost Pressure Sensor - Malfunction"},
  {"P1119", "Inlet Metering Valve Control"},
  {"P1120", "Electric Governor - Malfunction"},
  {"P1121", "Throttle Position Input - Abnormal"},
  {"P1122", "Boost Pressure Control Valve"},
  {"P1135", "Injection Timing Servo"},
  {"P1140", "Inlet Air Temp Sensor Malfunction"},
  {"P1145", "Main Duty Solenoid - Malfunction"},
  {"P1162", "High Pressure Pump & Fuel Line"},
  {"P1170", "ECM (Barometric Pressure Sensor)"},
  {"P1180", "Fuel Pressure Regulator - Malfunction"},
  {"P1181", "Fuel Pressure Monitoring"},
  {"P1186", "Fuel Pressure - Too Low"},
  {"P1187", "Regulator Valve - Stuck"},
  {"P1188", "Fuel Pressure - Leakage"},
  {"P1301", "TDC Sensor - Abnormal"},
  {"P1304", "Phase Sensor"},
  {"P1321", "Glow Indicator Lamp - Short"},
  {"P1322", "Glow Indicator Lamp - Open"},
  {"P1324", "Glow Relay - Malfunction"},
  {"P1400", "Manifold Differential Pressure Sensor"},
  {"P1405", "EGR Temperature Incorrect"},
  {"P1500", "Vehicle Speed Signal Malfunction"},
  {"P1514", "Oil Temperature Sensor"},
  {"P1529", "TCM MIL On Request Signal"},
  {"P1603", "CAN Communication Bus Off"},
  {"P1609", "Immobilizer Communication - Malfunction"},
  {"P1610", "Immobilizer - Smartra Error"},
  {"P1611", "Immobilizer - Transponder Malfunction"},
  {"P1616", "Main Relay Malfunction"},
  {"P1621", "Fuel Cut Valve - Malfunction"},
  {"P1632", "CAN Bus Off"},
  {"P1670", "Injector Classification"},
  {"P1693", "Immobilizer Transponder Error"},
  {"P1701", "TPS"},
  {"P1706", "Inhibitor Switch"},
  {"P1707", "Brake Switch"},
  {"P1745", "TCM K Line"},
  {"P1749", "Serial Communication Link"},
  {"P1805", "Immobilizer EEPROM Error"}
};

// Function to find a DTC description from the PROGMEM table
String getDTCDescription(const char* code) {
  for (unsigned int i = 0; i < (sizeof(dtc_descriptions) / sizeof(DTC)); i++) {
    // Need to read the pointer from PROGMEM first
    char* code_from_progmem = (char*)pgm_read_word(&dtc_descriptions[i].code);
    // Then compare the string using the PROGMEM version of strcmp
    if (strcmp_P(code, code_from_progmem) == 0) {
      char* desc_from_progmem = (char*)pgm_read_word(&dtc_descriptions[i].description);
      return String(desc_from_progmem);
    }
  }
  return ""; // Return empty string if not found
}

// Custom non-blocking function to read from K-Line with watchdog reset
int readKLineResponse(byte* buffer, size_t length, unsigned long timeout) {
  int count = 0;
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    if (KLine.available()) {
      buffer[count++] = KLine.read();
      if (count == length) {
        break; // Buffer is full
      }
    }
    esp_task_wdt_reset(); // Feed the watchdog timer to prevent a reset
    yield();              // Yield to other tasks
  }
  return count;
}

// —— K-LINE & FAULT CODE FUNCTIONS ——
// Helper to convert a nibble to a hex character
char toHex(byte n) {
  n &= 0x0F;
  return n < 10 ? n + '0' : (n - 10) + 'A';
}

// Convert 2-byte ECU response to a standard DTC string (e.g., "P0102")
void parseDTC(char* result, byte b1, byte b2) {
  char c1;
  switch (b1 >> 6) {
    case 0: c1 = 'P'; break; // Powertrain
    case 1: c1 = 'C'; break; // Chassis
    case 2: c1 = 'B'; break; // Body
    case 3: c1 = 'U'; break; // Network
  }
  // Ensure the result is null-terminated by writing at most 6 bytes (5 chars + NUL)
  snprintf(result, 6, "%c%d%c%c%c", c1, (b1 >> 4) & 0x03, toHex(b1), toHex(b2 >> 4), toHex(b2));
}

void readEcuFaults() {
  // First, check if K-Line is idle (HIGH). If not, likely not connected.
  pinMode(K_LINE_RX_PIN, INPUT);
  delay(1);
  if (digitalRead(K_LINE_RX_PIN) == LOW) {
    display.clearDisplay();
    display.setTextColor(SSD1322_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.println("K-Line Connection Error.");
    display.setCursor(10, 32);
    display.println("Check OBD-II connection.");
    display.display();
    messageMode = true;
    messageEndTime = millis() + 3000;
    numFaults = 0;
    return;
  }

  // Display status on screen while reading
  display.clearDisplay();
  display.setTextColor(SSD1322_WHITE);
  display.setTextSize(2);
  display.setCursor(50, 25);
  display.print("Reading DTCs...");
  display.display();
  
  numFaults = 0;
  faultScrollIndex = 0;

  KLine.begin(10400, SERIAL_8N1, K_LINE_RX_PIN, K_LINE_TX_PIN);
  KLine.setTimeout(2000);

  pinMode(K_LINE_TX_PIN, OUTPUT);
  digitalWrite(K_LINE_TX_PIN, LOW);
  delay(25);
  digitalWrite(K_LINE_TX_PIN, HIGH);
  delay(25);
  
  KLine.begin(10400, SERIAL_8N1, K_LINE_RX_PIN, K_LINE_TX_PIN);
  delay(30);
  
  byte request[] = {0x03};
  KLine.write(request, sizeof(request));
  KLine.flush();

  byte response[64];
  int len = readKLineResponse(response, 64, 2000);
  KLine.end();

  if (len == 0) {
    strcpy(faultCodes[0], "No Response From ECU");
    numFaults = 1;
    return;
  }

  if (len > 2 && response[0] == 0x43) {
    int dtcCount = (len - 1) / 2;
    for (int i = 0; i < dtcCount && numFaults < MAX_FAULTS; i++) {
      byte b1 = response[1 + i * 2];
      byte b2 = response[2 + i * 2];
      if (b1 == 0x00 && b2 == 0x00) continue;
      parseDTC(faultCodes[numFaults++], b1, b2);
    }
  }
  
  if (numFaults == 0) {
    strcpy(faultCodes[0], "No Faults Found");
    numFaults = 1;
  }
}

void clearEcuFaults() {
  pinMode(K_LINE_RX_PIN, INPUT);
  delay(1);
  if (digitalRead(K_LINE_RX_PIN) == LOW) {
    display.clearDisplay();
    display.setTextColor(SSD1322_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 20);
    display.println("K-Line Connection Error.");
    display.setCursor(10, 32);
    display.println("Cannot clear codes.");
    display.display();
    messageMode = true;
    messageEndTime = millis() + 3000;
    numFaults = 0;
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1322_WHITE);
  display.setTextSize(2);
  display.setCursor(50, 25);
  display.print("Clearing DTCs...");
  display.display();
  
  KLine.begin(10400, SERIAL_8N1, K_LINE_RX_PIN, K_LINE_TX_PIN);
  KLine.setTimeout(1000);

  pinMode(K_LINE_TX_PIN, OUTPUT);
  digitalWrite(K_LINE_TX_PIN, LOW);
  delay(25);
  digitalWrite(K_LINE_TX_PIN, HIGH);
  delay(25);
  
  KLine.begin(10400, SERIAL_8N1, K_LINE_RX_PIN, K_LINE_TX_PIN);
  delay(30);

  byte request[] = {0x04};
  KLine.write(request, sizeof(request));
  KLine.flush();
  
  byte response[8];
  readKLineResponse(response, 8, 1000); // Use non-blocking read
  KLine.end();
  
  delay(100);
  readEcuFaults();
}

// —— BLE SCAN TASK ——
void bleScanTask(void * parameter) {
  esp_task_wdt_add(NULL); // Add current task to WDT
  while (1) {
    front_left_updated = 0;
    front_right_updated = 0;
    rear_left_updated = 0;
    rear_right_updated = 0;

    // Scan for a shorter duration (4s) than the WDT timeout (10s).
    // The pBLEScan->start() call is blocking.
    pBLEScan->start(4, false);
    pBLEScan->clearResults();
    esp_task_wdt_reset(); // Feed the watchdog immediately after the scan.

    // Delay for 4s before the next scan. This keeps the total loop time
    // around 8s, well within the WDT timeout.
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    esp_task_wdt_reset(); // Feed WDT again before the next blocking scan.
  }
}

// —— HELPERS ——
inline float toVoltage(int adc) { return (adc / ADC_MAX) * VREF; }

inline float readOilPressure() {
  float v_adc = toVoltage(analogRead(OIL_PRESSURE_PIN));
  float v_sensor = v_adc * (OIL_R1 + OIL_R2) / OIL_R2; // Scale back to sensor voltage
  if (v_sensor < oil_min_v) return NAN;
  float p = oil_min_p + (v_sensor - oil_min_v) * (oil_max_p - oil_min_p) / (oil_max_v - oil_min_v);
  p = round(p * 100.0) / 100.0; // Round to 2 decimal places
  if (p < OIL_MIN_P_LIMIT || p > OIL_MAX_P_LIMIT) return NAN;
  return p;
}

inline float readMAP() {
  float v_adc = toVoltage(analogRead(MAP_PIN));
  float v_sensor = v_adc * (MAP_R1 + MAP_R2) / MAP_R2; // Scale back to sensor voltage
  if (v_sensor < map_min_v) return NAN;
  float p = map_min_p + (v_sensor - map_min_v) * (map_max_p - map_min_p) / (map_max_v - map_min_v);
  p = round(p * 100.0) / 100.0; // Round to 2 decimal places
  if (p < MAP_MIN_P_LIMIT || p > MAP_MAX_P_LIMIT) return NAN;
  return p;
}

inline float readBatteryVoltage() {
  const int samples = 10;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(BATTERY_PIN);
  }
  float adc = sum / (float)samples;
  float voltage = round((toVoltage(adc) * (16.0 / 3.3)) * 100.0) / 100.0;
  return voltage;
}

inline float readOilTemp() {
  float t = sensors.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) return NAN;
  t = t + oil_temp_offset;
  if (t < TEMP_MIN_OFFSET || t > TEMP_MAX_OFFSET) return NAN;
  return round(t * 10.0) / 10.0;
}

inline float readIAT() {
  float t = sensors.getTempCByIndex(1);
  if (t == DEVICE_DISCONNECTED_C) return NAN;
  t = t + iat_offset;
  if (t < TEMP_MIN_OFFSET || t > TEMP_MAX_OFFSET) return NAN;
  return round(t * 10.0) / 10.0;
}

inline float readEngineTemp() {
  float t = sensors.getTempCByIndex(2);
  if (t == DEVICE_DISCONNECTED_C) return NAN;
  t = t + engine_temp_offset;
  if (t < TEMP_MIN_OFFSET || t > TEMP_MAX_OFFSET) return NAN;
  return round(t * 10.0) / 10.0;
}

// Helper functions for strings in EEPROM
String loadString(int addr) {
  char buf[TPMS_MAX_LEN];
  for (int i = 0; i < TPMS_MAX_LEN - 1; i++) {
    buf[i] = EEPROM.read(addr + i);
    if (buf[i] == '\0') break;
  }
  buf[TPMS_MAX_LEN - 1] = '\0';
  return String(buf);
}

void saveString(int addr, const String& str) {
  int len = str.length();
  for (int i = 0; i < TPMS_MAX_LEN - 1 && i < len; i++) {
    EEPROM.write(addr + i, str[i]);
  }
  EEPROM.write(addr + std::min(len, TPMS_MAX_LEN - 1), '\0');
}

// Load all values from EEPROM
void loadAll() {
  uint8_t savedDayIndex = EEPROM.read(ADDR_DAY_BRIGHTNESS);
  uint8_t savedNightIndex = EEPROM.read(ADDR_NIGHT_BRIGHTNESS);
  if (savedDayIndex < numBrightnessLevels) dayBrightnessIndex = savedDayIndex;
  if (savedNightIndex < numBrightnessLevels) nightBrightnessIndex = savedNightIndex;
  float temp;
  EEPROM.get(ADDR_OIL_WARN, temp);
  oil_pressure_warn_threshold = constrain(temp, OIL_MIN_P_LIMIT, OIL_MAX_P_LIMIT);
  EEPROM.get(ADDR_OIL_CRIT, temp);
  oil_pressure_crit_threshold = constrain(temp, OIL_MIN_P_LIMIT, OIL_MAX_P_LIMIT);
  EEPROM.get(ADDR_OIL_MIN_V, oil_min_v);
  EEPROM.get(ADDR_OIL_MAX_V, oil_max_v);
  EEPROM.get(ADDR_OIL_MIN_P, temp);
  oil_min_p = constrain(temp, OIL_MIN_P_LIMIT, OIL_MAX_P_LIMIT);
  EEPROM.get(ADDR_OIL_MAX_P, temp);
  oil_max_p = constrain(temp, OIL_MIN_P_LIMIT, OIL_MAX_P_LIMIT);
  EEPROM.get(ADDR_MAP_MIN_V, map_min_v);
  EEPROM.get(ADDR_MAP_MAX_V, map_max_v);
  EEPROM.get(ADDR_MAP_MIN_P, temp);
  map_min_p = constrain(temp, MAP_MIN_P_LIMIT, MAP_MAX_P_LIMIT);
  EEPROM.get(ADDR_MAP_MAX_P, temp);
  map_max_p = constrain(temp, MAP_MIN_P_LIMIT, MAP_MAX_P_LIMIT);
  EEPROM.get(ADDR_OIL_TEMP_OFFSET, temp);
  oil_temp_offset = constrain(temp, TEMP_MIN_OFFSET, TEMP_MAX_OFFSET);
  EEPROM.get(ADDR_IAT_OFFSET, temp);
  iat_offset = constrain(temp, TEMP_MIN_OFFSET, TEMP_MAX_OFFSET);
  EEPROM.get(ADDR_ENG_TEMP_OFFSET, temp);
  engine_temp_offset = constrain(temp, TEMP_MIN_OFFSET, TEMP_MAX_OFFSET);
  tpms_fl = loadString(ADDR_TPMS_FL);
  tpms_fr = loadString(ADDR_TPMS_FR);
  tpms_rl = loadString(ADDR_TPMS_RL);
  tpms_rr = loadString(ADDR_TPMS_RR);
}

// Save all values to EEPROM
void saveAll() {
  EEPROM.write(ADDR_DAY_BRIGHTNESS, (uint8_t)dayBrightnessIndex);
  EEPROM.write(ADDR_NIGHT_BRIGHTNESS, (uint8_t)nightBrightnessIndex);
  EEPROM.put(ADDR_OIL_WARN, oil_pressure_warn_threshold);
  EEPROM.put(ADDR_OIL_CRIT, oil_pressure_crit_threshold);
  EEPROM.put(ADDR_OIL_MIN_V, oil_min_v);
  EEPROM.put(ADDR_OIL_MAX_V, oil_max_v);
  EEPROM.put(ADDR_OIL_MIN_P, oil_min_p);
  EEPROM.put(ADDR_OIL_MAX_P, oil_max_p);
  EEPROM.put(ADDR_MAP_MIN_V, map_min_v);
  EEPROM.put(ADDR_MAP_MAX_V, map_max_v);
  EEPROM.put(ADDR_MAP_MIN_P, map_min_p);
  EEPROM.put(ADDR_MAP_MAX_P, map_max_p);
  EEPROM.put(ADDR_OIL_TEMP_OFFSET, oil_temp_offset);
  EEPROM.put(ADDR_IAT_OFFSET, iat_offset);
  EEPROM.put(ADDR_ENG_TEMP_OFFSET, engine_temp_offset);
  saveString(ADDR_TPMS_FL, tpms_fl);
  saveString(ADDR_TPMS_FR, tpms_fr);
  saveString(ADDR_TPMS_RL, tpms_rl);
  saveString(ADDR_TPMS_RR, tpms_rr);
  EEPROM.commit();
}

void resetMinMax() {
  float* mins[6] = {&oilPressureMin, &mapMin,     &oilTempMin, &iatMin,     &batteryMin, &engineTempMin};
  float* maxs[6] = {&oilPressureMax, &mapMax,     &oilTempMax, &iatMax,     &batteryMax, &engineTempMax};
  for (int i = 0; i < 6; i++) {
    *mins[i] = INFINITY;
    *maxs[i] = -INFINITY;
  }
}

void drawCell(int idx, int cols, int rows, const char* lbl, float val, const char* unit,
              float vmin, float vmax, float recMin, float recMax) {
  int cw = 256 / cols, ch = 64 / rows;
  int x = (idx % cols) * cw, y = (idx / cols) * ch + 1;
  display.setTextSize(1);
  display.setCursor(x, y);
  char buf[32];
  if (!isnan(val)) {
    if (!strcmp(unit, "Bar") || !strcmp(unit, "V"))
      snprintf(buf, 32, "%s %.2f%s %.2f %.2f", lbl, val, unit,
               isinf(recMin) ? vmin : recMin, isinf(recMax) ? vmax : recMax);
    else
      snprintf(buf, 32, "%s %.1f%s %.1f %.1f", lbl, val, unit,
               isinf(recMin) ? vmin : recMin, isinf(recMax) ? vmax : recMax);
  } else {
    snprintf(buf, 32, "%s ERR", lbl);
  }
  display.println(buf);
  int by = y + ch - 9, bw = cw - 2;
  int fh = isnan(val) ? 0 : constrain((int)((val - vmin) / (vmax - vmin) * bw), 0, bw);
  display.drawRect(x + 1, by, bw, 8, SSD1322_WHITE);
  if (fh > 2) display.fillRect(x + 2, by + 1, fh - 2, 6, SSD1322_WHITE);
}

void drawTPMSPage() {
  display.clearDisplay();
  display.setTextColor(SSD1322_WHITE);

  char buf[16];
  const float PSI_TO_BAR = 0.0689476; // Conversion factor from PSI to bar
  int16_t x1, y1; // Declare text bounds variables
  uint16_t w, h;

  // Front Left
  display.setTextSize(2);
  if (front_left_updated) {
    float pressure_bar = front_left_pressure_psi * PSI_TO_BAR;
    snprintf(buf, 16, "%.2f", pressure_bar); // Pressure in bar, 2 decimal places
    display.setCursor(0, 16);
    display.print(buf);
    display.setTextSize(1);
    snprintf(buf, 16, "%d\xB0""C %.1fV", front_left_temperature, front_left_voltage);
    display.setCursor(40, 16); // Beside pressure
    display.print(buf);
  } else {
    display.setCursor(0, 16);
    display.print("N/A");
  }
  display.drawBitmap(50, 13, image_arrow_FL_bits, 50, 6, SSD1322_WHITE);

  // Front Right
  display.setTextSize(2);
  if (front_right_updated) {
    float pressure_bar = front_right_pressure_psi * PSI_TO_BAR;
    snprintf(buf, 16, "%.2f", pressure_bar);
    display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(256 - w, 16); // Right-align
    display.print(buf);
    display.setTextSize(1);
    snprintf(buf, 16, "%d\xB0""C %.1fV", front_right_temperature, front_right_voltage);
    display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(256 - w - 40, 16); // Beside pressure, right-aligned
    display.print(buf);
  } else {
    display.getTextBounds("N/A", 0, 0, &x1, &y1, &w, &h);
    display.setCursor(256 - w, 16);
    display.print("N/A");
  }
  display.drawBitmap(156, 13, image_arrow_FR_bits, 50, 6, SSD1322_WHITE);

  // Rear Left
  display.setTextSize(2);
  if (rear_left_updated) {
    float pressure_bar = rear_left_pressure_psi * PSI_TO_BAR;
    snprintf(buf, 16, "%.2f", pressure_bar);
    display.setCursor(0, 48);
    display.print(buf);
    display.setTextSize(1);
    snprintf(buf, 16, "%d\xB0""C %.1fV", rear_left_temperature, rear_left_voltage);
    display.setCursor(40, 48); // Beside pressure
    display.print(buf);
  } else {
    display.setCursor(0, 48);
    display.print("N/A");
  }
  display.drawBitmap(50, 45, image_arrow_RL_bits, 50, 6, SSD1322_WHITE);

  // Rear Right
  display.setTextSize(2);
  if (rear_right_updated) {
    float pressure_bar = rear_right_pressure_psi * PSI_TO_BAR;
    snprintf(buf, 16, "%.2f", pressure_bar);
    display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(256 - w, 48); // Right-align
    display.print(buf);
    display.setTextSize(1);
    snprintf(buf, 16, "%d\xB0""C %.1fV", rear_right_temperature, rear_right_voltage);
    display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(256 - w - 40, 48); // Beside pressure, right-aligned
    display.print(buf);
  } else {
    display.getTextBounds("N/A", 0, 0, &x1, &y1, &w, &h);
    display.setCursor(256 - w, 48);
    display.print("N/A");
  }
  display.drawBitmap(156, 45, image_arrow_RR_bits, 50, 6, SSD1322_WHITE);

  // Car and Bar Label
  display.drawBitmap(115, 0, image_car_bits, 32, 66, SSD1322_WHITE);
  display.setTextSize(1);
  display.setCursor(119, 2);
  display.print("bar"); // Updated from PSI to bar

  display.display();
}

void drawFaultCodesPage() {
  display.clearDisplay();
  display.setTextColor(SSD1322_WHITE);
  
  // Title
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("ECU Fault Codes");

  if (numFaults > 0) {
    // Show one code and its description at a time
    const char* code = faultCodes[faultScrollIndex];
    String desc = getDTCDescription(code);

    // Draw the Code
    display.setTextSize(2);
    display.setCursor(10, 22);
    display.print(code);

    // Draw the Description
    display.setTextSize(1);
    display.setCursor(10, 42);
    display.print(desc); // Display description if found

    // Show scroll position indicator
    display.setTextSize(1);
    display.setCursor(220, 2);
    char buf[10];
    snprintf(buf, 10, "%d/%d", faultScrollIndex + 1, numFaults);
    display.print(buf);

  } else {
    // This case is for when the page is opened before reading codes
    display.setTextSize(1);
    display.setCursor(0, 25);
    display.print("Press 'Read' (Btn 3) to scan for codes.");
  }

  // On-screen instructions for the buttons
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print("B4:Scroll B3:Read Hold-B3:Clear");
  display.display();
}

void drawPage(Page &pg) {
  display.clearDisplay();
  if (currentPage == TPMS_PAGE) {
    drawTPMSPage();
  } else if (currentPage == FAULT_CODES_PAGE) {
    drawFaultCodesPage();
  } else if (!pg.graph) {
    for (int i = 0; i < pg.count; i++) {
      drawCell(i, 2, pg.count / 2, pg.labels[i], pg.values[i], pg.units[i],
               min0[i], max0[i],
               (i==0?oilPressureMin:i==1?mapMin:i==2?oilTempMin:i==3?iatMin:i==4?batteryMin:engineTempMin),
               (i==0?oilPressureMax:i==1?mapMax:i==2?oilTempMax:i==3?iatMax:i==4?batteryMax:engineTempMax));
    }
  } else {
    display.setCursor(5, 6);
    for (int i = 0, x = 5; i < pg.count; i++, x += 90) {
      char buf[24];
      if (!isnan(pg.values[i])) {
        if (!strcmp(pg.units[i], "Bar"))
          snprintf(buf, 24, "%s: %.2f%s", pg.labels[i], pg.values[i], pg.units[i]);
        else
          snprintf(buf, 24, "%s: %.1f%s", pg.labels[i], pg.values[i], pg.units[i]);
      } else {
        snprintf(buf, 24, "%s: ERR", pg.labels[i]);
      }
      display.setCursor(x, 6);
      display.println(buf);
    }
    int gh = 48, gy = 16;
    display.drawRect(0, gy, GRAPH_WIDTH, gh, SSD1322_WHITE);
    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
      int i1 = (bufferIndex + i) % GRAPH_WIDTH;
      int i2 = (bufferIndex + i + 1) % GRAPH_WIDTH;
      float v1 = pg.buffer[i1], v2 = pg.buffer[i2];
      if (!isnan(v1) && !isnan(v2)) {
        int y1 = gy + gh - constrain((int)((v1 - pg.bufMin) / (pg.bufMax - pg.bufMin) * gh), 0, gh);
        int y2 = gy + gh - constrain((int)((v2 - pg.bufMin) / (pg.bufMax - pg.bufMin) * gh), 0, gh);
        display.drawLine(i, y1, i + 1, y2, SSD1322_WHITE);
      }
    }
  }
  display.display();
}

void displaySplash() {
  display.clearDisplay();
  // Center the 119x64 logo on the 256x64 display (x = (256 - 119) / 2 = 68)
  display.drawBitmap(-2, -1, hyundai_logo_bits, 119, 64, 2);
  // Use FreeSansBold12pt7b font for "TERRACAN"
  display.setTextColor(SSD1322_WHITE);
  display.setTextWrap(false);
  display.setFont(&FreeSansBold12pt7b);
  // Calculate text width for centering
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("TERRACAN", 0, 0, &x1, &y1, &w, &h);
  display.setCursor((256 - w) / 2, splashTextY); // Center text, use Y=60
  display.print("TERRACAN");
  display.display();

  unsigned long start = millis();
  while (millis() - start < splashDuration) {
    buttons[0].update();
    if (buttons[0].rose()) {
      currentPage = PageID((currentPage + 1) % 6);
      display.setFont(NULL); // Reset to default font
      return;
    }
  }
  display.setFont(NULL); // Reset to default font
}

void handleButton(int i) {
  switch (i) {
    case 0: 
      currentPage = PageID((currentPage + 1) % 6); 
      button3LongPress = false; // Reset long press flag when changing pages
      break;
    case 1:
      // Enter deep sleep on button 2 press
      Serial.println("Button 2 pressed, entering deep sleep");
      Serial.print("GPIO 2 state before sleep: "); Serial.println(digitalRead(buttonPins[1]));
      display.clearDisplay();
      display.display();
      esp_deep_sleep_start();
      break;
    case 2:
      buzzerMuted = !buzzerMuted;
      digitalWrite(BUZZER_PIN, LOW);
      break;
    case 3: // Brightness / ECU Read
      if (currentPage == FAULT_CODES_PAGE) {
        // Short press on Btn3 reads codes. Long press is handled in loop().
        if (!button3LongPress) {
          readEcuFaults();
        }
      } else {
        // Original brightness functionality on other pages
        if (isDayMode) {
          dayBrightnessIndex = (dayBrightnessIndex + 1) % numBrightnessLevels;
          display.setContrast(brightnessLevels[dayBrightnessIndex]);
          EEPROM.write(ADDR_DAY_BRIGHTNESS, (uint8_t)dayBrightnessIndex);
          EEPROM.commit();
        } else {
          nightBrightnessIndex = (nightBrightnessIndex + 1) % numBrightnessLevels;
          display.setContrast(brightnessLevels[nightBrightnessIndex]);
          EEPROM.write(ADDR_NIGHT_BRIGHTNESS, (uint8_t)nightBrightnessIndex);
          EEPROM.commit();
        }
      }
      break;
    case 4: // Graph Reset / ECU Scroll
      if (currentPage == FAULT_CODES_PAGE) {
        // Scroll through fault codes one by one
        if (numFaults > 1) { // Only scroll if there's more than one code
          faultScrollIndex++;
          if (faultScrollIndex >= numFaults) {
            faultScrollIndex = 0; // Wrap around to the beginning
          }
        }
      } else {
        // Original graph reset functionality on other pages
        memset(oilPressureBuffer, 0, sizeof oilPressureBuffer);
        memset(mapBuffer, 0, sizeof mapBuffer);
        memset(batteryBuffer, 0, sizeof batteryBuffer);
        bufferIndex = 0;
      }
      break;
  }
}

void serial() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("SET ")) {
      String rest = cmd.substring(4);
      int spacePos = rest.indexOf(' ');
      if (spacePos != -1) {
        String key = rest.substring(0, spacePos);
        String value = rest.substring(spacePos + 1);
        value.trim();
        if (key == "day_brightness") {
          int val = value.toInt();
          if (val >= 0 && val < numBrightnessLevels) {
            dayBrightnessIndex = val;
            if (isDayMode) display.setContrast(brightnessLevels[dayBrightnessIndex]);
            EEPROM.write(ADDR_DAY_BRIGHTNESS, (uint8_t)dayBrightnessIndex);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: day_brightness out of range (0-9)");
          }
        } else if (key == "night_brightness") {
          int val = value.toInt();
          if (val >= 0 && val < numBrightnessLevels) {
            nightBrightnessIndex = val;
            if (!isDayMode) display.setContrast(brightnessLevels[nightBrightnessIndex]);
            EEPROM.write(ADDR_NIGHT_BRIGHTNESS, (uint8_t)nightBrightnessIndex);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: night_brightness out of range (0-9)");
          }
        } else if (key == "oil_warn") {
          float val = value.toFloat();
          if (val >= OIL_MIN_P_LIMIT && val <= OIL_MAX_P_LIMIT) {
            oil_pressure_warn_threshold = val;
            EEPROM.put(ADDR_OIL_WARN, oil_pressure_warn_threshold);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: oil_warn out of range (0.00 to 20.00)");
          }
        } else if (key == "oil_crit") {
          float val = value.toFloat();
          if (val >= OIL_MIN_P_LIMIT && val <= OIL_MAX_P_LIMIT) {
            oil_pressure_crit_threshold = val;
            EEPROM.put(ADDR_OIL_CRIT, oil_pressure_crit_threshold);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: oil_crit out of range (0.00 to 20.00)");
          }
        } else if (key == "oil_min_v") {
          oil_min_v = value.toFloat();
          EEPROM.put(ADDR_OIL_MIN_V, oil_min_v);
          EEPROM.commit();
          Serial.println("OK");
        } else if (key == "oil_max_v") {
          oil_max_v = value.toFloat();
          EEPROM.put(ADDR_OIL_MAX_V, oil_max_v);
          EEPROM.commit();
          Serial.println("OK");
        } else if (key == "oil_min_p") {
          float val = value.toFloat();
          if (val >= OIL_MIN_P_LIMIT && val <= OIL_MAX_P_LIMIT) {
            oil_min_p = val;
            min0[0] = oil_min_p;
            pages[1].bufMin = oil_min_p;
            EEPROM.put(ADDR_OIL_MIN_P, oil_min_p);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: oil_min_p out of range (0.00 to 20.00)");
          }
        } else if (key == "oil_max_p") {
          float val = value.toFloat();
          if (val >= OIL_MIN_P_LIMIT && val <= OIL_MAX_P_LIMIT) {
            oil_max_p = val;
            max0[0] = oil_max_p;
            pages[1].bufMax = oil_max_p;
            EEPROM.put(ADDR_OIL_MAX_P, oil_max_p);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: oil_max_p out of range (0.00 to 20.00)");
          }
        } else if (key == "map_min_v") {
          map_min_v = value.toFloat();
          EEPROM.put(ADDR_MAP_MIN_V, map_min_v);
          EEPROM.commit();
          Serial.println("OK");
        } else if (key == "map_max_v") {
          map_max_v = value.toFloat();
          EEPROM.put(ADDR_MAP_MAX_V, map_max_v);
          EEPROM.commit();
          Serial.println("OK");
        } else if (key == "map_min_p") {
          float val = value.toFloat();
          if (val >= MAP_MIN_P_LIMIT && val <= MAP_MAX_P_LIMIT) {
            map_min_p = val;
            min0[1] = map_min_p;
            pages[2].bufMin = map_min_p;
            EEPROM.put(ADDR_MAP_MIN_P, map_min_p);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: map_min_p out of range (0.00 to 5.00)");
          }
        } else if (key == "map_max_p") {
          float val = value.toFloat();
          if (val >= MAP_MIN_P_LIMIT && val <= MAP_MAX_P_LIMIT) {
            map_max_p = val;
            max0[1] = map_max_p;
            pages[2].bufMax = map_max_p;
            EEPROM.put(ADDR_MAP_MAX_P, map_max_p);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: map_max_p out of range (0.00 to 5.00)");
          }
        } else if (key == "oil_temp_offset") {
          float val = value.toFloat();
          if (val >= TEMP_MIN_OFFSET && val <= TEMP_MAX_OFFSET) {
            oil_temp_offset = val;
            EEPROM.put(ADDR_OIL_TEMP_OFFSET, oil_temp_offset);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: oil_temp_offset out of range (0.00 to 150.00)");
          }
        } else if (key == "iat_offset") {
          float val = value.toFloat();
          if (val >= TEMP_MIN_OFFSET && val <= TEMP_MAX_OFFSET) {
            iat_offset = val;
            EEPROM.put(ADDR_IAT_OFFSET, iat_offset);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: iat_offset out of range (0.00 to 150.00)");
          }
        } else if (key == "eng_temp_offset") {
          float val = value.toFloat();
          if (val >= TEMP_MIN_OFFSET && val <= TEMP_MAX_OFFSET) {
            engine_temp_offset = val;
            EEPROM.put(ADDR_ENG_TEMP_OFFSET, engine_temp_offset);
            EEPROM.commit();
            Serial.println("OK");
          } else {
            Serial.println("ERROR: eng_temp_offset out of range (0.00 to 150.00)");
          }
        } else if (key == "tpms_fl") {
          tpms_fl = value;
          saveString(ADDR_TPMS_FL, tpms_fl);
          EEPROM.commit();
          Serial.println("OK");
        } else if (key == "tpms_fr") {
          tpms_fr = value;
          saveString(ADDR_TPMS_FR, tpms_fr);
          EEPROM.commit();
          Serial.println("OK");
        } else if (key == "tpms_rl") {
          tpms_rl = value;
          saveString(ADDR_TPMS_RL, tpms_rl);
          EEPROM.commit();
          Serial.println("OK");
        } else if (key == "tpms_rr") {
          tpms_rr = value;
          saveString(ADDR_TPMS_RR, tpms_rr);
          EEPROM.commit();
          Serial.println("OK");
        } else {
          Serial.println("ERROR: Unknown key");
        }
      } else {
        Serial.println("ERROR: Invalid command format");
      }
    } else if (cmd == "HELP") {
      Serial.println("Available settings and current values:");
      Serial.print("day_brightness: "); Serial.println(dayBrightnessIndex);
      Serial.print("night_brightness: "); Serial.println(nightBrightnessIndex);
      Serial.print("oil_warn: "); Serial.println(oil_pressure_warn_threshold, 2);
      Serial.print("oil_crit: "); Serial.println(oil_pressure_crit_threshold, 2);
      Serial.print("oil_min_v: "); Serial.println(oil_min_v, 2);
      Serial.print("oil_max_v: "); Serial.println(oil_max_v, 2);
      Serial.print("oil_min_p: "); Serial.println(oil_min_p, 2);
      Serial.print("oil_max_p: "); Serial.println(oil_max_p, 2);
      Serial.print("map_min_v: "); Serial.println(map_min_v, 2);
      Serial.print("map_max_v: "); Serial.println(map_max_v, 2);
      Serial.print("map_min_p: "); Serial.println(map_min_p, 2);
      Serial.print("map_max_p: "); Serial.println(map_max_p, 2);
      Serial.print("oil_temp_offset: "); Serial.println(oil_temp_offset, 2);
      Serial.print("iat_offset: "); Serial.println(iat_offset, 2);
      Serial.print("eng_temp_offset: "); Serial.println(engine_temp_offset, 2);
      Serial.print("tpms_fl: "); Serial.println(tpms_fl);
      Serial.print("tpms_fr: "); Serial.println(tpms_fr);
      Serial.print("tpms_rl: "); Serial.println(tpms_rl);
      Serial.print("tpms_rr: "); Serial.println(tpms_rr);
    } else {
      Serial.println("ERROR: Unknown command. Use SET or HELP");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Woke up from deep sleep or powered on");

  // Initialize watchdog timer
  const esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT,
    .idle_core_mask = (1 << 0) | (1 << 1),
    .trigger_panic = true
  };
  esp_err_t ret = esp_task_wdt_init(&wdt_config);
  if (ret != ESP_OK) {
    Serial.printf("Failed to initialize WDT: %d\n", ret);
  }
  esp_task_wdt_add(NULL); // Add main task to watchdog

  // Initialize EEPROM and load settings
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(ADDR_MAGIC) != EEPROM_MAGIC) {
    // First run, save default values and magic byte
    saveAll();
    EEPROM.write(ADDR_MAGIC, EEPROM_MAGIC);
    EEPROM.commit();
  } else {
    // Subsequent runs, load saved values
    loadAll();
  }

  for (int i = 0; i < 5; i++) {
    buttons[i].attach(buttonPins[i], INPUT); // External pull-down resistors
    buttons[i].interval(DEBOUNCE_DELAY);
    pinMode(buttonPins[i], INPUT);
  }
  pinMode(DAY_NIGHT_PIN, INPUT);
  // Configure button 2 (GPIO 2) for wakeup on HIGH (button pressed)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)buttonPins[1], 1);
  sensors.begin(); sensors.setWaitForConversion(false);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  for (int i = 0; i < GRAPH_WIDTH; i++) {
    oilPressureBuffer[i] = mapBuffer[i] = batteryBuffer[i] = 0;
  }
  resetMinMax();
  if (!display.begin(0x3D)) { while (1); }
  display.setRotation(2);
  isDayMode = digitalRead(DAY_NIGHT_PIN) == HIGH;
  display.setContrast(brightnessLevels[isDayMode ? dayBrightnessIndex : nightBrightnessIndex]);

  // Initialize BLE
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  xTaskCreatePinnedToCore(
    bleScanTask,   /* Task function */
    "BLEScanTask", /* Name of task */
    10000,         /* Stack size */
    NULL,          /* Parameter */
    1,             /* Priority */
    NULL,          /* Task handle */
    0);            /* Pin to core 0 */

  // Update page buffer limits with loaded EEPROM values
  min0[0] = oil_min_p;
  max0[0] = oil_max_p;
  min0[1] = map_min_p;
  max0[1] = map_max_p;
  pages[1].bufMin = oil_min_p;
  pages[1].bufMax = oil_max_p;
  pages[2].bufMin = map_min_p;
  pages[2].bufMax = map_max_p;

  displaySplash();
}

void loop() {
  esp_task_wdt_reset(); // Feed the watchdog
  unsigned long now = millis();
  // If in message mode, just wait for the message to time out.
  if (messageMode) {
    if (millis() >= messageEndTime) {
      messageMode = false; // End message mode
    } else {
      esp_task_wdt_reset(); // Keep feeding watchdog while message is displayed
      return; // Skip the rest of the loop
    }
  }

  for (int i = 0; i < 5; i++) {
    buttons[i].update();
    if (buttons[i].rose()) handleButton(i);
  }

  // Handle long press for clearing fault codes on the specific page
  if (currentPage == FAULT_CODES_PAGE) {
    // buttons[3].read() returns the raw pin state (HIGH if pressed)
    if (buttons[3].read() == HIGH) {
      if (button3PressTime == 0) {
        // Start timer on new press
        button3PressTime = millis();
        button3LongPress = false; // Reset flag
      } else if (millis() - button3PressTime > 5000) {
        // If held for 5 seconds, trigger long press action
        if (!button3LongPress) {
          clearEcuFaults();
          button3LongPress = true; // Set flag to prevent re-triggering and short press action
        }
      }
    } else {
      // Reset timer when button is released
      button3PressTime = 0;
    }
  }

  bool currentMode = digitalRead(DAY_NIGHT_PIN) == HIGH;
  if (currentMode != isDayMode) {
    isDayMode = currentMode;
    display.setContrast(brightnessLevels[isDayMode ? dayBrightnessIndex : nightBrightnessIndex]);
  }

  if (now - lastTempRequest >= TEMP_REQUEST_INTERVAL) {
    lastTempRequest = now;
    sensors.requestTemperatures();
  }

  if (now - lastUpdate >= SCREEN_UPDATE_INTERVAL) {
    lastUpdate = now;
    values0[0] = readOilPressure();
    values0[1] = readMAP();
    values0[4] = readBatteryVoltage();
    oilPressureBuffer[bufferIndex] = values0[0];
    mapBuffer[bufferIndex]         = values0[1];
    batteryBuffer[bufferIndex]     = values0[4];
    bufferIndex = (bufferIndex + 1) % GRAPH_WIDTH;
    drawPage(pages[currentPage]);
  }

  if (!buzzerMuted) {
    float op = values0[0];
    if (!isnan(op)) {
      if (op < oil_pressure_crit_threshold) digitalWrite(BUZZER_PIN, HIGH);
      else if (op < oil_pressure_warn_threshold) {
        if (now - lastBeepTime >= 1000) {
          digitalWrite(BUZZER_PIN, HIGH);
          if (now - lastBeepTime >= 1100) {
            digitalWrite(BUZZER_PIN, LOW);
            lastBeepTime = now;
          }
        }
      } else {
        digitalWrite(BUZZER_PIN, LOW);
        lastBeepTime = now;
      }
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }

  if (now - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    lastSensorUpdate = now;
    values0[2] = readOilTemp();
    values0[3] = readIAT();
    values0[5] = readEngineTemp();
    values1[0] = values0[0]; values1[1] = values0[2]; values1[2] = values0[5];
    values2[0] = values0[1]; values2[1] = values0[3]; values2[2] = values0[0];
    values3[0] = values0[4]; values3[1] = values0[2]; values3[2] = values0[5];

    oilPressureMin = isnan(values0[0]) ? oilPressureMin : min(oilPressureMin, values0[0]);
    oilPressureMax = isnan(values0[0]) ? oilPressureMax : max(oilPressureMax, values0[0]);
    mapMin = isnan(values0[1]) ? mapMin : min(mapMin, values0[1]);
    mapMax = isnan(values0[1]) ? mapMax : max(mapMax, values0[1]);
    oilTempMin = isnan(values0[2]) ? oilTempMin : min(oilTempMin, values0[2]);
    oilTempMax = isnan(values0[2]) ? oilTempMax : max(oilTempMax, values0[2]);
    iatMin = isnan(values0[3]) ? iatMin : min(iatMin, values0[3]);
    iatMax = isnan(values0[3]) ? iatMax : max(iatMax, values0[3]);
    batteryMin = isnan(values0[4]) ? batteryMin : min(batteryMin, values0[4]);
    batteryMax = isnan(values0[4]) ? batteryMax : max(batteryMax, values0[4]);
    engineTempMin = isnan(values0[5]) ? engineTempMin : min(engineTempMin, values0[5]);
    engineTempMax = isnan(values0[5]) ? engineTempMax : max(engineTempMax, values0[5]);
  }
  serial();
}
