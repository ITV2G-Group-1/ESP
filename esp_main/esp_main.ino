#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "TinyGPS++.h"
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "time.h"
#include "EEPROM.h"
#include "secret.h"

// Config
#define LDR_PIN 36
#define LIGHT_DELAY 3000  // ms
#define TEMPERATURE_DELAY 3000  // ms
#define GPS_DELAY 3000  // ms
#define SEND_DELAY 10000  // ms

// Hardcoded values
#define EEPROM_SIZE 20
#define EEPROM_UUID_ADDR 0
#define UUID_SIZE 20
#define ADC_MAX_VALUE 4096
#define LDR_VOLTAGE 3.3  // 3.3V
#define LDR_RESISTANCE 10000  // 10kΩ
#define BUILTIN_LED 2

String uuid;
WiFiClient client;
Adafruit_BME280 bme;  // I2C
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
TaskHandle_t gpsTask;
TaskHandle_t tempTask;
TaskHandle_t lightTask;
TaskHandle_t sendDataTask;
QueueHandle_t gps_queue;
QueueHandle_t sensor_queue;

struct sensor_type {
  char* type;  // "temperature"
  float value;  // 21.53
  int timestamp;  // 1643652932
};
struct gps_type {
  char* type;  // "temperature"
  float lat;
  float lng;
  int timestamp;  // 1643652932
};

String generateUUID(int length) {
  String hex[16] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};

  String uuid = "";
  for (int i = 0; i < length; i++)
  {
    uuid += hex[random(0, 16)];
  }
  return uuid;
}

float rawToLux(int raw) {
    // Source: https://arduinodiy.wordpress.com/2013/11/03/measuring-light-with-an-arduino/
    double voltage=raw*(LDR_VOLTAGE/ADC_MAX_VALUE);
    int lux=(LDR_RESISTANCE/2)/(2*LDR_VOLTAGE*((LDR_VOLTAGE-voltage)/voltage));
    return lux;
}

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;

  getLocalTime(&timeinfo);
  time(&now);
  return now;
}

void wifi_connect() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  printf("[+] CONNECTED (%s)\n", WiFi.localIP().toString().c_str());
  delay(1000);
}

static void readGPSLoop(void *arg) {
  for (;;) {
    int time = getTime();

    while (SerialGPS.available() > 0) {
        char c = SerialGPS.read();
        // printf("%c", c);
        gps.encode(c);
    }

    gps_type gps_data;
    gps_data.type = "gps";
    gps_data.lat = gps.location.lat();
    gps_data.lng = gps.location.lng();
    gps_data.timestamp = time;
    xQueueSendToBack(gps_queue, &gps_data, 0);

    printf("[ ] GPS = (%f, %f)\n", gps_data.lat, gps_data.lng);
    delay(GPS_DELAY);
  }
}

static void readTempLoop(void *arg) {
  for (;;) {
    int time = getTime();

    sensor_type temp_data;
    temp_data.type = "temperature";
    temp_data.value = bme.readTemperature();
    temp_data.timestamp = time;
    xQueueSendToBack(sensor_queue, &temp_data, 0);

    printf("[ ] Temperature = %f °C\n", temp_data.value);
    delay(TEMPERATURE_DELAY);
  }
}

static void readLightLoop(void *arg) {
  for (;;) {
    int time = getTime();

    sensor_type light_data;
    light_data.type = "light";
    light_data.value = rawToLux(analogRead(LDR_PIN));
    light_data.timestamp = time;
    xQueueSendToBack(sensor_queue, &light_data, 0);

    printf("[ ] Light = %d lux\n", int(light_data.value));
    delay(LIGHT_DELAY);
  }
}

void errorBlink() {
  digitalWrite(BUILTIN_LED, HIGH);
  delay(200);
  digitalWrite(BUILTIN_LED, LOW);
  delay(200);
  digitalWrite(BUILTIN_LED, HIGH);
  delay(200);
  digitalWrite(BUILTIN_LED, LOW);
  delay(200);
}

static void sendData(void *arg) {
  for (;;) {
    // Connect to socket
    printf("[~] Connecting socket %s:%d...\n", SERVER_HOST, SERVER_PORT);
    while (!client.connect(SERVER_HOST, SERVER_PORT)){
      printf("[-] SOCKET CONNECTION FAILED\n");
      errorBlink();
      delay(1000);
    }
    printf("[+] Connected to socket!\n");

    // Create data
    DynamicJsonDocument doc(2048);
    doc["uuid"] = uuid;
    JsonArray data = doc.createNestedArray("data");
    JsonObject json_data;

    // Get data from queue
    sensor_type sensor_data;
    while (xQueueReceive(sensor_queue, &sensor_data, 0) == pdTRUE) {
      json_data = data.createNestedObject();
      json_data["type"] = sensor_data.type;
      json_data["value"] = sensor_data.value;
      json_data["timestamp"] = sensor_data.timestamp;
    }
    gps_type gps_data;
    while (xQueueReceive(gps_queue, &gps_data, 0) == pdTRUE) {
      json_data = data.createNestedObject();
      json_data["type"] = gps_data.type;
      json_data["lat"] = gps_data.lat;
      json_data["lng"] = gps_data.lng;
      json_data["timestamp"] = gps_data.timestamp;
    }

    // Send data
    String json;
    serializeJson(doc, json);
    client.print(json);
    client.stop();
    delay(SEND_DELAY);
  }
}

void restart() {
  printf("Restarting...\n");
  delay(5000);
  ESP.restart();
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);  // Status LED
  digitalWrite(BUILTIN_LED, HIGH);  // LED on

  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  // Get CPU number
  int app_cpu = 0;
  app_cpu = xPortGetCoreID();
  printf("[+] app_cpu is %d (%s core)\n",
         app_cpu,
         app_cpu > 0 ? "Dual" : "Single");

  // Get UUID from EEPROM
  for (int i = 0; i < UUID_SIZE; i++) {
    byte letter = EEPROM.read(EEPROM_UUID_ADDR+i);

    if (letter == 0) break;  // If end of string

    uuid += char(letter);
  }
  // If not found, create new UUID
  if (uuid.length() < UUID_SIZE) {
    uuid = generateUUID(UUID_SIZE);  // Generate new one
    // Save to EEPROM
    for (int i = 0; i < UUID_SIZE; i++) {
      EEPROM.write(EEPROM_UUID_ADDR+i, uuid[i]);
    }
    EEPROM.commit();
  }
  printf("[+] UUID: %s\n", uuid.c_str());

  // Test BME280
  bool status = bme.begin(0x76);
  if (!status)   {
    printf("[-] Could not find a valid BME280 sensor, check wiring!\n");
    restart();
  }

  // Connect to WiFi and socket
  printf("[~] Connecting to %s...\n", WIFI_SSID);
  wifi_connect();

  // Create queue (2048 items)
  sensor_queue = xQueueCreate(2048, sizeof(struct sensor_type));
  gps_queue = xQueueCreate(2048, sizeof(struct gps_type));

  // Synchronize time with NTP server
  configTime(3600, 3600, "0.europe.pool.ntp.org", "1.europe.pool.ntp.org", "2.europe.pool.ntp.org");

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    printf("[-] Failed to obtain time\n");
    restart();
  }

  // Create tasks
  xTaskCreatePinnedToCore(
      readGPSLoop,     // pvTaskCode
      "read_gps_task", // pcName
      2048, NULL, 1, &gpsTask, app_cpu);
  printf("[+] Created readGPSLoop() task\n");
  xTaskCreatePinnedToCore(
      readTempLoop,      // pvTaskCode
      "read_temp_task",  // pcName
      2048, NULL, 1, &tempTask, app_cpu);
  printf("[+] Created readTempLoop() task\n");
  xTaskCreatePinnedToCore(
      readLightLoop,     // pvTaskCode
      "read_light_task", // pcName
      2048, NULL, 1, &lightTask, app_cpu);
  printf("[+] Created readLightLoop() task\n");
  xTaskCreatePinnedToCore(
      sendData,          // pvTaskCode
      "send_data_task",  // pcName
      4096, NULL, 1, &sendDataTask, app_cpu);
  printf("[+] Created sendData() task\n");

  digitalWrite(BUILTIN_LED, LOW);  // LED off
}

void loop() {
  delay(1000);
}
