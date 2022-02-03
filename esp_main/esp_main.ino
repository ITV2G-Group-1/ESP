#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "time.h"
#include "EEPROM.h"
#include "secret.h"

// Config
#define LDR_PIN 36
#define LIGHT_DELAY 3000  // ms
#define TEMPERATURE_DELAY 3000  // ms
#define SEND_DELAY 10000  // ms

// Hardcoded values
#define EEPROM_SIZE 20
#define EEPROM_UUID_ADDR 0
#define UUID_SIZE 20
#define ADC_MAX_VALUE 4096
#define LDR_VOLTAGE 3.3  // 3.3V
#define LDR_RESISTANCE 10000  // 10kΩ

String uuid;
WiFiClient client;
TaskHandle_t tempTask;
TaskHandle_t lightTask;
TaskHandle_t sendDataTask;
Adafruit_BME280 bme;  // I2C
QueueHandle_t data_queue;

struct sensor_data {
  char* type;  // "temperature"
  float value;  // 21.53
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

static void readTempLoop(void *arg) {
  for (;;) {
    int time = getTime();

    sensor_data temp_data;
    temp_data.type = "temperature";
    temp_data.value = bme.readTemperature();
    temp_data.timestamp = time;
    xQueueSendToBack(data_queue, &temp_data, 0);

    printf("[ ] Temperature = %f °C\n", temp_data.value);
    delay(TEMPERATURE_DELAY);
  }
}

static void readLightLoop(void *arg) {
  for (;;) {
    int time = getTime();

    sensor_data light_data;
    light_data.type = "light";
    light_data.value = rawToLux(analogRead(LDR_PIN));
    light_data.timestamp = time;
    xQueueSendToBack(data_queue, &light_data, 0);

    printf("[ ] Light = %d lux\n", int(light_data.value));
    delay(LIGHT_DELAY);
  }
}

static void sendData(void *arg) {
  for (;;) {
    // Connect to socket
    printf("[~] Connecting socket %s:%d...\n", SERVER_HOST, SERVER_PORT);
    while (!client.connect(SERVER_HOST, SERVER_PORT)){
      printf("[-] SOCKET CONNECTION FAILED\n");
      delay(500);
    }
    printf("[+] Connected to socket!\n");

    // Create data
    DynamicJsonDocument doc(2048);
    doc["uuid"] = uuid;
    JsonArray data = doc.createNestedArray("data");

    // Get data from queue
    sensor_data queue_data;
    JsonObject json_data;
    while (xQueueReceive(data_queue, &queue_data, 0) == pdTRUE) {
      json_data = data.createNestedObject();
      json_data["type"] = queue_data.type;
      json_data["value"] = queue_data.value;
      json_data["timestamp"] = queue_data.timestamp;
    }

    // Send data
    String json;
    serializeJson(doc, json);
    client.print(json);
    client.stop();
    delay(SEND_DELAY);
  }
}

void exit() {
  printf("Exiting...\n");
  while (1);  // Infinite loop
}

void setup() {
  delay(500); // Pause for serial setup
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

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
    exit();
  }

  // Connect to WiFi and socket
  printf("[~] Connecting to %s...\n", WIFI_SSID);
  wifi_connect();

  // Create queue (2048 items)
  data_queue = xQueueCreate(2048, sizeof(struct sensor_data));

  // Synchronize time with NTP server
  configTime(3600, 3600, "0.europe.pool.ntp.org", "1.europe.pool.ntp.org", "2.europe.pool.ntp.org");

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    printf("[-] Failed to obtain time\n");
    exit();
  }

  // Create tasks
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
}

void loop() {
  delay(1000);
}
