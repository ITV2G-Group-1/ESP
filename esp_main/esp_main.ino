#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "time.h"
#include "EEPROM.h"
#include "secret.h"

#define EEPROM_SIZE 20
#define EEPROM_UUID_ADDR 0
#define UUID_SIZE 20

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

String uuid;
WiFiClient client;
TaskHandle_t bmeTask;
TaskHandle_t sendDataTask;
Adafruit_BME280 bme;  // I2C
static QueueHandle_t data_queue;

struct sensor_data {
  String type;  // "temperature"
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

void wifi_connect() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  printf("[+] CONNECTED\n");
  delay(1000);
}

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;

  getLocalTime(&timeinfo);
  time(&now);
  return now;
}

static void readBMELoop(void *arg) {
  for (;;) {
    int time = getTime();

    sensor_data temp_data;
    temp_data.type = "temperature";
    temp_data.value = bme.readTemperature();
    temp_data.timestamp = time;
    xQueueSendToBack(data_queue, &temp_data, 0);

    sensor_data hum_data;
    hum_data.type = "humidity";
    hum_data.value = bme.readHumidity();
    hum_data.timestamp = time;
    xQueueSendToBack(data_queue, &hum_data, 0);

    sensor_data pres_data;
    pres_data.type = "pressure";
    pres_data.value = bme.readPressure() / 100.0F;
    pres_data.timestamp = time;
    xQueueSendToBack(data_queue, &pres_data, 0);

    printf("------------- DATA -------------\n");
    printf("[ ] Time = %d\n", time);
    printf("[ ] Temperature = %f °C\n", temp_data.value);
    printf("[ ] Pressure = %f hPa\n", pres_data.value);
    printf("[ ] Humidity = %f %%\n", hum_data.value);
    delay(3000);
  }
}

static void sendData(void *arg) {
  for (;;) {
    // Connect to socket
    printf("[*] Connecting socket %s:%d...\n", SERVER_HOST, SERVER_PORT);
    while (!client.connect(SERVER_HOST, SERVER_PORT)){
      printf("[-] SOCKET CONNECTION FAILED\n");
      delay(500);
    }

    // Create data
    DynamicJsonDocument doc(1024);
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
    delay(10000);
  }
}

void exit() {
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
    printf("[-] Could not find a valid BME280 sensor, check wiring!");
    exit();
  }

  // Connect to WiFi and socket
  printf("[*] Connecting to %s...\n", WIFI_SSID);
  wifi_connect();

  // Create queue (1000 items)
  data_queue = xQueueCreate(1000, sizeof(struct sensor_data));

  // Synchronize time with NTP server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    printf("Failed to obtain time\n");
    exit();
  }

  // Create tasks
  xTaskCreatePinnedToCore(
      readBMELoop,     // pvTaskCode
      "temp_bme_task", // pcName
      2048,            // usStackDepth
      NULL,            // pvParameters
      1,               // uxPriority
      &bmeTask,        // pvCreatedTask
      app_cpu          // xCoreID
  );
  printf("[+] Created readBMELoop() task\n");
  xTaskCreatePinnedToCore(
      sendData,
      "send_data_task",
      4096,
      NULL,
      1,
      &sendDataTask,
      app_cpu
  );
  printf("[+] Created sendData() task\n");
}

void loop() {
  delay(1000);
}
