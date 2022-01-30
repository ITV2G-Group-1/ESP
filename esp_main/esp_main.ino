#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "secret.h"

// data below is set as a define in the file secret.h
const char* ssid       = WIFI_SSID;
const char* password   = WIFI_PASSWORD;

TaskHandle_t bmeTask;
TaskHandle_t wifiTask;
Adafruit_BME280 bme; // I2C

void wifi_connect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
  }
  printf("[+] CONNECTED\n");
  delay(1000);
}

void postAPI(String api, String data) {
  // postAPI("temperature", "21.53");
  /*
  POST /api/[api]
  Authorization: Bearer [token]

  [data]
  */
  HTTPClient http;

  http.begin(SERVER_HOST + "api/" + api);
  http.addHeader("Authorization", "Bearer " + BEARER_TOKEN);

  int httpCode = http.POST(data);
  if (httpCode > 0) {
    String response = http.getString();
    printf("%s\n", response.c_str());
  } else {
    printf("[-] Error on HTTP request\n");
  }
}

static void readBMELoop(void *arg) {
  for (;;) {
    printf("-----------------------\n");
    printf("[ ] Temperature = %f °C\n", bme.readTemperature());
    printf("[ ] Pressure = %f hPa\n", bme.readPressure() / 100.0F);
    printf("[ ] Humidity = %f %\n", bme.readHumidity());
    delay(5000);
  }
}

void setup() {
  delay(500); // Pause for serial setup

  // Get CPU number
  int app_cpu = 0;
  app_cpu = xPortGetCoreID();
  printf("[+] app_cpu is %d (%s core)\n",
         app_cpu,
         app_cpu > 0 ? "Dual" : "Single");

  // Test BME280
  bool status = bme.begin(0x76);  
  if (!status) {
    printf("[-] Could not find a valid BME280 sensor, check wiring!");
    while (1);  // Stop
  }

  // Connect to WiFi
  printf("[*] Connecting to %s...\n", ssid);
  wifi_connect();

  // Create tasks
  xTaskCreatePinnedToCore(
    readBMELoop,  // pvTaskCode
    "temp_bme_task",  // pcName
    2048,  // usStackDepth
    NULL,  // pvParameters
    1,  // uxPriority
    &bmeTask,  // pvCreatedTask
    app_cpu  // xCoreID
  );
  printf("[+] Created readBMELoop() task\n");
}

void loop() {
  delay(1000);
}
