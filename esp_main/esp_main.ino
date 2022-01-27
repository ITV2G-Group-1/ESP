#include <WiFi.h>
#include <HTTPClient.h>
#include "secret.h"

// data below is set as a define in the file secret.h
const char* ssid       = WIFI_SSID;
const char* password   = WIFI_PASSWORD;

const int tempPin = 32;

int tempVal;
float volts;
float temp;
TaskHandle_t tempTask;
TaskHandle_t wifiTask;

void wifi_connect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      printf(".");
  }
  printf(" CONNECTED\r\n");
  delay(1000);
}

void postAPI(String api, String data) {
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
    printf("Error on HTTP request\n");
  }
}

float getTemp() {
  tempVal = analogRead(tempPin); // read sensor
  volts = tempVal / 1023.0; // calculate volts
  temp = (volts - 0.5) * 100 ; // calculate temperature in Celcius

  return temp;
}

static void readTempLoop(void *arg) {
  for (;;) {
    printf("[ ] Temperature: %f °C\n", getTemp()); // print temperature
    delay(1000);
  }
}

void setup() {
  int app_cpu = 0; // CPU number

  delay(500); // Pause for serial setup

  app_cpu = xPortGetCoreID();
  printf("[+] app_cpu is %d (%s core)\n",
         app_cpu,
         app_cpu > 0 ? "Dual" : "Single");

  printf("Connecting to %s", ssid);
  wifi_connect();

  postAPI("temperature", String(getTemp()));

  xTaskCreatePinnedToCore(
    readTempLoop,  // pvTaskCode
    "temp_read_task",  // pcName
    2048,  // usStackDepth
    NULL,  // pvParameters
    1,  // uxPriority
    &tempTask,  // pvCreatedTask
    app_cpu  // xCoreID
  );
  printf("[+] Created readTemp() task\n");
}

void loop() {
  delay(2000);
}
