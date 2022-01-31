#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include "EEPROM.h"
#include "secret.h"

#define EEPROM_SIZE 20
#define EEPROM_UUID_ADDR 0
#define UUID_SIZE 20

String uuid;
TaskHandle_t bmeTask;
TaskHandle_t wifiTask;
Adafruit_BME280 bme;  // I2C

String generateUUID(int length)
{
  String hex[16] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "a", "b", "c", "d", "e", "f"};

  String uuid = "";
  for (int i = 0; i < length; i++)
  {
    uuid += hex[random(0, 16)];
  }
  return uuid;
}

void wifi_connect()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  printf("[+] CONNECTED\n");
  delay(1000);
}

static void readBMELoop(void *arg)
{
  for (;;)
  {
    printf("------------- DATA -------------\n");
    printf("[ ] UUID = %s\n", uuid.c_str());
    printf("[ ] Temperature = %f °C\n", bme.readTemperature());
    printf("[ ] Pressure = %f hPa\n", bme.readPressure() / 100.0F);
    printf("[ ] Humidity = %f %%\n", bme.readHumidity());
    delay(5000);
  }
}

void setup()
{
  delay(500); // Pause for serial setup
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
  if (!status)
  {
    printf("[-] Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ; // Stop
  }

  // Connect to WiFi
  printf("[*] Connecting to %s...\n", WIFI_SSID);
  wifi_connect();

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
}

void loop()
{
  delay(1000);
}
