const int tempPin = 2;

int tempVal;
float volts;
float temp;
TaskHandle_t tempTask;

static void readTemp(void *arg) {
  for (;;) {
    tempVal = analogRead(tempPin);
    volts = tempVal / 1023.0;
    temp = (volts - 0.5) * 100 ;

    printf("[*] Temperature: %f °C\n", temp);
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

  xTaskCreatePinnedToCore(
    readTemp,
    "temp_read_task",
    2048,
    NULL,
    1,
    &tempTask,
    app_cpu
  );
  printf("[+] Created readTemp() task\n");
}

void loop() {
  delay(2000);
}
