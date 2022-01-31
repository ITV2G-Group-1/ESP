#include "EEPROM.h"

#define EEPROM_SIZE 20
#define START_ADDR 0x0

void setup() {
    Serial.begin(115200);
    printf("EEPROM: Clearing %d bytes starting from 0x%x\n", EEPROM_SIZE, START_ADDR);

    EEPROM.begin(EEPROM_SIZE);

    for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(START_ADDR+i, 0);
    }
    EEPROM.commit();
}

void loop() {
    delay(1000);
}
