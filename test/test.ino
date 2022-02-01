#define ADC_MAX_VALUE 4096
#define LDR_VOLTAGE 3.3  // 3.3V
#define LDR_RESISTANCE 10000  // 10kΩ

void setup()
{
    // initialize serial communication at 9600 bits per second:
    Serial.begin(115200);
}

int rawToLux(int raw) {
    // Source: https://arduinodiy.wordpress.com/2013/11/03/measuring-light-with-an-arduino/
    double voltage=raw*(LDR_VOLTAGE/ADC_MAX_VALUE);
    int lux=(LDR_RESISTANCE/2)/(2*LDR_VOLTAGE*((LDR_VOLTAGE-voltage)/voltage));
    return int(lux);
}

void loop()
{
    // reads the input on analog pin A0 (value between 0 and 4096)
    int rawData = analogRead(36);

    Serial.print("Raw reading: ");
    Serial.println(rawData); // the raw analog reading

    // Print lux
    Serial.print("Lux: ");
    Serial.println(rawToLux(rawData));

    delay(500);
}
