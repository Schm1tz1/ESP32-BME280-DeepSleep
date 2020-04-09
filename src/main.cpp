//
// ESP32 in DeepSleep reading a BME280 every few seconds
//

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_BME280.h>

#define SERIAL_BAUD 115200
#define SLEEP_IN_US 5000000

RTC_DATA_ATTR long globalCounter = 1;

struct weatherData{
    float temperature;
    float pressure;
    float humidity;
};

void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0 :
            Serial.println("Wakeup caused by external signal using RTC_IO");
            break;
        case ESP_SLEEP_WAKEUP_EXT1 :
            Serial.println("Wakeup caused by external signal using RTC_CNTL");
            break;
        case ESP_SLEEP_WAKEUP_TIMER :
            Serial.println("Wakeup caused by timer");
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD :
            Serial.println("Wakeup caused by touchpad");
            break;
        case ESP_SLEEP_WAKEUP_ULP :
            Serial.println("Wakeup caused by ULP program");
            break;
        default :
            Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
            break;
    }
}

struct weatherData readSensor(uint8_t bmeAddress = 0x76) {
    Adafruit_BME280 bme; // I2C

    while (!bme.begin(bmeAddress)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        delay(1000);
    }

    struct weatherData measurement;
    measurement.temperature = bme.readTemperature();
    measurement.pressure = bme.readPressure() / 100.0F;
    measurement.humidity = bme.readHumidity();

    return measurement;
}

void printValues(struct weatherData dataToPrint) {
    Serial.print("Temperature = ");
    Serial.print(dataToPrint.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(dataToPrint.pressure);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(dataToPrint.humidity);
    Serial.println(" %");

    Serial.println();
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);

    print_wakeup_reason();
    const struct weatherData newValues = readSensor();
    printValues(newValues);

    Serial.println("Going to sleep.");

    // this is hibernate mode (deep sleep with additional components turned off)
    esp_sleep_enable_timer_wakeup(SLEEP_IN_US);
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
}

void loop() {

}
