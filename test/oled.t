#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED I2C pins
#define SDA_PIN 8
#define SCL_PIN 9

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(SDA_PIN, SCL_PIN);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("Khong tim thay OLED!");
        while (true);
    }

    Serial.println("OLED da khoi dong!");

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ESP32-C3 OLED Test");

    display.setCursor(0, 16);
    display.println("SDA: GPIO 8");

    display.setCursor(0, 28);
    display.println("SCL: GPIO 9");

    display.setCursor(0, 44);
    display.println("Hello Hai!");

    display.display();
}

void loop() {
    static int count = 0;

    display.clearDisplay();

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("OLED dang chay");

    display.setCursor(0, 20);
    display.print("Dem: ");
    display.println(count);

    display.setCursor(0, 40);
    display.println("ESP32-C3 Super Mini");

    display.display();

    count++;
    delay(1000);
}