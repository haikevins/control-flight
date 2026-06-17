// Test Joystick với ESP32-C3 Super Mini
// VRX -> GPIO0
// VRY -> GPIO1
// SW  -> GPIO10

#include <Arduino.h>

#define VRX_PIN 3
#define VRY_PIN 4
#define SW_PIN  20

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(SW_PIN, INPUT_PULLUP); 
    // SW thường nối GND khi nhấn, nên dùng INPUT_PULLUP

    analogReadResolution(12); 
    // ESP32 đọc ADC 12-bit: 0 -> 4095

    Serial.println("Joystick test started");
}

void loop() {
    int xValue = analogRead(VRX_PIN);
    int yValue = analogRead(VRY_PIN);
    int swValue = digitalRead(SW_PIN);

    Serial.print("X: ");
    Serial.print(xValue);

    Serial.print(" | Y: ");
    Serial.print(yValue);

    Serial.print(" | SW: ");
    if (swValue == LOW) {
        Serial.print("PRESSED");
    } else {
        Serial.print("RELEASED");
    }

    if ((xValue < 3950 && xValue > 3850) && (yValue < 3800 && yValue > 3700)) 
    {
        Serial.print(" | Direction: CENTER");
    }

    if (xValue > 3600) 
    {
        if (yValue < 10)
        {
            Serial.println(" | Direction: LEFT");
        }
        else if (yValue > 4000)
        {
            Serial.println(" | Direction: RIGHT");
        }
    }

    if (yValue > 3600) 
    {
        if (xValue > 4000)
        {
            Serial.println(" | Direction: UP");
        }
        else if (xValue < 10)
        {
            Serial.println(" | Direction: DOWN");
        }
    }

    Serial.println();

    delay(200);
}