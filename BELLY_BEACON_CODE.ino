#define BLYNK_TEMPLATE_ID "TMPL3blfZFb8r"
#define BLYNK_TEMPLATE_NAME "bellybeacon"
#define BLYNK_AUTH_TOKEN "SCyiPnmXBmonVc6bO3X_yJ0ZjGWTTJyg"

#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

MAX30105 particleSensor;

const char* ssid = "vivo 1904";
const char* password = "14141919"; // Add your Wi-Fi password here
char auth[] = BLYNK_AUTH_TOKEN;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

uint32_t irBuffer[100];
uint32_t redBuffer[100];
int bufferLength = 100;

int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

const int flexPin = 32;
const int pressurePin = 34;
int flexValue;
int pressureValue;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing...");

    // Initialize Wi-Fi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi Connected");

    // Initialize Blynk
    Blynk.begin(auth, ssid, password);

    // Initialize sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 was not found. Please check wiring/power.");
        while (1);
    }

    Serial.println("Place your index finger on the sensor with steady pressure.");
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeIR(0x1F);
    pinMode(flexPin, INPUT);
    pinMode(pressurePin, INPUT);
}

void loop()
{
    flexValue = analogRead(flexPin);
    Serial.print("Flex Sensor Value: ");
    Serial.println(flexValue);

    pressureValue = analogRead(pressurePin);
    if (pressureValue > 10) {
        Serial.println("High pressure detected - Check digestive health!");
        Blynk.logEvent("pressure_alert", "High pressure detected!");
    } else {
        Serial.println("Normal pressure");
    }

    // Collect 100 samples of both IR and Red LED
    for (int i = 0; i < bufferLength; i++) {
        while (particleSensor.available() == false)
            particleSensor.check();

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();

        particleSensor.nextSample();
    }

    // Calculate heart rate and SpO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    Serial.print("Heart Rate: ");
    if (validHeartRate) {
        Serial.print(heartRate);
        Blynk.virtualWrite(V0, heartRate); // Send Heart Rate to Virtual Pin 0
    } else {
        Serial.print("Invalid");
    }

    Serial.print(" bpm, SpO2: ");
    if (validSPO2) {
        Serial.print(spo2);
        Blynk.virtualWrite(V1, spo2); // Send SpO2 to Virtual Pin 1
    } else {
        Serial.print("Invalid");
    }

    Serial.println();

    if (validHeartRate && heartRate > 100) {
        Blynk.logEvent("stomach_alert", "DIGESTION PROBLEM DETECTED;");
    }

    Blynk.run();
    delay(1000);
}
