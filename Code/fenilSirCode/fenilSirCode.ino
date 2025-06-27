#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// === Pin Definitions ===
#define SDA_PIN D4           // SDA pin for DOIT ESP32 DEVKIT V1
#define SCL_PIN D5           // SCL pin for DOIT ESP32 DEVKIT V1
#define LED_PIN D10            // Onboard LED pin

// === Global Objects ===
MAX30105 particleSensor;

// === Heart Rate Variables ===
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED off

  Serial.println("Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found. Please check wiring.");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x2A);
  particleSensor.setPulseAmplitudeGreen(0); // Green not used
}

void loop() {
  long irValue = particleSensor.getIR();

  if (irValue < 50000) {
    Serial.println("No finger detected");
    digitalWrite(LED_PIN, LOW);
    delay(200);
    return;
  }

  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute > 20 && beatsPerMinute < 255) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte i = 0; i < RATE_SIZE; i++) {
        beatAvg += rates[i];
      }
      beatAvg /= RATE_SIZE;

      Serial.print("Avg BPM: ");
      Serial.println(beatAvg);

      // Blink LED if BPM is abnormal
      if (beatAvg < 60 || beatAvg > 100) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
    }
  }
  delay(20);
}