#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

// Pin Definitions
const int buttonPin = 34;
const int vibrationMotorPin = 9;

bool systemActive = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(buttonPin, INPUT); // GPIO34 is input-only
  pinMode(vibrationMotorPin, OUTPUT);
  digitalWrite(vibrationMotorPin, LOW);

  // Initialize I2C with custom pins
  Wire.begin(21, 22); // SDA = 21, SCL = 22

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found. Check wiring.");
    while (1);
  }

  particleSensor.setup(); // Sensor config
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0); // turn off green LED

  Serial.println("System ready. Press button to begin.");
}

void loop() {
  // Wait for button press to start measurement
  if (!systemActive && digitalRead(buttonPin) == HIGH) {
    Serial.println("Button pressed. Starting heart rate measurement...");
    systemActive = true;
    delay(300); // debounce
  }

  // If active, read IR value and detect heartbeat
  if (systemActive) {
    long irValue = particleSensor.getIR();

    if (irValue > 50000) {
      if (checkForBeat(irValue)) {
        float bpm = beatsPerMinute;

        if (bpm > 30 && bpm < 180) {
          Serial.print("BPM Detected: ");
          Serial.println(bpm);

          digitalWrite(vibrationMotorPin, HIGH);
          delay(500); // vibrate for half a second
          digitalWrite(vibrationMotorPin, LOW);

          Serial.println("Measurement complete. Press button to restart.");
          systemActive = false;
        }
      }
    }
  }
}