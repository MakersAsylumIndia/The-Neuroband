#include <math.h>
#include <Wire.h>
#include "MAX30105.h"
#include <VibrationMotor.h>
#include "heartRate.h"


// PINS
const int LEDPin = 15;
const int buttonPin = 34;

// Variables
//red red brown
bool isPulsing;
MAX30105 particleSensor;

// Sensor Shit
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float currentBpm;
int avgBpm;

int bpmThreshold = 80;

#define SDA_PIN 21           // SDA pin for DOIT ESP32 DEVKIT V1
#define SCL_PIN 22           // SCL pin for DOIT ESP32 DEVKIT V1

int value=1;
int oldVal=1;

void setupHeartRateSensor() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found. Please check wiring.");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x2A);
  particleSensor.setPulseAmplitudeGreen(0); // Green not used
}

void setup() {
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);
  value = digitalRead(buttonPin);
  oldVal = value;

  setupHeartRateSensor();

  pinMode(LEDPin, OUTPUT);
}


void updateHeartRateSensor() {
  
  long irValue = particleSensor.getIR();

  bool isBeat = checkForBeat(irValue);
  // Serial.print("Is there a beat? ");
  // Serial.println(isBeat); 
  if (isBeat)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    currentBpm = 60 / (delta / 1000.0);

    if (currentBpm < 255 && currentBpm > 20)
    {
      rates[rateSpot++] = (byte)currentBpm; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      avgBpm = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        avgBpm += rates[x];
      avgBpm /= RATE_SIZE;
    }
    Serial.println(avgBpm);
  }

}

void buttonPressed() {
  Serial.println("Buttoned!");
}


void loop() {
  updateHeartRateSensor();

  if(avgBpm > bpmThreshold)
  {
    digitalWrite(LEDPin, HIGH);
  }
  else digitalWrite(LEDPin, LOW);

  value = digitalRead(buttonPin);
  if (value != oldVal) {
    if (value == 1) buttonPressed();
    oldVal = value;
  }
  delay(10);
}
