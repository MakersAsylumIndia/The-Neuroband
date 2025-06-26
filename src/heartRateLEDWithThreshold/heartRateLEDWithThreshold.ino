#include <math.h>
#include <Wire.h>
#include "MAX30105.h"
#include <VibrationMotor.h>
#include "heartRate.h"


// PINS
const int LEDPin = 9;
// Sensor should be connected SDA to A4 and SCL to A5

// Variables

MAX30105 particleSensor;

// Sensor Shit
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float currentBpm;
int avgBpm;

int bpmThreshold = 80;

void setupHeartRateSensor() {
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void setup() {
  Serial.begin(9600);

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

void loop() {
  updateHeartRateSensor();

  if(avgBpm > bpmThreshold)
  {
    digitalWrite(LEDPin, HIGH);
  }
  else digitalWrite(LEDPin, LOW);
  delay(10);
}
