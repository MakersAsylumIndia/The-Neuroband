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


int value=1;
int oldVal=1;

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
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);
  value = digitalRead(buttonPin);
  oldVal = value;

  setupHeartRateSensor();

  pinMode(LEDPin, OUTPUT);
}


void updateHeartRateSensor() {
  lastBeat = millis();
  bool isBeat = false;
  long irValue = particleSensor.getIR();
  isBeat = checkForBeat(irValue);
  while (!isBeat)
  {
    irValue = particleSensor.getIR();
    isBeat = checkForBeat(irValue);
    delay(10);
  }
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
    Serial.print("Current: ");
    Serial.print(currentBpm);
    Serial.print(" Average: ");
    Serial.println(avgBpm);
  }

}

void buttonPressed() {
  Serial.println("Buttoned!");
}

void pulse(int count) {
  if(!isPulsing){
    isPulsing = true;
    for(int c=0; c<count; c++){
      for(int i=0;i<256;i++)
      {
        analogWrite(LEDPin, i);
        delay(10);
      }
      delay(1000);
      for(int i=255;i>=0;i--)
      {
        analogWrite(LEDPin, i);
        delay(10);
      }
      delay(1000);
    }
    isPulsing = false;
  }
}

void loop() {
  updateHeartRateSensor();

  if(avgBpm > bpmThreshold)
  {
    pulse(1);
  }
  else digitalWrite(LEDPin, LOW);

  value = digitalRead(buttonPin);
  if (value != oldVal) {
    if (value == 1) buttonPressed();
    oldVal = value;
  }
  delay(10);
}
