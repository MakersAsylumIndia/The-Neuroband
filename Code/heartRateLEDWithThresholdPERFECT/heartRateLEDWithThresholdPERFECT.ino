#include <math.h>
#include <Wire.h>
#include "MAX30105.h"
#include <VibrationMotor.h>
#include "heartRate.h"


// PINS
const int LEDPin = 15;
const int buttonPin = 34;
int currentVal, oldVal;

// Variables
//red red brown
bool isPulsing;
MAX30105 particleSensor;

// Sensor Shit
const byte RATE_SIZE = 20; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float currentBpm;
int avgBpm;

int bpmThreshold = 93;


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
  currentVal = digitalRead(buttonPin);
  oldVal = currentVal;

  setupHeartRateSensor();

  pinMode(LEDPin, OUTPUT);
}


void updateHeartRateSensor() {
  bool isBeat = false;
  long irValue = particleSensor.getIR();
  isBeat = checkForBeat(irValue);

  while (!isBeat)
  {
    irValue = particleSensor.getIR();
    isBeat = checkForBeat(irValue);
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
    // Serial.print("Current: ");
    // Serial.println(currentBpm);
    // Serial.print(" Average: ");
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
        delay(5);
      }
      for(int i=255;i>=0;i--)
      {
        analogWrite(LEDPin, i);
        delay(5);
      }
    }
    isPulsing = false;
    lastBeat = millis() - (60000/avgBpm);
    avgBpm = bpmThreshold-1;
  }
}

void loop() {
  
  currentVal = digitalRead(buttonPin);
  if(currentVal !=  oldVal)
  {
    Serial.print("Button: ");
    Serial.println(currentVal);
    oldVal = currentVal;
  }

  updateHeartRateSensor();

  if(avgBpm > bpmThreshold)
  {
    pulse(1);
  }

}
