#include <math.h>
#include <Wire.h>
#include "MAX30105.h"
#include <VibrationMotor.h>
#include "heartRate.h"


// PINS
const int LEDPin = D7; //15
const int buttonPin = D10; //34

// Variables
//red red brown
bool isPulsing;
MAX30105 particleSensor;

// Sensor Shit
const byte RATE_SIZE = 20; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

// Sensor Reading STuff
float currentBpm;
int avgBpm;
int bpmThreshold = 93;

//Button Shit
long lastTimePressed;
const long buttonCoolDown = 1000;
volatile bool buttonPressed = false;


// crash out shit
const int panicHeapSize = 50;
int userPanics[panicHeapSize];  // Your input array (filled elsewhere)
int validCount = 0;   // Number of values remaining after outlier removal
int numUserPanics=0;

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


void IRAM_ATTR buttonPressHandler() {
  if(millis() - lastTimePressed > buttonCoolDown) {
    lastTimePressed = millis();
    buttonPressed = true;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello");

  pinMode(buttonPin, INPUT_PULLUP);
  
	attachInterrupt(buttonPin, buttonPressHandler, FALLING);

  setupHeartRateSensor();

  pinMode(LEDPin, OUTPUT);
}

void updateHeartRateSensor() {
  bool isBeat = false;
  long irValue = particleSensor.getIR();
  isBeat = checkForBeat(irValue);

  while (!isBeat)
  {
    if(buttonPressed) return;
    irValue = particleSensor.getIR();
    if(irValue < 50000) {
      Serial.println("No finger");
    }
    isBeat = checkForBeat(irValue);
  }
  if (isBeat)
  {
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
  updateHeartRateSensor();

  if(avgBpm > bpmThreshold)
  {
    pulse(1);
  }
  if(buttonPressed) {
    Serial.println("Pressed");
    pulse(5);
    buttonPressed = false;
  }
}
