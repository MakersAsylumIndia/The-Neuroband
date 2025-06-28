#include <math.h>
#include <Wire.h>
#include "MAX30105.h"
#include <VibrationMotor.h>
#include "heartRate.h"


// === Pin Definitions ===
#define SDA_PIN D4           // SDA pin for DOIT ESP32 DEVKIT V1
#define SCL_PIN D5           // SCL pin for DOIT ESP32 DEVKIT V1
const int LEDPin = D6; //15
const int buttonPin = D10; //34

// Variables
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
const int bpmHeadway = 5;

//Button Shit
long lastTimePressed;
const long buttonCoolDown = 50;
const long longPressThreshold = 800; 
long coolDownEndsAt;

bool buttonPressed  =false; bool longPressed = false;
bool button;


// crash out shit
const byte panicHeapSize = 25;
byte userPanics[panicHeapSize];  // Your input array (filled elsewhere)
byte validCount = 0;   // Number of values remaining after outlier removal
byte panicIterator=0;
byte numUserPanics=0;


float computeMean(byte data[], byte size) {
  long sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return (float)sum / size;
}

float computeStdDev(byte data[], byte size, float mean) {
  float variance = 0;
  for (int i = 0; i < size; i++) {
    variance += pow(data[i] - mean, 2);
  }
  return sqrt(variance / size);
}

float computeFilteredAverage(byte input[], byte size, float thresholdMultiplier = 2.0) {
  if (size < 5) return 0;

  float mean = computeMean(input, size);
  float stddev = computeStdDev(input, size, mean);

  float lower = mean - thresholdMultiplier * stddev;
  float upper = mean + thresholdMultiplier * stddev;

  long filteredSum = 0;
  validCount = 0;

  for (int i = 0; i < size; i++) {
    if (input[i] >= lower && input[i] <= upper) {
      filteredSum += input[i];
      validCount++;
    }
  }

  if (validCount == 0) return 0;
  return (float)filteredSum / validCount;
}

void setupHeartRateSensor() {
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found. Please check wiring.");
    while (1);
  }

  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}


void IRAM_ATTR buttonHandler() {
  if(millis() > coolDownEndsAt) {
    button = !button;
    coolDownEndsAt = millis() + buttonCoolDown;
    if(button){
      lastTimePressed = millis();
    }else {
      long duration = millis() - lastTimePressed;
      if(duration > longPressThreshold) longPressed=true;
      else buttonPressed=true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  
  button = false;
	attachInterrupt(buttonPin, buttonHandler, CHANGE);

  setupHeartRateSensor();

  pinMode(LEDPin, OUTPUT);
}

void updateHeartRateSensor() {
  bool isBeat = false;
  long irValue = particleSensor.getIR();
  isBeat = checkForBeat(irValue);

  while (!isBeat)
  {
    if(buttonPressed || longPressed) return;
    irValue = particleSensor.getIR();
    isBeat = checkForBeat(irValue);
  }
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

void reevalThreshold() {
  float avg = computeFilteredAverage(userPanics, numUserPanics);
  if(avg==0) bpmThreshold = 93;
  else bpmThreshold = avg; 
  
  Serial.print("New BPM Threshold : ");
  Serial.println(bpmThreshold);
}

void buttonPress() {
  if(avgBpm> 40&&avgBpm<255){
    userPanics[panicIterator++] = (byte) avgBpm;
    panicIterator %= panicHeapSize;
    if(numUserPanics<panicHeapSize)numUserPanics++;
    Serial.print("Array ");
    for(int i=0; i<panicHeapSize; i++)
    {
      Serial.print(userPanics[i]);
      Serial.print(" ");
    }
    Serial.println();

    bpmThreshold = avgBpm - bpmHeadway;
    Serial.print("New BPM Threshold : ");
    Serial.println(bpmThreshold);
  }
  pulse(5);
}

void loop() {
  updateHeartRateSensor();

  if(avgBpm > bpmThreshold)
  {
    pulse(1);
  }
  if(buttonPressed) {
    Serial.println("Normal Pressed");
    buttonPress();
    buttonPressed = false;
  }else if(longPressed) {
    Serial.println("Long pressed");
    reevalThreshold();
    longPressed = false;
  }
}