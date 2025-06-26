#include "SPIFFS.h"
#include <math.h>
#include <Wire.h>
#include "MAX30105.h"
#include <VibrationMotor.h>
#include "heartRate.h"


// PINS
const int redPin = 3, bluePin=6, greenPin=5;
const int buttonPin = 7;
const int motorPin = 9;
// Sensor should be connected SDA to A4 and SCL to A5

// Variables
int panicHeapSize = 50;

MAX30105 particleSensor;
VibrationMotor motor(motorPin); 

// Sensor Shit
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float currentBpm;
int avgBpm;

int userPanics[panicHeapSize];  // Your input array (filled elsewhere)
int validCount = 0;   // Number of values remaining after outlier removal
int numUserPanics=0;

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
  Serial.begin(9600);

  setupHeartRateSensor();

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  setColor(255, 0, 0);

  value = digitalRead(buttonPin);
  oldVal = value;

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
  } else {
    loadPanics();
  }
}

void updateHeartRateSensor() {
  
  long irValue = particleSensor.getIR();

  bool isBeat = checkForBeat(irValue);
  Serial.print("Is there a beat? ");
  Serial.println(isBeat); 
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
    setColor(255, 0, 0);
    pulse(1);
  }

  value = digitalRead(buttonPin);
  if (value != oldVal) {
    if (value == 1) buttonPressed();
    oldVal = value;
  }
  delay(10);
}

void setColor(int r, int g, int b) {
  digitalWrite(redPin, r);   
  digitalWrite(greenPin, g);
  digitalWrite(bluePin, b);
}

void buttonPressed() {
  pulse(5);
  if (numUserPanics < panicHeapSize) {
    userPanics[numUserPanics++] = avgBpm;
  } else {
    for (int i = 1; i < panicHeapSize; i++) {
      userPanics[i - 1] = userPanics[i];
    }
    userPanics[panicHeapSize-1] = avgBpm;
  }

  recalculateThreshold();
  savePanics(); // Save updated array to flash
}

void recalculateThreshold() {
  bpmThreshold = computeFilteredAverage(userPanics, numUserPanics);
  Serial.print("New BPM Threshold: ");
  Serial.println(bpmThreshold);
}

void pulse(int count) {
  for(int i=0;i<count;i++)
  {
    motor.fadeIn();
    delay(100);
    motor.fadeOut();
    delay(100);
  }
}

float computeMean(int data[], int size) {
  long sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  return (float)sum / size;
}

float computeStdDev(int data[], int size, float mean) {
  float variance = 0;
  for (int i = 0; i < size; i++) {
    variance += pow(data[i] - mean, 2);
  }
  return sqrt(variance / size);
}

float computeFilteredAverage(int input[], int size, float thresholdMultiplier = 2.0) {
  if (size < 2) return 0;

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

void savePanics() {
  File file = SPIFFS.open("/panics.dat", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.write((uint8_t*)&numUserPanics, sizeof(numUserPanics));
  file.write((uint8_t*)userPanics, numUserPanics * sizeof(int));

  file.close();
  Serial.println("Saved panic data.");
}

void loadPanics() {
  File file = SPIFFS.open("/panics.dat", FILE_READ);
  if (!file) {
    Serial.println("No existing panic data found.");
    return;
  }
  file.read((uint8_t*)&numUserPanics, sizeof(numUserPanics));
  file.read((uint8_t*)userPanics, numUserPanics * sizeof(int));

  file.close();
  Serial.print("Loaded panic data, count: ");
  Serial.println(numUserPanics);
}
