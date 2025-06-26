const int buttonPin = 34;
int currentVal, oldVal;

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  currentVal = digitalRead(buttonPin);
  oldVal = currentVal;
}

void loop() {
  // put your main code here, to run repeatedly:
  currentVal = digitalRead(buttonPin);
  if(currentVal !=  oldVal)
  {
    Serial.println("Buttoned");
    oldVal = currentVal;
  }
  delay(50);
}
