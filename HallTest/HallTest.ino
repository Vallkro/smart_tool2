/*
  DigitalReadSerial

  Reads a digital input on pin 2, prints the result to the Serial Monitor

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/DigitalReadSerial
*/
#define ENC_COUNT_REV 16

// digital pin 2 has a pushbutton attached to it. Give it a name:
int pushButton = 2;
long previousMillis = 0;
long currentMillis = 0;
volatile long encoderValue = 0;
int interval = 1000;
int rpm = 0;
double angle = 0.0;


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(3), updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(4), updateEncoder, RISING);
  previousMillis = millis();
  volatile long encoderValue = 0;
  // One-second interval for measurements
  int interval = 1000;
  // Counters for milliseconds during interval
  long previousMillis = 0;
  long currentMillis = 0;
  // Variable for RPM measuerment
  int rpm = 0;
}

// the loop routine runs over and over again forever:

void updateEncoder() {
  encoderValue++;
  angle = angle + 60;
  if (angle >= 360) {
  }
}
void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;


    rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);
    encoderValue = 0;
  }

  // print out the state of the button:
  //Serial.print("HU "); Serial.print(buttonState2);
  //Serial.print(",Hw "); Serial.print(buttonState3);
  //Serial.print(",Hv "); Serial.println(buttonState4);
  Serial.print("ANGLE : "); Serial.print(angle);
  Serial.print("  RPM : ");
  Serial.println(rpm);



  delay(1);        // delay in between reads for stability
}
