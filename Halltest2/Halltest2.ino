/*

This example shows how to measure motor's RPM through built-in
hall sensor encoder.

Product page:
  DC Motor IG42E-24K: https://www.cytron.io/c-84-dc-motor/c-91-dc-geared-motor/c-494-ig42-with-encoder-planetary-gear/p-ig42e-24k
  Cytron 10Amp DC Motor Driver: https://www.cytron.io/p-md10c
  Maker UNO: https://www.cytron.io/p-maker-uno

Created by:
  2/07/18  Idris, Cytron Technologies

 */

// Please insert your motor encoder output pulse per rotation
#define ENCODEROUTPUT 120

#define BUTTON 2 // Maker UNO built-in button connected to pin 2
#define HALLSEN_A 3 // Hall sensor A connected to pin 3 (external interrupt)
#define BUZZER  8 // Maker UNO built-in buzzer connected to pin 8
#define PWM 10 // MD10C PWM connected to pin 10
#define DIR 12 // MD10C DIR connected to pin 12

// Note and frequencies
#define NOTE_G4  392
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_G5  784

int melody[] = {
  NOTE_G4, NOTE_C5, NOTE_E5, NOTE_G5, 0, NOTE_E5, NOTE_G5
};

int noteDurations[] = {
  10, 10, 10, 10, 10, 10, 4
};

volatile long encoderValue = 0;

int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

int rpm = 0;
boolean measureRpm = false;
int motorPwm = 0;

void setup()
{
  Serial.begin(9600); // Initialize serial with 9600 baudrate

  pinMode(BUTTON, INPUT_PULLUP); // Set button as input pullup
  pinMode(HALLSEN_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(BUZZER, OUTPUT); // Set buzzer as output
  pinMode(PWM, OUTPUT); // Set PWM pin as output
  pinMode(DIR, OUTPUT); // Set DIR pin as output
  
  // Attach interrupt at hall sensor A on each rising signal
  attachInterrupt(digitalPinToInterrupt(HALLSEN_A), updateEncoder, RISING);

  
}

void loop()
{
   // If button is pressed...
    measureRpm = !measureRpm;
    
    
    
    delay(500);
    encoderValue = 0;
    previousMillis = millis();
  

  // Update RPM value on every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    
    

    // Revolutions per minute (RPM) =
    // (total encoder pulse in 1s / motor encoder output) x 60s
    rpm = (float)(encoderValue * 60 / ENCODEROUTPUT);

    // Only update display when there have readings
   
      Serial.print(motorPwm);
      Serial.print('\t');
      Serial.print(encoderValue);
      Serial.print(" pulse / ");
      Serial.print(ENCODEROUTPUT);
      Serial.print(" pulse per rotation x 60 seconds = ");
      Serial.print(rpm);
      Serial.println(" RPM");
    
    
    encoderValue = 0;
  }
}

void updateEncoder()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue++;
  Serial.print(encoderValue);
}

void playMelody()
{
 
}
