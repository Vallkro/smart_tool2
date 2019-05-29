#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
// i2c
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK 13
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI 11
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
#define SPEED_1 10
#define SPEED_2 9
#define SPEED_3 8
#define SPEED_4 7
#define SPEED_5 1 // doesnt work properly
#define REV_ON 0
#define HU 4
#define HW 2
#define HV 3
#define dataToSendLength 34
#define GEARBOX_RATIO 10  //needs tweaking
#define piNum 3.141592653589793

//i2c
#define SLAVE_ADDRESS 0x05
//omega
#define ENC_COUNT_REV 16

typedef union
{
  short number;
  uint8_t bytes[2];
} SHORTUNION_t;
typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;
SHORTUNION_t myShort;
SHORTUNION_t myShort2;
SHORTUNION_t omegaAxle;

float torqueTest = 0;
int anglePlot = 0;
float currentValue2 = 0;





// software SPI
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

//setting up global vars;
char number[50];
char state = 'i';
bool done = false;
byte stage = -1;
short indexR = 0;
short int counter = 0;


#define FLOATS_SENT 1
float data[FLOATS_SENT];
char dataToSend[dataToSendLength];
//IMU
FLOATUNION_t  accelerationX;
FLOATUNION_t  accelerationY;
FLOATUNION_t  accelerationZ;
float  magneticX;
float  magneticY;
float  magneticZ;

FLOATUNION_t  gyroX;
FLOATUNION_t  gyroY;
FLOATUNION_t  gyroZ;
FLOATUNION_t  torqueUni;

float  v1;
// CURRENT SENSOR
const int numReadings = 10;
float readings[numReadings];      // the readings from the analog input
int indexI = 0;                  // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average

float currentValue = 0;


//for omega and angle.number
volatile long encoderValue = 0;
volatile long encoderValuePrev = 0;
int interval = 5;            //  interval for measurements
volatile float previousMillis = 0;          // Counters for milliseconds during interval
volatile float currentMillis = 0;
volatile float omega = 0;              // Variable for omega measuerment
volatile long angleMotor = 0;
volatile long prevAngleMotor = 0;
volatile SHORTUNION_t angle;
volatile int prevAngle = 0;



//sequence vars
bool pause = true;
long currentSequenceMillis = 0;
long prevSequenceMillis = 0;


void setupSensor()
{
  //gyro setup
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);


}

void setup()
{
  angle.number = 0;
  angleMotor = 0;
  pinMode(SPEED_1, OUTPUT);
  pinMode(SPEED_2, OUTPUT);
  pinMode(SPEED_3, OUTPUT);
  pinMode(SPEED_4, OUTPUT);
  pinMode(SPEED_5, OUTPUT);
  pinMode(REV_ON, OUTPUT);
  digitalWrite(SPEED_1, LOW);
  digitalWrite(SPEED_2, LOW);
  digitalWrite(SPEED_3, LOW);
  digitalWrite(SPEED_4, LOW);
  digitalWrite(SPEED_5, LOW);
  digitalWrite(REV_ON, LOW);


  Serial.begin(9600);

  // setup i2c comm
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  delay(100);

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    SerialUSB.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }

  SerialUSB.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();



  //setup outputs
  pinMode(SPEED_1, OUTPUT);
  pinMode(SPEED_2, OUTPUT);
  pinMode(SPEED_3, OUTPUT);
  pinMode(SPEED_4, OUTPUT);
  pinMode(SPEED_5, OUTPUT);
  pinMode(REV_ON, OUTPUT);

  pinMode(HV, INPUT_PULLUP);
  pinMode(HW, INPUT_PULLUP);
  pinMode(HU, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HW), updateEncoder, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(HV), updateEncoder, CHANGE);  
  // attachInterrupt(digitalPinToInterrupt(HU), updateEncoder, CHANGE); // is not connected to an interupptpin at the moment, swap to enable
  previousMillis = millis();
}

void readNprintGyro() {
  lsm.read();  /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);

  accelerationX.number = a.acceleration.x;
  accelerationY.number = a.acceleration.y;
  accelerationZ.number = a.acceleration.z;

  magneticX = m.magnetic.x;
  magneticY = m.magnetic.y;
  magneticZ = m.magnetic.z;

  gyroX.number = g.gyro.x;
  gyroY.number = g.gyro.y;
  gyroZ.number = g.gyro.z;


/*
  SerialUSB.print("Accel X: "); SerialUSB.print(a.acceleration.x); SerialUSB.print(" m/s^2");
  SerialUSB.print("\tY: "); SerialUSB.print(a.acceleration.y);     SerialUSB.print(" m/s^2 ");
  SerialUSB.print("\tZ: "); SerialUSB.print(a.acceleration.z);     SerialUSB.println(" m/s^2 ");

  SerialUSB.print("Mag X: "); SerialUSB.print(m.magnetic.x);   SerialUSB.print(" gauss");
  SerialUSB.print("\tY: "); SerialUSB.print(m.magnetic.y);     SerialUSB.print(" gauss");
  SerialUSB.print("\tZ: "); SerialUSB.print(m.magnetic.z);     SerialUSB.println(" gauss");

  SerialUSB.print("Gyro X: "); SerialUSB.print(g.gyro.x);   SerialUSB.print(" dps");
  SerialUSB.print("\tY: "); SerialUSB.print(g.gyro.y);      SerialUSB.print(" dps");
  SerialUSB.print("\tZ: "); SerialUSB.print(g.gyro.z);      SerialUSB.println(" dps");

  SerialUSB.println();
*/
  //delay(20);
}//readNprintGryo

void readCurrent() {
  total = total - readings[indexI];
  readings[indexI] = analogRead(1); //Raw data reading
  currentValue2 = (readings[indexI] - 510) * 33 / 1024 / 0.4;
  readings[indexI] = (readings[indexI] - 510) * 3.3 / 1024 / 0.04 - 0.04; //Data processing:510-raw data from analogRead when the input is 0; 5-5v; the first 0.04-0.04V/A(sensitivity); the second 0.04-offset val;
  total = total + readings[indexI];
  indexI = indexI + 1;
  if (indexI >= numReadings)
    indexI = 0;
  average = total / numReadings; //Smoothing algorithm 
  currentValue = average; // Current is stored in this variable
  //SerialUSB.println(currentValue); // Print the current
}

void updateEncoder() {
  encoderValue++;
}
void readangle() {
  //angle.number = encoderValue * 30;
  angleMotor = encoderValue * 60;
}
void readNprintVolt() {
  float Ain = analogRead(AR_DEFAULT);


  // calculate the voltage

  v1 = 6.6 * (Ain * 3.3) / 1024.0;
  
}
void readOmega() {
  currentMillis = millis();
  readangle();
  if (currentMillis - previousMillis >= interval) {
    omega = (float)((angleMotor - prevAngleMotor) * 2 * 3.14 / 360) / (abs(currentMillis - previousMillis) * 0.001); //abs((currentMillis - previousMillis)));
    omegaAxle.number = omega / GEARBOX_RATIO;
    prevAngleMotor = angleMotor;
    previousMillis = currentMillis;
    //encoderValue = 0;

  }

}//readOmega

void readTorque() {
  if (abs(omega) >= 0.1) {
    // torqueUni.number= (currentValue * v1*GEARBOX_RATIO) / (omega);
    torqueTest = (currentValue * v1 * GEARBOX_RATIO) / (omega);
  } else {
    torqueTest = 0;
  }
  torqueUni.number = torqueTest;
}
void receiveData(int byteCount) {
  SerialUSB.println("GOT SOME");
  int i = 0;
  while (Wire.available()) {
    number[i] = Wire.read();
    SerialUSB.println(number[i]);

    i++;
  }
  number[i] = '\0';

  //commands the sequence
  if (number[0] == 'A') {
    number[0] = 'n'; // just to get rid of it
    stage = 0;
    angle.number = 0;
    angleMotor = 0;
    encoderValue = 0;
    pause = false;
  } else if (number[0] == 'B') {
    number[0] = 'n'; // just to get rid of it
    stage = 100;
    angle.number = 0;
    angleMotor = 0;
    encoderValue = 0;
    pause = false;

  } else if (number[0] == 'C') {
    number[0] = 'n'; // just to get rid of it
    stage = 150;
    angle.number = 0;
    angleMotor = 0;
    encoderValue = 0;
    pause = false;
  }
  if (number[0] == 'P') {         //pause
    number[0] = 'n'; // just to get rid of it
    digitalWrite(SPEED_1, LOW);
    digitalWrite(SPEED_2, LOW);
    digitalWrite(SPEED_3, LOW);
    digitalWrite(SPEED_4, LOW);
    digitalWrite(SPEED_5, LOW);
    digitalWrite(REV_ON, LOW);
    //stop
    pause = true;
  }
  if (number[0] == 'R') {         //reset
    number[0] = 'n'; // just to get rid of it

    pause = true;
    stage = -1;
    state = 'i';
    angle.number = 0;
    encoderValue = 0;
    digitalWrite(SPEED_1, LOW);
    digitalWrite(SPEED_2, LOW);
    digitalWrite(SPEED_3, LOW);
    digitalWrite(SPEED_4, LOW);
    digitalWrite(SPEED_5, LOW);
    digitalWrite(REV_ON, LOW);
  }
  // SerialUSB.print(number);
}  // end while


void buildDataToSend() {
  angle.number = encoderValue * 60;
  // Could do with for loops, but better overview this way
  dataToSend[0] = state;
  dataToSend[1] = stage;
  dataToSend[2] = angle.bytes[0];
  dataToSend[3] = angle.bytes[1];
  dataToSend[4] = omegaAxle.bytes[0];
  dataToSend[5] = omegaAxle.bytes[1];
  dataToSend[6] = accelerationX.bytes[0];
  dataToSend[7] = accelerationX.bytes[1];
  dataToSend[8] = accelerationX.bytes[2];
  dataToSend[9] = accelerationX.bytes[3];
  dataToSend[10] = accelerationY.bytes[0];
  dataToSend[11] = accelerationY.bytes[1];
  dataToSend[12] = accelerationY.bytes[2];
  dataToSend[13] = accelerationY.bytes[3];
  dataToSend[14] = accelerationZ.bytes[0];
  dataToSend[15] = accelerationZ.bytes[1];
  dataToSend[16] = accelerationZ.bytes[2];
  dataToSend[17] = accelerationZ.bytes[3];
  dataToSend[18] = gyroX.bytes[0];
  dataToSend[19] = gyroX.bytes[1];
  dataToSend[20] = gyroX.bytes[2];
  dataToSend[21] = gyroX.bytes[3];
  dataToSend[22] = gyroY.bytes[0];
  dataToSend[23] = gyroY.bytes[1];
  dataToSend[24] = gyroY.bytes[2];
  dataToSend[25] = gyroY.bytes[3];
  dataToSend[26] = gyroZ.bytes[0];
  dataToSend[27] = gyroZ.bytes[1];
  dataToSend[28] = gyroZ.bytes[2];
  dataToSend[29] = gyroZ.bytes[3];
  dataToSend[30] = torqueUni.bytes[0];
  dataToSend[31] = torqueUni.bytes[1];
  dataToSend[32] = torqueUni.bytes[2];
  dataToSend[33] = torqueUni.bytes[3];



}
void sendData() {
  if (indexR >= dataToSendLength) {
    indexR = 0;
  }
  Wire.write(dataToSend[indexR]);
  ++indexR;



}

void sequenceA() {
  currentSequenceMillis = millis();
  if (state == 'e' && stage == 4) {
    SerialUSB.println(torqueTest);
    state=='f';
    delay(2000);
  }
  if (state == 'e' && stage == 3) {
    //digitalWrite(SPEED_1, LOW);
    if (torqueTest <= 1) {
      stage = 4;
    }
    //SerialUSB.println("STAGE 1");
    prevSequenceMillis = millis();
    SerialUSB.print("FINAL2 ");//SerialUSB.print(angleMotor/GEARBOX_RATIO);
    SerialUSB.println(torqueTest);


  }

  if (state == 'e' && stage == 2 && (torqueTest >= 1.2)) {
    digitalWrite(SPEED_1, LOW);
    //digitalWrite(SPEED_5,HIGH);
    stage = 3;
    SerialUSB.print("FINAL"); SerialUSB.println(torqueTest);
    prevSequenceMillis = millis();
    angleMotor = 0;
    encoderValue = 0;

  }
  if (state == 'e' && stage == 1 && ((angleMotor / GEARBOX_RATIO ) >= 360 * 20)) {
    //digitalWrite(SPEED_1, LOW);
    //digitalWrite(SPEED_2,HIGH);
    stage = 2;
    //SerialUSB.println("STAGE 1");
    prevSequenceMillis = millis();

  }
  if (state == 'i' && stage == 0) {
    state = 'e';
    //start low speed
    angle.number = 0;
    angleMotor = 0;
    torqueTest = 0;
    encoderValue = 0;
    digitalWrite(SPEED_1, HIGH);
    stage = 1;
    prevSequenceMillis = millis();
    //SerialUSB.println("STAGE0");
  }
  //delay(10); //some value between 3 and 15 to take account for the rise time of the relays



}
void sequenceC() {
  currentSequenceMillis = millis();
  if (state == 'e' && stage == 154) {
    SerialUSB.println(torqueTest);
    delay(20000);
  }
  if (state == 'e' && stage == 153) {
    //digitalWrite(SPEED_1, LOW);
    if (torqueTest <= 1) {
      stage = 154;
    }
    //SerialUSB.println("STAGE 1");
    prevSequenceMillis = millis();
    SerialUSB.print("FINAL2 ");//SerialUSB.print(angleMotor/GEARBOX_RATIO);
    SerialUSB.println(torqueTest);


  }

  if (state == 'e' && stage == 152 && (torqueTest >= 1.2)) {
    digitalWrite(SPEED_1, LOW);
    //digitalWrite(SPEED_5,HIGH);
    stage = 153;
    SerialUSB.print("FINAL"); SerialUSB.println(torqueTest);
    prevSequenceMillis = millis();
    angleMotor = 0;
    encoderValue = 0;

  }
  if (state == 'e' && stage == 151 && ((angleMotor / GEARBOX_RATIO ) >= 360 * 20)) {
    //digitalWrite(SPEED_1, LOW);
    //digitalWrite(SPEED_2,HIGH);
    // stage=152;
    digitalWrite(SPEED_1, LOW);
    //SerialUSB.println("STAGE 1");
    prevSequenceMillis = millis();
    SerialUSB.print("DONE");
    delay(10000);

  }
  if (state == 'i' && stage == 150) {
    state = 'e';
    //start low speed
    angle.number = 0;
    angleMotor = 0;
    torqueTest = 0;
    encoderValue = 0;
    digitalWrite(SPEED_1, HIGH);
    digitalWrite(REV_ON, HIGH);
    stage = 151;
    prevSequenceMillis = millis();
    //SerialUSB.println("STAGE0");
  }
  //delay(10); //some value between 3 and 15 to take account for the rise time of the relays



}
void sequenceB() {
  currentSequenceMillis = millis();
  readangle();

  if (state == 'e' && stage == 101 && (angleMotor / GEARBOX_RATIO ) >= 20) {
    digitalWrite(SPEED_1, LOW);
    //SerialUSB.println("STAGE 1");
    state = 'f';
    stage = 102;
  }

  if (state == 'i' && stage == 100) {
    state = 'e';
    //start low speed
    angle.number = 0;
    angleMotor = 0;
    encoderValue = 0;
    digitalWrite(SPEED_1, HIGH);
    stage = 101;
    prevSequenceMillis = millis();
    prevAngle = 0;
    //SerialUSB.println("STAGE0");
  }
}

void loop()
{


  sequenceA();
  sequenceB();
  sequenceC();
  readangle();
  readOmega();
  readNprintGyro();
  readNprintVolt();
  readCurrent();
  readTorque();
  buildDataToSend();
  SerialUSB.print(omega);
  //SerialUSB.print("STAGE : "); SerialUSB.print(stage); SerialUSB.print(" Encoder : "); SerialUSB.print(encoderValue);
  //SerialUSB.print(" CURRENT : ");
  /*SerialUSB.print(currentValue);
    //SerialUSB.print(" angleMotor : "); SerialUSB.print(angleMotor);
    //SerialUSB.print(" omega : ");
    SerialUSB.print(" ");
    SerialUSB.print(currentValue2);
    SerialUSB.print(" ");
    SerialUSB.print(omega);
    SerialUSB.print(" ");
    //SerialUSB.print("Voltage : ");
    SerialUSB.print(v1);
    SerialUSB.print(" ");
    //SerialUSB.print(" STAGE  : "); SerialUSB.print(stage);
    //SerialUSB.print("Torque : ");
    SerialUSB.println(torqueTest);// SerialUSB.print("  "); SerialUSB.print(omega); SerialUSB.print("   ");
    //SerialUSB.println(angleMotor/GEARBOX_RATIO);

  */
}
