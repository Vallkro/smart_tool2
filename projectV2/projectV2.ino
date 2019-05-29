#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
// i2c
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 5
#define LSM9DS1_MCS 6
#define SPEED_1 7
#define dataToSendLength 20

//i2c
#define SLAVE_ADDRESS 0x05
//rpm
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
SHORTUNION_t torqueUni;





// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

//setting up global vars;
char number[50];
char state = 'i';
bool done = false;
byte stage = -1;
short indexR = 0;
short counter = 0;


#define FLOATS_SENT 1

float temperature = 10.5;
float luminosity = 5.2;
float data[FLOATS_SENT];
char dataToSend[dataToSendLength];
//IMU
FLOATUNION_t  accelerationX;
FLOATUNION_t  accelerationY;
FLOATUNION_t  accelerationZ;

float  magneticX;
float  magneticY;
float  magneticZ;

float  gyroX;
float  gyroY;
float  gyroZ;

float  v1;
// CURRENT SENSOR
const int numReadings = 50;
float readings[numReadings];      // the readings from the analog input
int indexI = 0;                  // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average

float currentValue = 0;


//for rpm and angle.number
volatile long encoderValue = 0;
int interval = 1000;            // One-second interval for measurements
long previousMillis = 0;          // Counters for milliseconds during interval
long currentMillis = 0;
int rpm = 0;              // Variable for RPM measuerment
volatile SHORTUNION_t angle;



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
  data[0] = temperature;
  data[1] = luminosity;
  Serial.begin(9600);
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("LSM9DS1 data read demo");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
  // setup i2c comm
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  //setup outputs

  pinMode(7, OUTPUT);
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), updateEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(4), updateEncoder, CHANGE);
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

  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;



  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");
  /*
      Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
      Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
      Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

      Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
      Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
      Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

      Serial.println();
  */
  //delay(20);
}//readNprintGryo

void readCurrent() {
  total = total - readings[indexI];
  readings[indexI] = analogRead(0); //Raw data reading
  readings[indexI] = (readings[indexI] - 510) * 5 / 1024 / 0.04 - 0.04; //Data processing:510-raw data from analogRead when the input is 0; 5-5v; the first 0.04-0.04V/A(sensitivity); the second 0.04-offset val;
  total = total + readings[indexI];
  indexI = indexI + 1;
  if (indexI >= numReadings)
    indexI = 0;
  average = total / numReadings; //Smoothing algorithm (http://www.arduino.cc/en/Tutorial/Smoothing)
  currentValue = average; // Current is stored in this variable
  //Serial.println(currentValue); // Print the current
}

void updateEncoder() {
  encoderValue++;
}
void readangle() {
  angle.number = encoderValue * 60;
}
void readNprintVolt() {
  float Ain = analogRead(A2);


  // calculate the voltage
  // use 5.0 for a 5.0V ADC reference voltage
  // 5.015V is the calibrated reference voltage
  v1 = (Ain * 5.015) / 1024.0;
  // send voltage for display on Serial Monitor
  // voltage multiplied by 11 when using voltage divider that
  // divides by 11. 11.132 is the calibrated voltage divide
  // value
  // Serial.print(v1 * 11.132);
  // Serial.println (" V");
}
void readRPM() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    rpm = (float)(encoderValue * 30 / (currentMillis - previousMillis));
    encoderValue = 0;
  }
}//readRPM
void receiveData(int byteCount) {
  int i = 0;
  while (Wire.available()) {
    number[i] = Wire.read();
    i++;
  }
  number[i] = '\0';

  //commands the sequence
  if (number[0] == 'A') {
    number[0] = 'n'; // just to get rid of it
    stage = 0;
    pause = false;
  } else if (number[0] == 'B') {
    number[0] = 'n'; // just to get rid of it

    pause = false;

  }
  if (number[0] == 'P') {         //pause
    number[0] = 'n'; // just to get rid of it

    //stop
    pause = true;
  }
  if (number[0] == 'R') {         //reset
    number[0] = 'n'; // just to get rid of it

    pause = true;
    stage = -1;
    state = 'i';
    angle.number = 0;
  }
  Serial.print(number);
}  // end while


void buildDataToSend() {
  dataToSend[0] = state;
  dataToSend[1] = stage;
  dataToSend[2] = angle.bytes[0];
  dataToSend[3] = angle.bytes[1];
  dataToSend[4] = torqueUni.bytes[0];
  dataToSend[5] = torqueUni.bytes[1];
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
  dataToSend[18] = 0;
  dataToSend[19] = 0;
  dataToSend[20] = 0;

}
void sendData() {
  Wire.write(dataToSend[indexR]);
  ++indexR;
  if (indexR >= dataToSendLength) {
    indexR = 0;
  }
  delay(10);
}

void sequenceA() {
  currentSequenceMillis = millis();


  if (state == 'i' && stage == 0) {
    state = 'e';
    //start low speed
    angle.number = 0;
    digitalWrite(SPEED_1, HIGH);
    stage = 1;
    prevSequenceMillis = millis();
    //Serial.println("STAGE0");
  }
  //delay(10); //some value between 3 and 15 to take account for the rise time of the relays

  if (state == 'e' && stage == 1 && (angle.number >= 3000 || (currentSequenceMillis - prevSequenceMillis) >= 1000)) {
    digitalWrite(SPEED_1, LOW);
    //Serial.println("STAGE 1");
  }

}

void sequenceB() {
  currentSequenceMillis = millis();
  readangle();



  if (state == 'e' && stage == 1 && (angle.number >= 3000 || (currentSequenceMillis - prevSequenceMillis) >= 1000)) {
    digitalWrite(SPEED_1, LOW);
    //Serial.println("STAGE 1");
  }

  if (state == 'i' && stage == 0) {
    state = 'e';
    //start low speed
    angle.number = 0;
    digitalWrite(SPEED_1, HIGH);
    stage = 1;
    prevSequenceMillis = millis();
    //Serial.println("STAGE0");
  }
}

void loop()
{
  angle.number = 20;
  buildDataToSend();
  sequenceA();
  readangle();
  readRPM();
  readNprintGyro();
  readNprintVolt();
  readCurrent();
  Serial.print("STAGE : "); Serial.print(stage); Serial.print(" Encoder : "); Serial.print(encoderValue);
  Serial.print(" CURRENT : "); Serial.print(currentValue);
  Serial.print(" angle.number : "); Serial.print(angle.number); Serial.print(" RPM : "); Serial.println(rpm);

}
