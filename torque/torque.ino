/* This script is a part of the capstone-/bachelor thesis work:
    Smart oil filter tool for use in manufacturing

    The purpose of this script is to calculate torque applied by tool in a tightening 
    sequence. This is done with the following formula:

        I * V * E * 60
    T = --------------
        RPM * 2 * Pi
    
    Where:
    I = current
    V = voltage
    E = efficiency

    Current and voltage are measured using dedicated sensors. RPM is measured using the built in 
    hall sensor in the BLDC.

    Team Mother Truckers
    Chalmers University of Technology
    Pennsylvania State University
*/

// Declare the function


//#include <ros.h>
#include "Arduino.h"

// GLOBAL VARIABLES
/*
#define voltageSensor 1
#define currentSensor 0
#define hallSensor 2 

#define speedHigh 1
#define speedLow 2
*/

int i = 1;

float efficiency = 0.95; // Just an estimate
float gearbox_ratio = 10;
float torque = 0.0;
const double Pi = 3.141592653589793;

float setTorque = 0.0; //<-- Stop at this torque limit
float actualTorque = 0.0;

// VOLTAGE SENSOR 
float Vin = 0.0;
int Vintest = 20;
float Vout = 0.0;
float R1 = 10.0; // resistance of R1 (10M)
float R2 = 1.0; // resistance of R2 (1M) 
int value = 0;

// CURRENT SENSOR
const int numReadings = 50;
float readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average

float currentValue = 0;

// HALL SENSOR (RPM)
 int refsig = 200; //for converting the analog signal coming from hall sensor to digital through arduino code
 int val;//the digital value of the incoming analog signals
 int prev_val = 0;
 int t, cur_t; //time variables
 float rpm = 0.0; 

void setup()
{
  Serial.begin(9600); // To be removed
  pinMode(7, OUTPUT);
  digitalWrite(7,HIGH);
  
  pinMode(1, INPUT); //VOLT
  pinMode(0, INPUT); //CURRENT
  pinMode(2, INPUT); //HALL
  
  //pinMode(speedLow, OUTPUT);
  
  // Current sensor:
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;  
  
}

float updateTorque()
{
  // Voltage:
  value = analogRead(1);
  Vout = (value * 5.0) / 1024.0; 
  Vin = Vout / (R2/(R1+R2)); 
  if (Vin<0.09) {
   Vin=0.0; // Remove undesired readings
  }

  // Current:
  total= total - readings[index];          
    readings[index] = analogRead(0); //Raw data reading
    readings[index] = (readings[index]-510)*5/1024/0.04-0.04;//Data processing:510-raw data from analogRead when the input is 0; 5-5v; the first 0.04-0.04V/A(sensitivity); the second 0.04-offset val;
    total= total + readings[index];       
    index = index + 1;                    
    if (index >= numReadings)              
      index = 0;                           
    average = total/numReadings;   //Smoothing algorithm (http://www.arduino.cc/en/Tutorial/Smoothing)    
    currentValue= average; // Current is stored in this variable 
    //Serial.println(currentValue); // Print the current  
    
    // Hall sensor (RPM)
   int sig = digitalRead(2); //read raw value of hall sensor
   if (sig > refsig) val = HIGH; //convert it to digital 0,1 form
   else val = LOW;
   if (prev_val == 0 && val == 1) { //check for rising edge
     cur_t = micros();
     rpm = 1000000 * 60 / (cur_t - t);
     //Serial.println(rpm); //print the rpm
     t = micros();
   }
   prev_val = val;
  

  // Torque calculation:
  torque = (currentValue * Vintest * efficiency * 60) / (rpm * 2 * Pi);
  torque = torque * gearbox_ratio;

  // Return the torque value

  return currentValue;
}


void loop()
{
  actualTorque = updateTorque();
  

  // Screw-on phase
  /*
  digitalWrite(speedLow,HIGH);
  delay(1000);
  digitalWrite(speedLow,LOW);
  */
  Serial.println(currentValue);
  // Tightening phase
  if (actualTorque <= setTorque) {
    digitalWrite(7,HIGH); 
    
  }
}
