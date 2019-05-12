/*
I2C Pinouts

SDA -> A4
SCL -> A5
*/

//Import the library required 
#include <Wire.h>

//Slave Address for the Communication
#define SLAVE_ADDRESS 0x05

char number[50];
int state = 0;
String string1=String(100);
String string2=String(556);
String allString=string1+ " "+ string2;
const int len=allString.length()+1;
char data[10];


int index = 0;

typedef union
{
 float number;
 uint8_t bytes[4];
} FLOATUNION_t;
typedef union
{
  short number;
  uint8_t bytes[4];
} SHORTUNION_t;
SHORTUNION_t myShort;
SHORTUNION_t myShort2;


FLOATUNION_t myFloat;
//myFloat.number = 123.456; // Assign a number to the float

union cvt {
float val;
unsigned char b[4];
} x;


//Code Initialization
void setup() {
  // initialize i2c as slave
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);
 // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);


  x.val= 4828.9038;
  myFloat.number=4828.9038;
  myShort.number=699;
  myShort2.number=713;
    int iSend=0;
    /*
    for(int i =0;i<2;i++){
    data[i]=myShort.bytes[i]; 
    iSend++; 
    //Serial.print(myShort.bytes[i]); Serial.print(" "); Serial.print(i); Serial.println("");
    }
    for(int i=iSend;i<iSend+2;i++){
      data[i]=myShort2.bytes[i];
      iSend++ ;   
    }
    iSend=0;*/
    data[0]=myShort.bytes[0];
    data[1]=myShort.bytes[1];
    data[2]=myShort2.bytes[0];
    data[3]=myShort2.bytes[1];
}


void loop() {
  
  delay(10);
} // end loop

// callback for received data
void receiveData(int byteCount){
  int i = 0;
  while(Wire.available()) { 
    number[i] = Wire.read();
    i++;
  }
  number[i] = '\0';
    
   
  Serial.print(number);
}  // end while

// callback for sending data
void sendData() { 
  
    Serial.println("");
    Wire.write(data[index]);
    ++index;
    if (index >= 4) {
         index = 0;
    }
    delay(10);
 }

//End of the program 
