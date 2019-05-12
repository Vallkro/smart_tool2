#include <Wire.h>
#define SLAVE_ADDRESS 0x2A

void setup() {
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);
    Wire.onRequest(sendData); 
}

void loop() {
}

char data[] = "66.1 33.4 11.2 44.5";
int index = 0;

// callback for sending data
void sendData() { 
    Wire.write(data[index]);
    ++index;
    if (index >= sizeof(data)) {
         index = 0;
    }
 }
