#!/usr/bin/python3

#RPi Pinouts

#I2C Pins 
#GPIO2 -> SDA
#GPIO3 -> SCL

#Import the Library Requreid 
import smbus
import time
import struct

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(0)

# This is the address we setup in the Arduino Program
#Slave Address 1
address = 0x04

#Slave Address 2
address_2 = 0x05

def writeNumber(value):
    #bus.write_byte(address, value)
    bus.write_byte(address_2, value)
    # bus.write_byte_data(address, 0, value)
    return -1

def readNumber():
    # number = bus.read_byte(address)
    number = bus.read_byte_data(address_2,20)
    return number
    
#while True:
    #Receives the data from the User
    #data = raw_input("Enter the data to be sent : ")
  #  data_list = list(data)
  #  for i in data_list:
        #Sends to the Slaves 
   #     writeNumber(int(ord(i)))
   #     time.sleep(.1)

  #  writeNumber(int(0x0A))
    

while True:
    #[0]=state;
#[1]=stage;
#[2]=angle.bytes[0];
#[3]=angle.bytes[1];
#[4]=torqueUni.bytes[0];
#[5]=torqueUni.bytes[1];
#[6]=accelerationX.bytes[0];
#[7]=accelerationX.bytes[1];
#[8]=accelerationX.bytes[2];
#[9]=accelerationX.bytes[3];
#[10]=accelerationY.bytes[0];
#[11]=accelerationY.bytes[1];
#[12]=accelerationY.bytes[2];
#[13]=accelerationY.bytes[3];
#[14]=accelerationY.bytes[0];
#[15]=accelerationY.bytes[1];
#[16]=accelerationY.bytes[2];
#[17]=accelerationY.bytes[3];
#[18]=0;
#[19]=0;
#[20]=0;

#put the data from the I^2C bus into a List
    dataList=[]
    for i in range(0,20):
        dataList.append(readNumber())
        time.sleep(0.1)
    #Pick out the numbers            
    state=chr(dataList[0])
    stage=bytes(dataList[1])
    shorts=struct.unpack('<HH',bytearray(dataList[2:6]))
    angle=shorts[0]
    torque=shorts[1]

    floats=struct.unpack('<fff',bytearray(dataList[6:18]))
    accX=floats[0]
    accY=floats[1]
    accZ=floats[2]

    print(angle)
    print(state)
    print(stage)
    print(accX,accY,accZ)
    print("  ")
    time.sleep(1)
   
        
#End of the Script

