from time import sleep

import serial

ser =serial.Serial("/dev/ttyUSB0",115200)

text=[]
file1=open("datos.txt","w+")

while True:

    sleep(1)

    getVal = ser.readline()

    
    file1.write(str(getVal))