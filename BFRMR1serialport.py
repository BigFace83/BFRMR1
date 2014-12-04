
# |B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|

import serial
import time

print ("Opening Serial Port")
ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 10)
time.sleep(1) # wait here a bit, let arduino boot up fully before sending data
print ("Port open:  " + ser.portstr)       # check which port was really used


def getserial():
    #print "Serial in waiting ", ser.inWaiting()
    if ser.inWaiting()>=10:
        preamble1 = ord(ser.read())
        if preamble1 == 255:
            if ord(ser.read()) == 255:
 
                x = ser.read(8) #read in 8 bytes. 3 IR sensors, 2 servo feedbacks, 1 sonar value and 2 encoder readings
                a = [ord(x) for x in x]

                if ser.inWaiting()>0:            #if any data is left in input buffer following the read of a packet
                    ser.read(ser.inWaiting())    #read what is left to clear data from buffer.
                                                 #This ensures next data packet read is the most up to date data
                return a

    
 

def sendserial(serialdata):
    for x in range(len(serialdata)):
        ser.write(chr(serialdata[x]))
    #print serialdata


def closeserial():
    ser.close() 

