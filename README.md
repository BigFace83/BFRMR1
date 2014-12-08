|B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|

Author : Peter Neal

BFRMR1
======

Software for BFRMR1 mobile robot.

Contains arduino code and python scripts to run on the onboard Raspberry pi.


BFRMR1_arduino.ino

Arduino code. Will wait until a packet of data is received and act depending on
the instruction sent. Functions include control loops for the two head servos and
the two continuous rotation servos used for the drive wheels. Encoders on the 
wheels provide feedback for the control loop. Will also read all of the sensors
and return a packet of data when a command has completed or a data request is 
received.


BFRMR1Main.py

Main script for control of the robot. Writes to the tft screen, uses interrupts
to take input from the push buttons and executes a particular behaviour based on
user selection.


BFRMR1serialport.py

Opens a serial connection to the Arduino and contains a function to read
a packet of data from the Arduino and return as an array, and a function to send
data.
Uses pyserial


BFRMR1tft.py

Driver script for the Adafruit 2.2" tft display. Writes data to the screen
using the spi interface.
Includes functions to write text to the screen. Text lookup table contained
in font5x7.py and font8x12.py.


BFRMR1OpenCV.py

Contains functions related to vision using OpenCV.
Requires OpenCV on the Raspberry pi. 


