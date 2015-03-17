#!/usr/bin/python

########################################################################################
#
# |B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|
#
# A Python script for controlling the BFRMR1 mobile robot
#
#
# Author : Peter Neal
#
# Date : 30 July 2014
# Last Update : 14 March 2015
#
########################################################################################

import BFRMR1tft
import RPi.GPIO as GPIO
import time
import BFRMR1serialport
import os
import BFRMR1OpenCV
import numpy
import math
from colorama import init,Fore
init(autoreset=True)

GPIO.setup(4, GPIO.IN) #Buttons
GPIO.setup(17, GPIO.IN)
GPIO.setup(27, GPIO.IN)
GPIO.setup(22, GPIO.IN)

TurnRatio = 3.4
Run = True

MapWidth = 200
MapHeight = 200
MapScale = 2 #Map is divided by scale value. The higher the value, the smaller the map


# Data Packet from robot
#
# Byte  Description
# 0	Left Looking IR		
# 1	Centre IR
# 2	Right Looking IR
# 3	Head Pan Position
# 4	Head Tilt Position
# 5	Sonar Sensor
# 6	Left Encoder
# 7	Right Encoder
#          
########################################################################################
#
# Opencv threshold values
#
########################################################################################

YELLOWOBJECTS = [15,130,110,30,255,255]
BLUEOBJECTS = [40,120,120,160,255,255]
ORANGEOBJECTS = [5,170,170,15,255,255]

########################################################################################
#
# Robot control variables
#
########################################################################################
#Instructions
GETDATA = 0
ROBOTFORWARD = 1
ROBOTLEFT = 2
ROBOTRIGHT = 3
ROBOTREVERSE = 4
HEADMOVE = 5
PLAYTONE = 6

headpan = 128
headtilt = 128
LeftEncoderTotal = 0
RightEncoderTotal = 0
SonarThreshold = 20
IRThreshold = 60
AutoSpeed = 5  #speed of robot during automatic operation

########################################################################################
# HeadMove
#
# Takes three arguments of head pan and head tilt angle and speed. The angle is converted to a value
# between 0 and 255 to be sent to arduino. Min angle -80 degrees, Max 80 degrees. Speed value should
# be between 0 and 10.
# Returns a packet of sensor data recieved from robot when command is complete.
########################################################################################
def HeadMove(pan,tilt,speed):

    panservovalue = int(128-(pan*1.45)) #convert from angle to value between 0 and 255
    tiltservovalue = int(128+(tilt*1.45))
    if speed <= 10:
        BFRMR1serialport.sendserial([255, 255, HEADMOVE, panservovalue, tiltservovalue, speed, 0]) #send command to move head
        while True:
            a = BFRMR1serialport.getserial() #wait here until data is received to confirm command complete
            if a is not None:
                break    
        return a #return data packet
    else:
        print "Speed out of range. Value should be between 0 and 10"
    

########################################################################################
# RobotMove
#
# Used to move robot either forward, backwards or turn. Takes a direction argument:
# ROBOTFORWARD = 1
# ROBOTLEFT = 2
# ROBOTRIGHT = 3
# ROBOTREVERSE = 4
# Encodercount argument dictates distance to move/turn and speed is self explanatory.
# Speed values between 1 and 10 are sensible
# Returns a packet of sensor data recieved from robot when command is complete.
########################################################################################
def RobotMove(direction,encodercount,speed, SonarThreshold, IRThreshold):

    # 3.4 degrees = 1 encoder count for a turn
    BFRMR1serialport.sendserial([255, 255, direction, encodercount, speed, SonarThreshold, IRThreshold]) #send command to move head
    while True:
        a = BFRMR1serialport.getserial() #wait here until data is received to confirm command complete
        if a is not None:
            break 
       
    return a #return data packet

########################################################################################
# GetData
#
# Sends a command to the robot to trigger a packet of sensor data to be returned.
# Returns a packet of sensor data received from robot.
########################################################################################
def GetData():
    BFRMR1serialport.sendserial([255, 255, GETDATA, 0, 0, 0, 0]) #send command to get sensor data from robot
    while True:
        a = BFRMR1serialport.getserial() #wait here until data is received to confirm command complete
        if a is not None:
            break    
    return a #return data packet

########################################################################################
# PlayTone
#
# Sends a command to the robot to play a tone dictated by toneselection argument.
########################################################################################

def PlayTone(toneselection):
    BFRMR1serialport.sendserial([255, 255, PLAYTONE, toneselection, 0, 0, 0]) #send command to play a tone
    return

########################################################################################
#
# Test moves routine. Will execute a move depending on value of PointerCounterTest
#
########################################################################################

def Turn180():
    RobotData = RobotMove(ROBOTLEFT,int(180/TurnRatio),3, 0, 255) #no sonar or IR threshold so move always completes
    print RobotData

def Turn360():
    RobotData = RobotMove(ROBOTLEFT,int(360/TurnRatio),3, 0, 255) #no sonar or IR threshold so move always completes
    print RobotData

def MoveForward():
    RobotData = RobotMove(ROBOTFORWARD,50,AutoSpeed, 0, 255) #no sonar or IR threshold so move always completes
    print RobotData

def MoveReverse():
    RobotData = RobotMove(ROBOTREVERSE,50,AutoSpeed, 0, 255) #no sonar or IR threshold so move always completes
    print RobotData

########################################################################################
#
# Button 0 Interrupt Routine
#
########################################################################################

def Button0Pressed(channel):  
    print "Button 0 pressed"
    
########################################################################################
#
# Button 1 Interrupt Routine
#
########################################################################################

def Button1Pressed(channel):  
    print "Button 1 pressed"
    
########################################################################################
#
# Button 2 Interrupt Routine
#
########################################################################################

def Button2Pressed(channel):
    print "Button 2 pressed"
          
########################################################################################
#
# Button 3 Interrupt Routine
#
######################################################################################## 
           
def Button3Pressed(channel):
    print "Button 3 pressed"

########################################################################################
#
# Main
#
########################################################################################

GPIO.add_event_detect(4,  GPIO.FALLING, callback=Button0Pressed, bouncetime=200)
GPIO.add_event_detect(17, GPIO.FALLING, callback=Button1Pressed, bouncetime=200)
GPIO.add_event_detect(27, GPIO.FALLING, callback=Button2Pressed, bouncetime=200)
GPIO.add_event_detect(22, GPIO.FALLING, callback=Button3Pressed, bouncetime=200)

while True:

########################################################################################
#
# 
#
########################################################################################

    while Run is True:
        
        MapArray = BFRMR1OpenCV.NewMap(MapWidth,MapHeight)
        TargetFound = False

        #Scan area in front of robot looking for a QR code until code is found
        HeadTiltAngle = 0
        for y in range(-40,41,20):
            HeadPanAngle = y
            RobotData = HeadMove(HeadPanAngle,HeadTiltAngle, 8)
            time.sleep(0.2) #small delay, let image settle
            print "Scanning image"
            Data = BFRMR1OpenCV.FindQRBorder(YELLOWOBJECTS)
            '''
            QRCodeData = BFRMR1OpenCV.ReadQRCode()
            if QRCodeData is 0:
                print "No QR code found"
            else:
                QRValue = QRCodeData[0]
                QRX = QRCodeData[1]
                QRY = QRCodeData[2]
                QRDistance = QRCodeData[3]
          
                #Calculate X and Y values for map here
                #Find X and Y coordinates of QR code in camera centred world coorinates in CM
                XDist = QRX - 320.00
                XCamAngle = math.atan(XDist/640.00)
                XCamAngleDeg = math.degrees(XCamAngle)
                TotalAngle = XCamAngleDeg + HeadPanAngle

                #Turn to face QR Code       
                if TotalAngle > 0:
                    RobotData = RobotMove(ROBOTRIGHT,int(abs(TotalAngle/TurnRatio)),3, 0, 255) 
                elif TotalAngle < 0:
                    RobotData = RobotMove(ROBOTLEFT,int(abs(TotalAngle/TurnRatio)),3, 0, 255)
                TargetFound = True
                break

        if TargetFound is True:
            print "Centring Head"
            RobotData = HeadMove(0, 0, 8)
            time.sleep(1)
            QRCodeData = BFRMR1OpenCV.ReadQRCode()
            if QRCodeData is 0:
                print "No QR code found"
                RobotData = RobotMove(ROBOTFORWARD,20,3, 0, 255)
            else:
                QRValue = QRCodeData[0]
                QRX = QRCodeData[1]
                QRY = QRCodeData[2]
                QRDistance = QRCodeData[3]

            RobotData = GetData()
            LeftIR = RobotData[0] 
            CentreIR = RobotData[1]
            RightIR = RobotData[2]
            IRString = "LeftIR = " + str(LeftIR) + " CentreIR = " + str(CentreIR) + " RightIR = " + str(RightIR)
            print(Fore.GREEN + IRString)

            if LeftIR > IRThreshold or CentreIR > IRThreshold or RightIR > IRThreshold:
                RobotData = RobotMove(ROBOTREVERSE,20,3, 0, 255)

        '''           
                
        '''
        XCam = math.sin(XCamAngle) * QRDistance
        YCam = math.cos(XCamAngle) * QRDistance
        HeadPanRad = math.radians(HeadPanAngle)
        XWorld = XCam*math.cos(-HeadPanRad) - YCam*math.sin(-HeadPanRad)
        YWorld = XCam*math.sin(-HeadPanRad) + YCam*math.cos(-HeadPanRad)

        #Scale and translate X and Y coordinates ready to plot to map
        XWorld = int((XWorld/MapScale) + (MapWidth/2))
        YWorld = int(MapHeight - (YWorld/MapScale))
                
        print "XWorld =",XWorld, "YWorld =", YWorld

        MapArray = BFRMR1OpenCV.AddToMap(MapArray,XWorld,YWorld,QRCodeData[0])
        BFRMR1OpenCV.ShowMap(MapArray)
        '''

BFRMR1serialport.closeserial()  





