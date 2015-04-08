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

TurnRatio = 4
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

YELLOWOBJECTS = [10,100,90,30,255,255]
BLUEOBJECTS = [100,145,130,130,255,255]
ORANGEOBJECTS = [5,170,170,15,255,255]
GREENOBJECTS = [40,60,60,95,200,200]

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
# LookAtTarget - Turns head to centre target on the camera. X and Y arguements are camera
# coordinates of target
# Returns new head servo angles
#
########################################################################################
def LookAtTarget(X, Y, HeadPanAngle, HeadTiltAngle):

    print "Looking at Target"
    HeadAngles = []
    PlayTone(0)
    XDist = X - 320.00
    XCamAngle = math.atan(XDist/640.00)
    YDist = 240.00 - Y
    YCamAngle = math.atan(YDist/640.00)
    HeadTiltAngle = HeadTiltAngle + math.degrees(YCamAngle)
    HeadPanAngle = HeadPanAngle + math.degrees(XCamAngle)
    RobotData = HeadMove(HeadPanAngle,HeadTiltAngle, 5)
 
    HeadAngles.append(HeadPanAngle)
    HeadAngles.append(HeadTiltAngle)
    return HeadAngles
    
########################################################################################
#
# TurnToTarget - Turns robot to face object. Arguement passed to function is head pan angle
#
########################################################################################
def TurnToTarget(TurnAngle, Speed):
       
    print "Turning to face Target"
    if TurnAngle > 0:
        RobotData = RobotMove(ROBOTRIGHT,int(abs(TurnAngle/TurnRatio)),Speed, 0, 255) 
    elif TurnAngle < 0:
        RobotData = RobotMove(ROBOTLEFT,int(abs(TurnAngle/TurnRatio)),Speed, 0, 255)
    

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
# Scan for target - Scan area in front of robot looking for a target. If target is found
# turn to face it and check that it is still there and directly in front of robot.
# If target is found, return type, distance and angle. 
# TargetData = [boxcentrex, boxcentrey, Distance, SymbolFound, EdgeDifference] 
#
########################################################################################
def ScanForTarget():

    print "Scanning for target"
    returndata = -1
    #Scan area in front of robot looking for an image target
    HeadTiltAngle = 0
    for y in range(-40,41,10):
        HeadTiltAngle = 0
        HeadPanAngle = y
        RobotData = HeadMove(HeadPanAngle,HeadTiltAngle, 8)
        print "Scanning image"
        TargetData = BFRMR1OpenCV.FindSymbol(GREENOBJECTS)
        if TargetData == 0:
            print "No Target in image"
        else:
            HeadAngles = LookAtTarget(TargetData[0], TargetData[1], HeadPanAngle, HeadTiltAngle) #turn head to face target
            HeadPanAngle = HeadAngles[0] # get new head angles
            HeadTiltAngle = HeadAngles[1]
            for x in range (0,3): #check 3 times to see if target is still in front of robot
                TargetData = BFRMR1OpenCV.FindSymbol(GREENOBJECTS)           
                if TargetData != 0: #if target is still present
                    TurnToTarget(HeadPanAngle, 3) #Turn to face target
                    HeadPanAngle = 0
                    RobotData = HeadMove(HeadPanAngle,HeadTiltAngle, 8) #centre head pan angle

                    #Robot is now facing target, check again that it is still there and dead ahead.
                    #If not, correct heading and check again, up to 3 times
                    for x in range (0,3):
                        TargetData = BFRMR1OpenCV.FindSymbol(GREENOBJECTS)           
                        if TargetData != 0: #target found
                            HeadAngles = LookAtTarget(TargetData[0], TargetData[1], HeadPanAngle, HeadTiltAngle)
                            HeadPanAngle = HeadAngles[0]
                            HeadTiltAngle = HeadAngles[1]
                            if abs(HeadPanAngle) < 4: #When robot is looking at target, if head angle is less than 4 degrees either way
                                                      #then target is dead ahead
                                print "Target dead ahead"
                                print "Distance from OpenCV =", TargetData[2]
                                print "Sonar Distance =", RobotData[5]
                                returndata = [TargetData[3], TargetData[2], TargetData[4]]
                                return(returndata)
                            else: 
                                print "Target NOT ahead" 
                                TurnToTarget(HeadPanAngle, 3) #Turn to face target
               
                else:
                    Target = 0
                    print "Target Lost"
                    

            
                 

    return(returndata)



########################################################################################
#
# Main
#
########################################################################################

GPIO.add_event_detect(4,  GPIO.FALLING, callback=Button0Pressed, bouncetime=200)
GPIO.add_event_detect(17, GPIO.FALLING, callback=Button1Pressed, bouncetime=200)
GPIO.add_event_detect(27, GPIO.FALLING, callback=Button2Pressed, bouncetime=200)
GPIO.add_event_detect(22, GPIO.FALLING, callback=Button3Pressed, bouncetime=200)

TargetData = ScanForTarget()
if TargetData == -1:
    print "No Target found from scan"
else:
    print "Target Aquired"
        
        

BFRMR1serialport.closeserial()  





