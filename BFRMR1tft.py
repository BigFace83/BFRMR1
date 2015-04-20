########################################################################
#
# |B|I|G| |F|A|C|E| |R|O|B|O|T|I|C|S|
#
# A Python library for controlling the Adafruit 2.2" TFT LCD module
# from a Raspbery Pi.
#
# Author : Peter Neal
# Date: 25 July 2014
# Last Edit: 28 Nov 2014
########################################################################
import RPi.GPIO as GPIO
import time
import spidev   #hardware SPI
import font5x7  #Have these files in the same directory as this file
import font8x12

#TFT to RPi connections
# PIN TFT RPi
# 1 backlight 3V3
# 2 MISO <none>
# 3 CLK SCLK (GPIO 11)
# 4 MOSI MOSI (GPIO 10)
# 5 CS-TFT GND
# 6 CS-CARD <none>
# 7 D/C GPIO 25
# 8 RESET GPIO 24
# 9 VCC 3V3
# 10 GND GND
RST = 24
DC = 25

#Colour space is BGR565
#Color constants
BLACK = 0x0000
RED = 0x001F
BLUE = 0xF800
GREEN = 0x0400
YELLOW = 0x0FFF
WHITE = 0xFFFF
PURPLE = 0x8001
LIGHTBLUE = 0xFB26
ORANGE = 0x1C7F

#ILI9340 commands
SWRESET = 0x01 #software reset
SLPOUT = 0x11 #sleep out
DISPON = 0x29 #display on
CASET = 0x2A #column address set
RASET = 0x2B #row address set
RAMWR = 0x2C #RAM write
MADCTL = 0x36 #axis control
COLMOD = 0x3A #color mode

bColor = BLACK

########################################################################
#
#Low-level routines
#These routines access GPIO directly

def SetPin(pinNumber,value):
    #sets the GPIO pin to desired value (1=on,0=off)
    GPIO.output(pinNumber,value)

def InitIO():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(DC,GPIO.OUT)
    GPIO.setup(RST,GPIO.OUT)


########################################################################
#
#SPI routines:

def WriteByte(value, data=True):
    SetPin(DC,data)
    spi.writebytes([value])

def WriteCmd(value):
    "Send command byte to display"
    WriteByte(value,False)
    #set D/C line to 0 = command

def WriteWord (value):
    "sends a 16-bit word to the display as data"
    WriteByte(value >> 8)
    #write upper 8 bits
    WriteByte(value & 0xFF)
    #write lower 8 bits

def WriteList (byteList):
    "Send list of bytes to display, as data"
    for byte in byteList:
        #grab each byte in list
        WriteByte(byte)
        #and send it

def WriteBulk (value,reps,count):
    "sends a 16-bit pixel word many, many times using hardware SPI"
    "number of writes = reps * count. Value of reps must be <= 2048"
    SetPin(DC,0)
    #command follows
    spi.writebytes([RAMWR])
    #issue RAM write command
    SetPin(DC,1)
    #data follows
    valHi = value >> 8
    #separate color into two bytes
    valLo = value & 0xFF
    byteArray = [valHi,valLo]*reps #create buffer of multiple pixels
    for a in range(count):
        spi.writebytes(byteArray)
        #send this buffer multiple times

def Command(cmd, *bytes):
    "Sends a command followed by any data it requires"
    SetPin(DC,0)
    #command follows
    spi.writebytes([cmd])
    #send the command byte
    if len(bytes)>0:
        #is there data to follow command?
        SetPin(DC,1)
        #data follows
        spi.writebytes(list(bytes)) #send the data bytes

########################################################################
#
#ILI9340 driver routines:


def InitDisplay():
    "Resets & prepares display for active use."
    SetPin(RST,0) #Hardware reset if required
    time.sleep(0.2)
    SetPin(RST,1)
    time.sleep(0.2)
    #Don't think the software reset is required if hardware reset used
    #WriteCmd (SWRESET) #software reset, puts display into sleep
    #time.sleep(0.2) #wait 200mS for controller register init
    WriteCmd (SLPOUT) #sleep out
    Command (COLMOD, 0x05) #set color mode to 16 bit
    time.sleep(0.2) #wait 200mS for TFT driver circuits
    WriteCmd (DISPON) #display on!


def SetAddrWindow(x0,y0,x1,y1):
    "sets a rectangular display window into which pixel data is placed"
    WriteCmd(CASET)
    #set column range (x0,x1)
    WriteWord(x0)
    WriteWord(x1)
    WriteCmd(RASET)
    #set row range (y0,y1)
    WriteWord(y0)
    WriteWord(y1)

def SetOrientation(degrees):
    "Set the display orientation to 0,90,180,or 270 degrees"
    if degrees==90: arg=0x60    
    elif degrees==180: arg=0xC0
    elif degrees==270: arg=0xA0
    else: arg=0x00
    Command (MADCTL,arg)

##########################################################################################
#
# Graphics routines

def FillRect(x0,y0,x1,y1,color):
    "fills rectangle with given color"
    y0passed = y0        #to allow for rotation
    y1passed = y1        #flip y0 and y1
    y0 = 240 - y1passed 
    y1 = 240 - y0passed
    width = x1-x0+1
    height = y1-y0+1
    SetAddrWindow(x0,y0,x1,y1)
    WriteBulk(color,width,height)

def FillScreen(color):
    "Fills entire screen with given color"
    FillRect(0,0,320,240,color)

def ClearScreen():
    "Fills entire screen with black"
    FillRect(0,0,320,240,bColor)

##########################################################################################
#
# Text routines

def PutCh5x7 (ch,xPos,yPos,color): #Put an individual character to the screen using 5x7 font
    yPos = 240-yPos #compensate for rotating display. Puts 0,0 in top left hand corner
    charData = font5x7.data[ord(ch)-32]    #get the character data
    SetAddrWindow(xPos,yPos-6,xPos+4,yPos)  #set display window to 8x12 rect
    SetPin(DC,0)
    spi.writebytes([RAMWR])  #pixel data follows
    SetPin(DC,1)
    buf = []                 #setup buffer for pixel data
    mask = 0x40              #start with msb in each column 
    for row in range(7):     #iterate over all 96 pixels 
        for col in range(5):   #do all columns for each row
            bit = charData[col] & mask  #get the next bit
            if (bit==0):         #is it a 0 or 1?
                pixel = bColor   #0: use background color
            else:
                pixel = color    #1: use foreground color
            buf.append(pixel>>8)
            buf.append(pixel&0xFF)
        mask >>= 1   #go to next bit in column
    
    spi.writebytes(buf)  #write all 96 pixels to TFT

def PutCh8x12 (ch,xPos,yPos,color): #Put an individual character to the screen using 8x12 font
    yPos = 240-yPos #compensate for rotating display. Puts 0,0 in top left hand corner
    charData = font8x12.data[ord(ch)-32]    #get the character data
    SetAddrWindow(xPos,yPos-11,xPos+7,yPos)  #set display window to 8x12 rect
    SetPin(DC,0)
    spi.writebytes([RAMWR])  #pixel data follows
    SetPin(DC,1)
    buf = []                 #setup buffer for pixel data
     
    for row in range(12):     #iterate over all 96 pixels
        mask = 0x80              #start with msb in each column
        for col in range(8):   #do all columns for each row
            bit = charData[11-row] & mask  #get the next bit
            if (bit==0):         #is it a 0 or 1?
                pixel = bColor   #0: use background color
            else:
                pixel = color    #1: use foreground color
            buf.append(pixel>>8)
            buf.append(pixel&0xFF)
            mask >>= 1   #go to next bit in column
    
    spi.writebytes(buf)  #write all 96 pixels to TFT

def PutLine5x7 (string,xPos,yPos,color):
    for a in string:
        PutCh5x7 (a,xPos,yPos,color)
        xPos = xPos + 6

def PutLine8x12 (string,xPos,yPos,color):
    for a in string:
        PutCh8x12 (a,xPos,yPos,color)
        xPos = xPos + 8

def DrawHeader():
    global bColor #to change background colour. remember to put it back again after!
    FillRect(0,0,320,30,BLACK)
    bColor = BLACK
    PutLine8x12('BIG FACE ROBOTICS - BFRMR1',40,14,RED)
    bColor = LIGHTBLUE

########################################################################
#
# Screen routines:
#
########################################################################


def StartupScreen():
    ClearScreen()
    DrawHeader()   


########################################################################
#
# Start and stop routines for TFT display

def stoptft():
    spi.close()
    print "TFT stopped"


########################################################################
#
# Main: This runs when script is imported. Sets up spi and draws
# start screen to tft


print "Adafruit 2.2 TFT display driver started"
spi = spidev.SpiDev()
spi.open(0,0)
spi.mode = 0
spi.max_speed_hz = 20000000
InitIO()
InitDisplay()
SetOrientation(270)
StartupScreen()  #display Basic screen on start-up






