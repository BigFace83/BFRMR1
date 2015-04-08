#include <Servo.h> 

Servo leftwheel;
Servo rightwheel;
Servo headpan;
Servo headtilt;

unsigned char datapacket[8] = {0,0,0,0,0,0,0,0};
unsigned char leftencoder;
unsigned char leftencodertotal;
unsigned char rightencoder;
unsigned char rightencodertotal;
unsigned char headpanposition = 128; //initialise to 128 for central position
unsigned char headtiltposition = 128;
unsigned char toneselection = 0;
unsigned char sonarthreshold;
unsigned char irthreshold;
unsigned char headspeed;

unsigned char encodercount = 0;
unsigned char wheelspeed = 0;
unsigned char robotdirection;

boolean leftwheelforward = true;
boolean rightwheelforward = true;
unsigned char objectdetected = 0;

int leftwheelspeed;
int rightwheelspeed;

int headpanms = 1500;
int headtiltms = 1500;

unsigned char preamble1;
unsigned char preamble2; 

int leftwheelsetpoint = 0;
int rightwheelsetpoint = 0;

unsigned long previousMilliswheels = 0; //variables for servo and wheel pid loops
unsigned long previousMillisservos = 0;

/**************************************
Interrupt handlers for encoders
**************************************/
void incrementleft()
{
  leftencoder ++;
  leftencodertotal ++;
}

void incrementright()
{
  rightencoder ++;
  rightencodertotal ++;
}


void setup() 
{ 
  
  
  Serial.begin(9600); //start serial communication
  
  headpan.attach(2);
  headtilt.attach(3);
  leftwheel.attach(4);
  rightwheel.attach(5);

  attachInterrupt(2, incrementleft, CHANGE);
  attachInterrupt(3, incrementright, CHANGE);

  pinMode(28, OUTPUT);
  pinMode(13, OUTPUT);
  
  headpan.writeMicroseconds(1500); //set servos to centre position
  headtilt.writeMicroseconds(1500);
  leftwheel.writeMicroseconds(1500);
  rightwheel.writeMicroseconds(1500);
  
  playtone3();
  
  digitalWrite(13, LOW); //onboard led off
  
  

} 


void loop() 
{ 
  
  if (Serial.available()>=7)
  {
    
    digitalWrite(28, HIGH); //servo power relay on
    digitalWrite(13, HIGH); //onboard led on
    preamble1 = Serial.read(); 
    if(preamble1==255)
    {
      
      preamble2 = Serial.read();
      if(preamble2==255)
      {
        unsigned char instruction = Serial.read();
        switch(instruction){
          case 0: //get data - return a packets of all sensor data
            Serial.read(); //read extra bytes of data to tidy up
            Serial.read();
            Serial.read();
            Serial.read();
            readsensors();
            writeserialdata(datapacket);
            break;
          case 1: //robot forward
            encodercount = Serial.read();
            wheelspeed = Serial.read();
            sonarthreshold = Serial.read();
            irthreshold = Serial.read();
            robotdirection = 1;
            moverobot(robotdirection,encodercount, wheelspeed,sonarthreshold, irthreshold);
            writeserialdata(datapacket);
            leftencodertotal = 0; //after sending data, clear encoder totals
            rightencodertotal = 0;
            break;
          case 2: //robot left
            encodercount = Serial.read();
            wheelspeed = Serial.read();
            sonarthreshold = Serial.read();
            irthreshold = Serial.read();
            robotdirection = 2;
            moverobot(robotdirection,encodercount, wheelspeed,sonarthreshold, irthreshold);
            writeserialdata(datapacket);
            leftencodertotal = 0; //after sending data, clear encoder totals
            rightencodertotal = 0;
            break;
          case 3: //robot right
            encodercount = Serial.read();
            wheelspeed = Serial.read();
            sonarthreshold = Serial.read();
            irthreshold = Serial.read();
            robotdirection = 3;
            moverobot(robotdirection,encodercount, wheelspeed,sonarthreshold, irthreshold);
            writeserialdata(datapacket);
            leftencodertotal = 0; //after sending data, clear encoder totals
            rightencodertotal = 0;
            break;
          case 4: //robot reverse
            encodercount = Serial.read();
            wheelspeed = Serial.read();
            sonarthreshold = Serial.read();
            irthreshold = Serial.read();
            robotdirection = 4;
            moverobot(robotdirection,encodercount, wheelspeed,sonarthreshold, irthreshold);
            writeserialdata(datapacket);
            leftencodertotal = 0; //after sending data, clear encoder totals
            rightencodertotal = 0;
            break;  
          case 5: //Head move
            headpanposition = Serial.read();
            headtiltposition = Serial.read();
            headspeed = Serial.read();
            Serial.read();
            movehead(headpanposition, headtiltposition, headspeed);          
            readsensors();
            writeserialdata(datapacket);
            break;
          case 6: //Play tone
            toneselection = Serial.read();
            Serial.read();
            Serial.read();
            Serial.read();
            switch(toneselection){
              case 0:
                playtone0();
                break;
              case 1:
                playtone1();
                break;  
            }
            break;    
        }
      
      }

    }
  } 
}  


void movehead(unsigned char headpanposition, unsigned char headtiltposition, unsigned char headspeed)
{
  
  int headpantarget = map(headpanposition, 0 ,255 ,600 ,2400);  
  int headtilttarget = map(headtiltposition, 0 ,255 ,600 ,2400);
  
  while(true){
    
        unsigned long currentMillis = millis();
        
        if(currentMillis - previousMillisservos > (10-headspeed)) //use this to determine frequency of servo loop
        {
            // save the last time pid loop was called
            previousMillisservos = currentMillis;
            
            if(headpantarget > headpanms)
            { 
              headpanms = headpanms + 3;
              headpan.writeMicroseconds(headpanms);
            }
            else if(headpantarget < headpanms)
            {
              headpanms = headpanms - 3;
              headpan.writeMicroseconds(headpanms);
            }
                
                
            
            if(headtilttarget > headtiltms)
            { 
              headtiltms = headtiltms + 3;
              headtilt.writeMicroseconds(headtiltms);
            }
            else if(headtilttarget < headtiltms)
            {
              headtiltms = headtiltms - 3;
              headtilt.writeMicroseconds(headtiltms);
            }     
               
                 
        }
        int differencepan = headpantarget - headpanms; 
        int differencetilt = headtilttarget - headtiltms;
        
        if(abs(differencepan) < 3 && abs(differencetilt) < 3) //if both servo positions have been reached
        {
          break; //exit loop and return
        }
  }

 
}

void moverobot(unsigned char robotdirection,unsigned char encodercount, unsigned char wheelspeed, unsigned char sonarthreshold, unsigned char irthreshold)
{
  
  //movehead(128,128,6); //centre head before moving
  readsensors(); // Get latest readings before setting off. If this isn't done, robot may stop first time through the loop if
                 // last readings were below thresholds
  int pgain = 4;
  if(robotdirection == 1) //robot forward
  {
      leftwheelsetpoint = wheelspeed;
      rightwheelsetpoint = wheelspeed;
  }
  else if(robotdirection == 2) //robot left
  {
      leftwheelsetpoint = -wheelspeed;
      rightwheelsetpoint = wheelspeed;
  }
  else if(robotdirection == 3) //robot right
  {
      leftwheelsetpoint = wheelspeed;
      rightwheelsetpoint = -wheelspeed;
  }
  else if(robotdirection == 4) //robot reverse
  {
      leftwheelsetpoint = -wheelspeed;
      rightwheelsetpoint = -wheelspeed;
  }
  
  //initialize all encoder counts to zero and wheel speeds to stopped before
  //entering pid loop
  leftencodertotal = 0;
  rightencodertotal = 0;
  leftencoder = 0;
  rightencoder = 0;
  leftwheelspeed = 1500;
  rightwheelspeed = 1500;
  
  
  while(true){
    
        unsigned long currentMillis = millis();
        
        if(currentMillis - previousMillisservos > 200) //use this to determine frequency of servo loop
        {
            // save the last time pid loop was called
            previousMillisservos = currentMillis;
            
            /*******************************************************************************************
            * Control loop for left wheel   
            *******************************************************************************************/

           if(leftwheelsetpoint>0)
           {
               double perror = leftwheelsetpoint - leftencoder;
               leftencoder = 0;
               leftwheelspeed = leftwheelspeed + ((double)pgain*perror);
           }
    
           else if(leftwheelsetpoint<0)
           {
               double perror = -leftwheelsetpoint - leftencoder;
               leftencoder = 0;
               leftwheelspeed = leftwheelspeed - ((double)pgain*perror);
           }
    

           /*******************************************************************************************
           * Control loop for right wheel   
           *******************************************************************************************/
      
           if(rightwheelsetpoint>0)
           {
               double perror = rightwheelsetpoint - rightencoder;
               rightencoder = 0;
               rightwheelspeed = rightwheelspeed - ((double)pgain*perror);
           }
           
           else if(rightwheelsetpoint<0)
           {
               double perror = -rightwheelsetpoint - rightencoder;
               rightencoder = 0;
               rightwheelspeed = rightwheelspeed + ((double)pgain*perror);  
           }
    
    
          leftwheel.writeMicroseconds(leftwheelspeed);
          rightwheel.writeMicroseconds(rightwheelspeed);
          
        }
        
        /*******************************************************************************************
        * Check to see if encoder setpoints have been reached
        * If they have, stop the wheels and return
        *******************************************************************************************/
        if(leftencodertotal>=encodercount) //if encoder target has been reached
        {
          leftwheelsetpoint = 0; //put setpoint to zero
          leftwheel.writeMicroseconds(1500); //stop the wheels
        }
        if(rightencodertotal>=encodercount) //if encoder target has been reached
        {
          rightwheelsetpoint = 0; //put setpoint to zero
          rightwheel.writeMicroseconds(1500); //stop the wheels
        }
        if(leftencodertotal >= encodercount && rightencodertotal >= encodercount) //if both wheels have reached their target
        {
          leftwheel.writeMicroseconds(1500); //stop the wheels
          rightwheel.writeMicroseconds(1500); //as a double check that wheels are stopped
          break; //return from while loop
        }
        
        /*******************************************************************************************
        * Check for objects whilst moving. If an object is detected, stop the wheels and return
        *******************************************************************************************/
        readsensors();
        if(datapacket[0]>irthreshold || datapacket[1]>irthreshold  || datapacket[2]>irthreshold  || datapacket[5]<sonarthreshold) //if obstacle is detected by ir sensors or sonar
        {
          leftwheelsetpoint = 0; //put setpoint to zero
          leftwheel.writeMicroseconds(1500); //stop left wheel
          rightwheelsetpoint = 0; //put setpoint to zero
          rightwheel.writeMicroseconds(1500);//stop right wheel
          
          break;
        }
   
   
   
        
  }   
}


void readsensors() //Read all of the sensors and form data into a packet ready to send
{
  
  //read IR sensors and servo feedback
  unsigned char leftlookingIR = analogRead(2)/4; //divide by four to keep value within range of unsigned char
  unsigned char centreIR = analogRead(3)/4;
  unsigned char rightlookingIR = analogRead(4)/4;
  unsigned char headpanservo = analogRead(5)/4;
  unsigned char headtiltservo = analogRead(6)/4;
  
  //read head sonar sensor
  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);             // Make sure pin is low before sending a short high to trigger ranging
  delayMicroseconds(2);
  digitalWrite(38, HIGH);            // Send a short 10 microsecond high burst on pin to start ranging
  delayMicroseconds(10);
  digitalWrite(38, LOW);             // Send pin low again before waiting for pulse back in
  pinMode(38, INPUT);
  int duration = pulseIn(38, HIGH);  // Reads echo pulse in from SRF05 in micro seconds
  int headsensor = (duration/58);      // Dividing this by 58 gives us a distance in cm
  if(headsensor > 255)
  {
    headsensor = 255;
  }
  
  //form data into a data packet array
  datapacket[0] = leftlookingIR;
  datapacket[1] = centreIR;
  datapacket[2] = rightlookingIR;
  datapacket[3] = headpanservo;
  datapacket[4] = headtiltservo;
  datapacket[5] = headsensor;
  datapacket[6] = leftencodertotal;  //send encoder totals, but reset to zero following send
  datapacket[7] = rightencodertotal; //encoder count sent to pc is total since last send
                 
  
  
  return;
  

}

void writeserialdata(unsigned char datapacket[])
{
      Serial.write(255);
      Serial.write(255);

      for(int i = 0;i<8;i++)    //write datapacket values to serial
      {
        Serial.write(datapacket[i]);
      }
}


void playtone0()
{ 
    for(int i=100; i<600; i=i+10)
    {
       tone(12, i, 10);
       delay(5);
       
    }
    // stop the tone playing:
    //noTone(12);
}
void playtone1()
{ 
    tone(12, 300, 100);
    delay(100);
    tone(12, 200, 100);
    delay(100);
    tone(12, 100, 100);
    delay(100);
    // stop the tone playing:
    //noTone(12);
}
void playtone2()
{ 
    tone(12, 300, 20);
    delay(20);
    
}
void playtone3()
{ 
    tone(12, 500, 20);
    delay(20);
    tone(12, 400, 20);
    delay(20);
    tone(12, 300, 20);
    delay(20);
    tone(12, 400, 20);
    delay(20);
    tone(12, 300, 20);
    delay(20);
    tone(12, 500, 20);
    delay(20);
    
}

