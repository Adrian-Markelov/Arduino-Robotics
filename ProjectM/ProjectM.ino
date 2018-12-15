/* Project Mecanum

Example control software for Susan-M, the mecanum drive demo robot.
Direct RC control works by interpreting the PWM inputs from the
radio receiver, as though the arduino was a servo mechanism, and
producing PWM drive to the Jaguar motor controllers.  This is not
a FIRST compatible robot project, as no *rio is involved and no
connection to the field system is possible.  Similarly, there is
no Enable/Disable system, making this significantly less safe than
a FIRST robot.

Users Beware

Rev 0 4/27/15 RSaunders

Programmed to be built with Arduino v1.0.6

*/

#include <Servo.h>
Servo LtArmServo;
Servo RtArmServo;

unsigned long currentTime;
unsigned int err;
// Blinky LED
const int LEDpin = 13;
boolean LEDflipFlop;
//  PWM inputs
boolean PWMenabled;
const int XmovePWM = 4;
const int YmovePWM = 2;
const int throtPWM = 7;
const int spinPWM = 8;
const float PWMmax = 1960.0-980.0;
const int PWMmin = 980;
float XmoveNOW;
float YmoveNOW;
float throtNOW;
float spinNOW;
float FwdRevSpd;
float RhtLftSpd;
float SpinSpd;
float LFmotor;
float LRmotor;
float RFmotor;
float RRmotor;
float MotorLimit;

// Output Servos via PWM
const int LtArmServoPin = 5;
const int RtArmServoPin = 6;
const int servoPWMmin = 544;
const int servoPWMmax = 2400;
//int servoPWMnowA;
//int servoPWMdirA;


// Single threaded self scheduling with three levels
unsigned long fastFrameLast;
unsigned long midFrameLast;
unsigned long idleFrameLast;
const long midFrameTime = 100;    // 100 ms = 10Hz.  Since PWM read takes ~12ms times 4, don't set below 60
const long idleFrameTime = 1000;  // 1000 ms = 1 Hz
// CMA-10 Average frame run times
const float CMAlength = 10.0;
float fastFrameAve;
float midFrameAve;
float idleFrameAve;

// Debug Print control flags
unsigned int debugCTL;
const int debugDefault = 9;
const int DBframeAve = 1;  // A
const int DBfastCount = 2;  // b
const int DBshowSign = 4;  // C
const int DBnoPWMs = 8;  // D
const int DBthrotPWM = 16;  // E
const int DBjoysticks = 32;  // F
const int DBspeeds = 64;  // G
const int DBjags = 128;  // H
int fastFrameCount;


void setup(){
  Serial.begin(57600);
  Serial.println("Project M Control Program v0.6");
  pinMode(LEDpin, OUTPUT); // the onboard LED
  LEDflipFlop = false;
  pinMode(XmovePWM, INPUT);
  pinMode(YmovePWM, INPUT);
  pinMode(throtPWM, INPUT);
  pinMode(spinPWM, INPUT);
  fastFrameLast = 0;
  midFrameLast = 0;
  idleFrameLast = 0;
  fastFrameAve = 0.0;
  midFrameAve = 0.0;
  idleFrameAve = 0.0;
  debugCTL = debugDefault;
  fastFrameCount = 0;
  RtArmServo.attach(RtArmServoPin,servoPWMmin,servoPWMmax);
  LtArmServo.attach(LtArmServoPin,servoPWMmin,servoPWMmax);
  //pinMode(servoPinA, OUTPUT);
  //servoPWMnowA = 0;
  //servoPWMdirA = 10;
}
void loop(){
  
  // the Fast process runs very often, but with no fixed frame rate
  fastFrameLast = micros();
  err = fastFrame();
  currentTime = micros();
  if (currentTime > fastFrameLast)  // in case micros wrapped around (after 70min when timmer goes back to 0)
    fastFrameAve = (fastFrameAve*(CMAlength-1.0)+(float)(currentTime-fastFrameLast))/CMAlength;
  currentTime = millis();
  
  // the Mid task runs at a regular pace
  if ((currentTime - midFrameLast) > midFrameTime) {
    midFrameLast = currentTime;
    err = midFrame();
    currentTime = millis();
    midFrameAve = (midFrameAve*(CMAlength-1.0)+(float)(currentTime-midFrameLast))/CMAlength;
  }
  else // Mid and Idle can't run back-to-back as this might starve Fast
    if ((currentTime - idleFrameLast) > idleFrameTime) {
    idleFrameLast = currentTime;
    err = idleFrame();
    currentTime = millis();
    idleFrameAve = (idleFrameAve*(CMAlength-1.0)+(float)(currentTime-idleFrameLast))/CMAlength;
  }
}  // Main Loop


//***********************************************************************************************
//***********************************************************************************************
//***********************************************************************************************






// Process code for the Fast Process
int fastFrame(){
  int nextByte;
  boolean OnOff;
  int maskBit;
  fastFrameCount++;
  if (Serial.available() > 0) {
    nextByte = Serial.read();
    // Letters a to o turn on a debug and A to O turn it off.  All others mean nothing.
    if ((nextByte > 64) && (nextByte < 112) && !((nextByte > 79) && (nextByte < 97))) {
      OnOff = ((nextByte & 32) != 0);  // On if lower case. Off if upper case
      maskBit = 1 << ((nextByte & 15)-1);
      if (OnOff) {
        debugCTL = debugCTL | maskBit;
      } else {
        debugCTL = debugCTL & (~maskBit);
      }
    } 
  }
  return 0;
}

// Process code for the Mid Process
int midFrame(){
  int PWMinput;
  const long PWMwait = 50000;
  
  int LFfwd = 1;  //  Wheel specific motor directions,  What is fwd?  What is right?
  int LFrht = 1;
  int LRfwd = 1;
  int LRrht = -1;
  int RFfwd = -1;
  int RFrht = -1;
  int RRfwd = -1;
  int RRrht = 1;
   
  if (LEDflipFlop) {
    LEDflipFlop = false;
    digitalWrite(LEDpin, HIGH);
  }
  else
  {
    LEDflipFlop = true;
    digitalWrite(LEDpin, LOW);
  }
  
  // Read PWM Signals
  
  if(PWMenabled) {
    PWMinput = pulseIn(throtPWM, HIGH, PWMwait);
    if (PWMinput == 0) {
      Serial.print("PWM Error on pin #");
      Serial.print(throtPWM);
      Serial.println(" Turning off PWMs");
      PWMenabled = false;
    } 
    throtNOW = ((float)(PWMinput-PWMmin))/PWMmax;
    if((debugCTL & DBthrotPWM) != 0) {
      Serial.print("Throttle PWM reads=");
      Serial.print(PWMinput);
      Serial.print(":");
      Serial.println(throtNOW);
    }
    PWMinput = pulseIn(XmovePWM, HIGH, PWMwait);
    if (PWMinput == 0) {
      Serial.print("PWM Error on pin #");
      Serial.print(XmovePWM);
      Serial.println(" Turning off PWMs");
      PWMenabled = false;
    } 
    XmoveNOW = ((float)(PWMinput-PWMmin))/PWMmax;
    PWMinput = pulseIn(YmovePWM, HIGH, PWMwait);
    if (PWMinput == 0) {
      Serial.print("PWM Error on pin #");
      Serial.print(YmovePWM);
      Serial.println(" Turning off PWMs");
      PWMenabled = false;
    } 
    YmoveNOW = ((float)(PWMinput-PWMmin))/PWMmax;
    PWMinput = pulseIn(spinPWM, HIGH, PWMwait);
    if (PWMinput == 0) {
      Serial.print("PWM Error on pin #");
      Serial.print(spinPWM);
      Serial.println(" Turning off PWMs");
      PWMenabled = false;
    } 
    spinNOW = ((float)(PWMinput-PWMmin))/PWMmax;
    if((debugCTL & DBjoysticks) != 0) {
      Serial.print("Joystick Values: Throt=");
      Serial.print(throtNOW);
      Serial.print(" X=");
      Serial.print(XmoveNOW);
      Serial.print(" Y=");
      Serial.print(YmoveNOW);
      Serial.print(" Spin=");
      Serial.println(spinNOW);
    }
    FwdRevSpd = (XmoveNOW-0.5)*throtNOW*2.0;
    RhtLftSpd = (YmoveNOW-0.5)*throtNOW*2.0;
    SpinSpd = (spinNOW-0.5)*throtNOW*1.5;  // Spin has a little less gain

    if((debugCTL & DBspeeds) != 0) {
      Serial.print("Commanded Speeds: Fwd/Rev=");
      Serial.print(FwdRevSpd);
      Serial.print(" Right/Left=");
      Serial.print(RhtLftSpd);
      Serial.print(" Spin=");
      Serial.println(SpinSpd);
    }
    LFmotor = LFfwd*FwdRevSpd + LFrht*RhtLftSpd + LFfwd*SpinSpd;
    LRmotor = LRfwd*FwdRevSpd + LRrht*RhtLftSpd + LFfwd*SpinSpd;
    RFmotor = RFfwd*FwdRevSpd + RFrht*RhtLftSpd - RFfwd*SpinSpd;
    RRmotor = RRfwd*FwdRevSpd + RRrht*RhtLftSpd - RRfwd*SpinSpd;
    MotorLimit = max(max(abs(LFmotor),abs(LRmotor)),max(abs(RFmotor),abs(RRmotor)));
    if (MotorLimit > 1) {
      LFmotor = LFmotor/MotorLimit;
      LRmotor = LRmotor/MotorLimit;
      RFmotor = RFmotor/MotorLimit;
      RRmotor = RRmotor/MotorLimit;
    }   
    if((debugCTL & DBjags) != 0) {
      Serial.print("Motor Commands: LF=");
      Serial.print(LFmotor);
      Serial.print(" LR=");
      Serial.print(LRmotor);
      Serial.print(" RF=");
      Serial.print(RFmotor);
      Serial.print(" RR=");
      Serial.println(RRmotor);
      
      // Output signals on PWMs 11, 10, 9, and 3 goes here
    }
  }
  else {
      //  PWMs can be ignored, for testing without them
    if((debugCTL & DBthrotPWM) != 0) {
      Serial.println("PWMs are off");
    }
  }
  
  err = SemaphoreTask();  // Wave those flags
  return err;
}

// Process code for the Idle Process
int idleFrame(){
// Debug Frame Average Times
  if((debugCTL & DBframeAve) != 0) {
    Serial.print("Frame Averages - Fast=");
    Serial.print(fastFrameAve);
    Serial.print("us  Mid=");
    Serial.print(midFrameAve);
    Serial.print("ms  Idle=");
    Serial.print(idleFrameAve);
    Serial.println("ms");
  }
  if((debugCTL & DBfastCount) != 0) {
    Serial.print("Fast Frame Count=");
    Serial.println(fastFrameCount);
    fastFrameCount = 0;
  }
  if((debugCTL & DBnoPWMs) != 0) {
    PWMenabled = false;
  } 
  else  {
    PWMenabled = true;
  }
  
  return 0;
}

// Semaphore Data
boolean SnumFlag = true;
byte Snums[] = {51,19,20,21,22,42,34,26,12,59};
byte Sletrs[] = {18,19,20,21,22,42,34,26,12,59,38,51,43,35,27,60,52,44,36,28,53,45,30,39,31,37,33};
byte SnumOnOff[] = {53,38};
byte SshowTime = 30;
byte StimeShown = 0;
byte WtimeShown = 0;
char Smessage[] = "~Welcome Team 2537";
byte SmessChar = 0;
byte SleftFlag;
byte SrightFlag;
int RtArmAngle = 90;
int LtArmAngle = 90;
int LastArmAngle = 0;

// Routine that sends semaphore flag messages
int SemaphoreTask(){
  char nextChar;
  byte nextLetter;
  byte nextFlag;
  
  if(StimeShown++ > SshowTime){
    StimeShown = 0;
    nextChar = Smessage[SmessChar];
    if (nextChar == 0) {
      SmessChar = 0;
      nextChar = Smessage[0];
    }
    SmessChar++;
    
    if (nextChar == ' '){
      nextFlag = Sletrs[0];
    }
    else {
      if ((nextChar >= '0') && (nextChar <= '9')) {
        if (SnumFlag) {
          nextFlag = Snums[(nextChar-'0')];
        }
        else {
          nextFlag = SnumOnOff[0];
          nextChar = '#';
          SnumFlag = true;
          SmessChar--;
        }
      }
      else {  // It's a letter
        nextLetter = nextChar & 31;
        if ((nextLetter == 0) || (nextLetter > 26)) {
          nextFlag = 0;
        }
        else {
          if (!SnumFlag) {
            nextFlag = Sletrs[nextLetter];
          }
          else {
            nextFlag = SnumOnOff[1];
            nextChar = '@';
            SnumFlag = false;
            SmessChar--;
          }
        }
      }
    }
    SleftFlag = nextFlag >> 3;
    SrightFlag = nextFlag & 7;
    if((debugCTL & DBshowSign) != 0) {
      Serial.print("New Sign:");
      Serial.print(nextChar);
      Serial.print(": Left=");
      Serial.print(SleftFlag);
      Serial.print(" Right=");
      Serial.println(SrightFlag);
    } 
  }
     
  // Generate output
  if((debugCTL & DBshowSign) != 0) {
    if (SleftFlag == 0) {
    // wiggle flags
      if (LastArmAngle == 0) {
        LtArmAngle = 130;
        RtArmAngle = 50;
        LastArmAngle = 1;
        WtimeShown = 0;
      }
      else {
        if (WtimeShown++ > 5) {
          WtimeShown = 0;
          LastArmAngle = LtArmAngle;
          LtArmAngle = RtArmAngle;
          RtArmAngle = LastArmAngle;
        }
      }
    }
    else {
      // show a letter
      LtArmAngle = 30*(SleftFlag-1);
      RtArmAngle = 30*(7-SrightFlag);
      LastArmAngle = 0;
    }
    LtArmServo.write(LtArmAngle);
    RtArmServo.write(RtArmAngle);
  }
  return 0;
}



