
unsigned int ultraPwmFrontPin = 2;
unsigned int ultraTriggerFrontPin = 52;
unsigned int ultraPwmBackPin = 2;
unsigned int ultraTriggerBackPin = 52;

unsigned int in1_harvesterPin = 1;
unsigned int in2_harvesterPin = 1;
unsigned int enA_harvesterPin = 1; 

unsigned int in1_leftBackPin = 1;
unsigned int in2_leftBackPin = 1;
unsigned int enA_leftBackPin = 1;

unsigned int in1_rightBackPin = 1;
unsigned int in2_rightBackPin = 1;
unsigned int enA_rightBackPin = 1;

unsigned int in1_leftForwardPin = 1;
unsigned int in2_leftForwardPin = 1;
unsigned int enA_leftForwardPin = 1;

unsigned int in1_rightForwardPin = 1;
unsigned int in2_rightForwardPin = 1;
unsigned int enA_rightForwardPin = 1;

unsigned int servoLeftPin = 1;
unsigned int servoRightPin = 1;


String controllerData = "<>";

char routineDecision = 'b';
/*for routineDecision
 * a = arm
 * d = drive
 */
char operationMode = 'n';
/* for operationMode
 * n = null 
 * a = autonomous
 * t = teleop
 */

void setup() {
  // put your setup code here, to run once:
  pinMode(ultraPwmFrontPin, INPUT);
  pinMode(ultraTriggerFrontPin,OUTPUT);

  pinMode(ultraPwmBackPin, INPUT);
  pinMode(ultraTriggerBackPin,OUTPUT);

  pinMode(in1_harvesterPin,OUTPUT);
  pinMode(in2_harvesterPin,OUTPUT);
  pinMode(enA_harvesterPin,OUTPUT);
  
  pinMode(in1_leftBackPin,OUTPUT);
  pinMode(in2_leftBackPin,OUTPUT);
  pinMode(enA_leftBackPin,OUTPUT);

  pinMode(in1_rightBackPin,OUTPUT);
  pinMode(in2_rightBackPin,OUTPUT);
  pinMode(enA_rightBackPin,OUTPUT);

  pinMode(in1_leftForwardPin,OUTPUT);
  pinMode(in2_leftForwardPin,OUTPUT);
  pinMode(enA_leftForwardPin,OUTPUT);

  pinMode(in1_rightForwardPin,OUTPUT);
  pinMode(in2_rightForwardPin,OUTPUT);
  pinMode(enA_rightForwardPin,OUTPUT);
  
  pinMode(servoLeftPin,OUTPUT);
  pinMode(servoRightPin,OUTPUT);
  
  Serial.begin(9600);

  // ask if you want to go into autonomous 
  // if not skip autonomous routine and go to teleop
  // routine in the loop
  while(Serial.readStringUntil('<')){}// start char
  controllerData = Serial.readStringUntil('<');
  
  while(operationMode == 'n'){
      operationMode = parseOperationMode(controllerData); // check each iteration if we should go to either teleop of autonomous
      if(operationMode == 'a'){
        runAutonomousRoutine();  // run all auto code in one
        break;
      }
  digitalWrite(in1_harvesterPin, HIGH);
  digitalWrite(in2_harvesterPin, LOW);
  }
  
}


// loop is teleop routine
void loop() {

while(Serial.readStringUntil('<')){}// start char
controllerData = Serial.readStringUntil('>');
routineDecision = parseRoutineSelection(controllerData);

//------------------------------------

if(routineDecision == 'd'){
  driveRoutine(parseX(controllerData),parseY(controllerData));
} else if (routineDecision == 'a'){
  armRoutine(parseLeftServo(controllerData), parseRightServo(controllerData));
}

// harvester set laways on
analogWrite(enA_harvesterPin,180);

}


/*
 * Data Parseers below ****************************************
 */

char parseRoutineSelection(String data){
  int start = data.indexOf('<') + 1;
  
  char routineSelect = data.charAt(start);
  return routineSelect;
}

int parseX(String data){
  int start = data.indexOf(',');
  int last = data.indexOf(';');
  String xString = data.substring(start,last);
  return xString.toInt();
}

int parseY(String data){
  int start = data.indexOf(';');
  int last = data.indexOf('!');
  String yString = data.substring(start,last);
  return yString.toInt();
}

int parseLeftServo(String data){
  int start = data.indexOf('!');
  int last = data.indexOf('?');
  String LString = data.substring(start,last);
  return LString.toInt();
}

int parseRightServo(String data){
  int start = data.indexOf('?');
  int last = data.indexOf('@');
  String RString = data.substring(start,last);
  return RString.toInt();
}

char parseOperationMode(String data){
  int start = data.indexOf('@') + 1;
  
  char OperationMode = data.charAt(start);
  return OperationMode;
}




/* 
 *  Routine functions below ***********************
 */
void runAutonomousRoutine(){
  digitalWrite(in1_leftForwardPin, HIGH);
  digitalWrite(in2_leftForwardPin, LOW);
  digitalWrite(in1_rightForwardPin, HIGH);
  digitalWrite(in2_rightForwardPin, LOW);
  long timmerInit = millis();
  long timmerDiff = 0;
  while(timmerDiff < 3000){
    analogWrite(enA_leftForwardPin,200);
    analogWrite(enA_rightForwardPin,200);
    timmerDiff = millis() - timmerInit; 
  }

}

void driveRoutine(int x, int y){
  if (y>0){
    digitalWrite(in1_leftForwardPin, HIGH);
    digitalWrite(in1_rightForwardPin, HIGH);
    digitalWrite(in2_leftForwardPin, LOW);
    digitalWrite(in2_rightForwardPin, LOW);
    if(x>0){
      analogWrite(enA_leftForwardPin,y/2);
    }else if (x<0){
      analogWrite(enA_rightForwardPin,y/2);
    }else{
      analogWrite(enA_leftForwardPin,y/2);
      analogWrite(enA_rightForwardPin,y/2);
    }
  }else if(y < 0){
    digitalWrite(in1_leftBackPin, LOW);
    digitalWrite(in1_rightBackPin, LOW);
    digitalWrite(in2_leftForwardPin, HIGH);
    digitalWrite(in2_rightForwardPin, HIGH);
      if(x>0){
        analogWrite(enA_leftBackPin,x/2);
      }else{
        analogWrite(enA_rightBackPin,x/2);
      }
  }else{
       if(x>0){
        digitalWrite(in1_leftBackPin, HIGH);
        digitalWrite(in2_leftBackPin, LOW);
        digitalWrite(in1_leftForwardPin, HIGH);
        digitalWrite(in1_leftForwardPin, LOW);
        analogWrite(enA_leftBackPin,x/2);
        analogWrite(enA_leftForwardPin,x/2);
       }else{
        digitalWrite(in1_rightBackPin, HIGH);
        digitalWrite(in2_rightBackPin, LOW);
        digitalWrite(in1_rightForwardPin, HIGH);
        digitalWrite(in1_rightForwardPin, LOW);
        analogWrite(enA_rightBackPin,x/2);
        analogWrite(enA_rightForwardPin,x/2);
        }
  
  }
}

void armRoutine(int left, int right){
analogWrite(servoLeftPin,left);
analogWrite(servoRightPin,right);
  
}
