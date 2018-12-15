#include <Esplora.h>

char RoutineSelect = 'R';
char OperationMode = 'O';
int R = 0;
int L = 0;
int X = 0;
int Y = 0;

String StartString = "<";
String comma = ",";
String semicolon = ";";
String exclamation = "!";
String questionMark = "?";
String atSymb = "@";
String CloseString = ">";

String data;

void setup(){
  Serial.begin(9600);
  data = String("<>");
  }



void loop()
{

if(!Esplora.readButton(1)){
  if(!Esplora.readButton(2) || !Esplora.readButton(4)){
    RoutineSelect = 'a';
    if(!Esplora.readButton(2)){
      L = 250;
    }
    if(!Esplora.readButton(4)){
      R = 250;
    }
   }else{
    RoutineSelect = 'd';
    L = 0;
    R = 0;
    }
  }else{
    RoutineSelect = '!';
  }

if(!Esplora.readButton(3)){
  OperationMode = 'a';
}else{
  OperationMode = 'n';
}

X = Esplora.readJoystickX();
Y = Esplora.readJoystickY();

data = StartString + RoutineSelect + comma + X + semicolon + Y + exclamation + L + questionMark + R + atSymb +  OperationMode + CloseString;
Serial.print(data);
//delay(200);

}
