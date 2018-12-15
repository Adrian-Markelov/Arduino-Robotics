// has therrian mod 

#define AVECNT 5

unsigned long pulse_width=0;
unsigned long sum=0;
unsigned long ave=0;
unsigned long value[AVECNT]={0};
int ptr=0;
int a=1;

void setup(){
  Serial.begin(9600);
  //Add str for -1 values to print
  pinMode(3,INPUT);
  for(int i=0;i<AVECNT;i++){
    pulse_width=0;
    while(pulse_width==0){  //We need to replace as it stops everything if broken
      pulse_width=pulseIn(3,HIGH);
      delay(100);
      if(a==2){
        break;
      }
    }
    value[i]=pulse_width;
    sum+=pulse_width;
    Serial.print(value[i]);  //debug
    Serial.print(" of ");    //debug
    Serial.println(sum);     //debug
  }
  ave = sum / AVECNT;
  Serial.println(ave);
  //Print valid first line out
}

void loop(){
  pulse_width=0;
  a=1;
  while(pulse_width==0){   //We need to replace as it stops everything if broken
    pulse_width=pulseIn(3,HIGH)/10;
    delay(100);
    if(a==2){
      break;
    }
    a++;
  }
  sum = sum - value[ptr];
  value[ptr++] = pulse_width;
  sum = sum + pulse_width;
  Serial.println(sum/AVECNT);
 // Serial.println(sum);
  
  if(ptr==AVECNT){
    ptr=0;
  }
}
