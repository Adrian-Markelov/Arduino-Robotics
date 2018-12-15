#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_9DOF.h>

/*    
 Connections (For I2C)
 ===========
 Connect SCL to analog 5
 Connect SDA to analog 4
 Connect VDD to 5V DC
 Connect GROUND to common ground
 
 Monitor angles on upright and arm.
 Returns height, angle, + or - tilt angle
 */
#define ARMIMU 0
#define UPIMU 1
#define AVECNT 10
#define ANGOFFSET 90
#define ANGSTART 23
#define IMUADDRESS 29
/*
ARMIMU  
 port on the multiplexer; Arm
 UPIMU
 port on the multiplexer; Up vertical
 AVECNT
 avg 10 to smooth the bumps out; 'Average Count'
 ANGOFFSET
 shouldn't change; the value is 90 because the horizontal to the floor is set to 0 but has to be 90 degree from the vertical, which is assumed to be 0
 ANGSTART
 starting angle; shouldn't change
 
 
 '#define' here is just for compiler, to give a name to the constant variable before being compiled to know which constant variable is needed;
 will be substituted to the variable defined when compiled  
 */

int angleZero = ANGSTART;
int sum1 = 0;
int ave1 = 0;
int angle1[AVECNT] = {
  0};
int ptr1 = 0;
int ang1 = 0;
/*
sum1
 sum of the array (10 angle inputs)
 ave1
 the avg. angle to be used
 angle1[AVECNT]
 [AVECNT] is the length of the array
 the array for 10 angles saved to avg the 10 variables
 ptr1
 just for index number access
 ang1
 each angle input that will be stored into the array
 */

int sum2 = 0;
int ave2 = 0;
int angle2[AVECNT] = {
  0};
int ptr2 = 0;
int ang2 = 0;

/*
For the second accelerometer
 */

long time1 = 0;
long time2 = 0;
int diff = 0;

/* Assign a unique base ID for this sensor */
#define MUX         0x70  //Multiplexer Address

Adafruit_9DOF    dof = Adafruit_9DOF();
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
sensors_event_t accel, mag, gyro, temp;
sensors_vec_t   orientation;

int h1 = 0;
int lv = 0;
/*
hl
 height
 lv
 level
 */

/* Height in mm and level [amt. of row][amt. of column]*/
int height [143][2] = {  //Height in mm and level
  {
    0,0   }
  ,
  {
    8,0         }
  ,
  {
    15,0         }
  ,
  {
    23,0         }
  ,
  {
    31,0         }
  ,
  {
    40,0         }
  ,
  {
    48,0         }
  ,
  {
    57,0         }
  ,
  {
    67,0         }
  ,
  {
    76,0         }
  ,
  {
    86,0         }
  ,
  {
    96,0         }
  ,
  {
    107,0         }
  ,
  {
    117,0         }
  ,
  {
    128,0         }
  ,
  {
    139,0         }
  ,
  {
    151,1         }
  ,
  {
    162,1         }
  ,
  {
    174,1         }
  ,
  {
    186,1         }
  ,
  {
    199,1         }
  ,
  {
    211,1         }
  ,
  {
    224,1         }
  ,
  {
    237,1         }
  ,
  {
    250,1         }
  ,
  {
    264,1         }
  ,
  {
    277,1         }
  ,
  {
    291,1         }
  ,
  {
    305,1         }
  ,
  {
    320,1         }
  ,
  {
    334,1         }
  ,
  {
    349,1         }
  ,
  {
    364,1         }
  ,
  {
    379,1         }
  ,
  {
    394,1         }
  ,
  {
    410,1         }
  ,
  {
    425,1         }
  ,
  {
    441,2         }
  ,
  {
    457,2         }
  ,
  {
    473,2         }
  ,
  {
    489,2         }
  ,
  {
    506,2         }
  ,
  {
    522,2         }
  ,
  {
    539,2         }
  ,
  {
    556,2         }
  ,
  {
    572,2         }
  ,
  {
    589,2         }
  ,
  {
    607,2         }
  ,
  {
    624,2         }
  ,
  {
    641,2         }
  ,
  {
    659,2         }
  ,
  {
    676,2         }
  ,
  {
    694,2         }
  ,
  {
    711,2         }
  ,
  {
    729,2         }
  ,
  {
    747,3         }
  ,
  {
    765,3         }
  ,
  {
    783,3         }
  ,
  {
    801,3         }
  ,
  {
    819,3         }
  ,
  {
    837,3         }
  ,
  {
    855,3         }
  ,
  {
    874,3         }
  ,
  {
    892,3         }
  ,
  {
    910,3         }
  ,
  {
    928,3         }
  ,
  {
    947,3         }
  ,
  {
    965,3         }
  ,
  {
    983,3         }
  ,
  {
    1002,3         }
  ,
  {
    1020,3         }
  ,
  {
    1038,4         }
  ,
  {
    1056,4         }
  ,
  {
    1075,4         }
  ,
  {
    1093,4         }
  ,
  {
    1111,4         }
  ,
  {
    1129,4         }
  ,
  {
    1147,4         }
  ,
  {
    1165,4         }
  ,
  {
    1183,4         }
  ,
  {
    1201,4         }
  ,
  {
    1219,4         }
  ,
  {
    1236,4         }
  ,
  {
    1254,4         }
  ,
  {
    1271,4         }
  ,
  {
    1289,4         }
  ,
  {
    1306,4         }
  ,
  {
    1323,5         }
  ,
  {
    1341,5         }
  ,
  {
    1358,5         }
  ,
  {
    1374,5         }
  ,
  {
    1391,5         }
  ,
  {
    1408,5         }
  ,
  {
    1424,5         }
  ,
  {
    1441,5         }
  ,
  {
    1457,5         }
  ,
  {
    1473,5         }
  ,
  {
    1489,5         }
  ,
  {
    1505,5         }
  ,
  {
    1520,5         }
  ,
  {
    1536,5         }
  ,
  {
    1551,5         }
  ,
  {
    1566,5         }
  ,
  {
    1581,5         }
  ,
  {
    1596,5         }
  ,
  {
    1610,6         }
  ,
  {
    1625,6         }
  ,
  {
    1639,6         }
  ,
  {
    1653,6         }
  ,
  {
    1666,6         }
  ,
  {
    1680,6         }
  ,
  {
    1693,6         }
  ,
  {
    1706,6         }
  ,
  {
    1719,6         }
  ,
  {
    1731,6         }
  ,
  {
    1744,6         }
  ,
  {
    1756,6         }
  ,
  {
    1768,6         }
  ,
  {
    1779,6         }
  ,
  {
    1791,6         }
  ,
  {
    1802,6         }
  ,
  {
    1813,6         }
  ,
  {
    1823,6         }
  ,
  {
    1834,6         }
  ,
  {
    1844,6         }
  ,
  {
    1854,6         }
  ,
  {
    1863,6         }
  ,
  {
    1873,6         }
  ,
  {
    1882,6         }
  ,
  {
    1890,6         }
  ,
  {
    1899,7         }
  ,
  {
    1907,7         }
  ,
  {
    1915,7         }
  ,
  {
    1922,7         }
  ,
  {
    1930,7         }
  ,
  {
    1937,7         }
  ,
  {
    1943,7         }
  ,
  {
    1950,7         }
  ,
  {
    1956,7         }
  ,
  {
    1962,7         }
  ,
  {
    1967,7         }
  ,
  {
    1972,7         }
  ,
  {
    1977,7         }
};

//just for printing out
String h = "<h:";
String a = "a:";
String l = "s:";
String t = "t:";
String g = "g:";
char ending = '>';
char sep = '!';
String str = "<h:-1!s:-1!a:-1!t:-1!g:-1>";

void setup(void) 
{
  Serial.begin(9600);
  Wire.begin();

  mux(ARMIMU);
  if(!lsm.begin()) {
    Serial.print(F("LSM9DS0 1 not detected!")); //cannot find IMU
  } 
  else {  //avg angle of the Arm acc.
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
    for (int i = 0; i < AVECNT; i++) {  //loop to fill average
      ang1 = 0;
      lsm.getEvent(&accel, &mag, &gyro, &temp); 
      if (dof.fusionGetOrientation(&accel, &mag, &orientation)) {
        ang1 = ANGOFFSET + round(orientation.pitch);
      }
      angle1[i] = ang1;
      sum1 += ang1;
    }
    ave1 = sum1 / AVECNT;
    angleZero = ave1; //set height 0 to startup angle
  }
  mux(UPIMU);
  if(!lsm.begin()) {
    Serial.print(F("LSM9DS0 2 not detected!")); //cannot find IMU
  } 
  else {
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
    for (int i = 0; i < AVECNT; i++) {  //loop to fill average
      ang2 = 0;
      lsm.getEvent(&accel, &mag, &gyro, &temp); 
      if (dof.fusionGetOrientation(&accel, &mag, &orientation)) {
        //ang2 = ANGOFFSET + round(orientation.pitch);
        ang2 = round(orientation.pitch);
      }
      angle2[i] = ang2;
      sum2 += ang2;
    }
    ave2 = sum2 / AVECNT;

  }
  Serial.println(str);
  time1 = millis();
}

void loop(void) {
  int offset; 
  mux(ARMIMU);  //arm accelerometer
  Wire.beginTransmission (IMUADDRESS);
  if (Wire.endTransmission () == 0){   // Make sure device is still there
    lsm.getEvent(&accel, &mag, &gyro, &temp); 
    if (dof.fusionGetOrientation(&accel, &mag, &orientation)) {
      ang1 = ANGOFFSET + round(orientation.pitch);

      sum1 = sum1 - angle1[ptr1];
      angle1[ptr1++] = ang1;
      sum1 = sum1 + ang1;
      ave1 = sum1 / AVECNT;

      offset = ave1 - angleZero;
      if (offset < 0 | offset > 143){
        offset = 0;
      }
      h1 = height[offset][0];
      lv = height[offset][1];
      if (ptr1 == AVECNT) {
        ptr1 = 0;
      }
    }
  } 
  else {
    ave1 = 0;
  }

  mux(UPIMU);  //up vertical accelerometer
  Wire.beginTransmission (IMUADDRESS);
  if (Wire.endTransmission () == 0){   // Make sure device is still there
    lsm.getEvent(&accel, &mag, &gyro, &temp); 
    if (dof.fusionGetOrientation(&accel, &mag, &orientation)) {
      //ang2 = ANGOFFSET + round(orientation.pitch);  //round() rounds float to integer
      ang2 = round(orientation.pitch);

      sum2 = sum2 - angle2[ptr2];
      angle2[ptr2++] = ang2;
      sum2 = sum2 + ang2;
      ave2 = sum2 / AVECNT;

      if (ptr2 == AVECNT) {
        ptr2 = 0;
      }
    }
  } 
  else {
    ave2 = 0;
  }

  //print logic goes here
  if (ave1 == 0) { //in case we get no reading
    ave1 = -1;
    ang1 = -1;
    h1 = -1;
    lv = -1;
  }
  if (ave2 == 0) { //in case we get no reading
    ave2 = -1;
    ang2 = -1;
  }
  int yaw = round(orientation.heading);  

  str = h + h1 + sep + l + lv + sep +a + ave1 + sep + t + ave2 + sep + g + yaw + sep +ending;
  Serial.println(str);  //"ln" only for debug

  time2 = millis();
  diff = time2 - time1;
  time1 = time2;
  if (diff < 50) {
    delay(50 - diff);
  }
}

/********************************************************
 * When selecting a channel bit2 of the control register
 * must be set to a logic 1 to enable channel selection.
 * If bit2 is a logic zero then all channels will be disabled.
 * If 0xFF is the selected channel it will disable all channels.
 ********************************************************/
void mux(byte channel) {
  byte controlRegister = 0x04;  
  Wire.beginTransmission(MUX);
  if (Wire.endTransmission () == 0){   // Make sure device is still there
    controlRegister |= channel;
    Wire.beginTransmission(MUX);
    if (channel == 0xFF){
      Wire.write(0x00);
    } 
    else { //deselect all channels
      Wire.write(controlRegister);
    }     //set to selected channel
    Wire.endTransmission();
  }
}


