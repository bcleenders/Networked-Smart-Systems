/* Connect MMA8452Q to Arduino Uno: 
 * VCC_IN -> 5V 
 * GND -> GND 
 * SCL -> A5
 * SDA -> A4
 * GND -> SA0
 
 LED Layout:
       11
       10
 6  5  2  3  4
       8
       9
 
 */

#include <I2C.h>
#include "MMA845.h"

MMA845 mma;

int led_M = 2;
int led_R1 = 3;
int led_R2 = 4;
int led_L1 = 5;
int led_L2 = 6;
int led_D1 = 8;
int led_D2 = 9;
int led_U1 = 10;
int led_U2 = 11;

void setup()
{

  Serial.begin(115200);
  mma.begin();

  pinMode(led_M, OUTPUT);
  pinMode(led_R1, OUTPUT); 
  pinMode(led_R2, OUTPUT);
  pinMode(led_L1, OUTPUT);
  pinMode(led_L2, OUTPUT);
  pinMode(led_D1, OUTPUT);
  pinMode(led_D2, OUTPUT);
  pinMode(led_U1, OUTPUT);
  pinMode(led_U2, OUTPUT);
}

void loop()
{
  int x = 0, y = 0, z = 0;

  mma.getAccXYZ(&x, &y, &z); //get accelerometer readings in normal mode (hi res).
  //getAccXYZ(&x, &y, &z, false); //get accelerometer readings in fast mode (low res).
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);
  
  int t1 = 40;
  int t2 = 80;

  if(x < -t2) { digitalWrite(led_D2, HIGH); } else { digitalWrite(led_D2, LOW); }  
  if(x < -t1) { digitalWrite(led_D1, HIGH); } else { digitalWrite(led_D1, LOW); }
  if(x > t1)  { digitalWrite(led_U1, HIGH); } else { digitalWrite(led_U1, LOW); }
  if(x > t2) { digitalWrite(led_U2, HIGH); } else { digitalWrite(led_U2, LOW); }
  
  if(y < -t2) { digitalWrite(led_R2, HIGH); } else { digitalWrite(led_R2, LOW); }  
  if(y < -t1) { digitalWrite(led_R1, HIGH); } else { digitalWrite(led_R1, LOW); }
  if(y > t1)  { digitalWrite(led_L1, HIGH); } else { digitalWrite(led_L1, LOW); }
  if(y > t2) { digitalWrite(led_L2, HIGH); } else { digitalWrite(led_L2, LOW); }
  
  if((y > -60 && y < 60) || (x > -60 && x < 60)) { digitalWrite(led_M, HIGH); } else { digitalWrite(led_M, LOW); }


  delay(2);
}



