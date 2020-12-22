#include "Stepper.h"
#define STEPS  32 
#include <Servo.h>
#include <PID_v1.h>
Servo myservo;
Stepper small_stepper(STEPS, 8, 10, 9, 11);

const int redpin = 0;
const int bluepin = 1;

int red, red1;
int blue, blue2;

int i=90,j;

int offset=200;
int difference;

int serpos=90;
int steppos;

int buffmin = -50;
int buffmax = 50;

int maxpos, maximum, current;

double Setpoint, Input, Output;
double Kp=0, Ki=10, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(9600);
  small_stepper.setSpeed(500);
  red1 = analogRead(redpin);
  myservo.attach(6);
  myservo.write(90);
  Setpoint=0.00;
}


void loop()
{
  //Take a reading using analogRead() on sensor pin and store it in lightVal
  red = analogRead(redpin);
  blue = analogRead(bluepin);

  difference=red-blue;
  Input=analogRead(redpin);

  Serial.print(red);
  Serial.print(" ");
  Serial.print(blue);
  Serial.print(" ");
  Serial.println(difference);
  delay(100);

  if(difference>buffmax)
  {
    steppos=steppos+5;
    small_stepper.step(steppos);
    delay(10);
  }
  
  else if(difference<buffmin)
   {
     steppos=steppos-1;
     small_stepper.step(-steppos);
     delay(10);
   }

  
  if (difference>buffmin & difference<buffmax)
  {
    small_stepper.step(0);
    maximum=analogRead(redpin);

    for (i=90; i<180; ++i)
    {
      myservo.write(i);
      delay(10);
      current=analogRead(redpin);
      if (current > maximum)
      {
        maxpos=i;
        maximum=current;
      }

    }
  }



  //myservo.write(maxpos);
  //i=0;
//  Serial.println(maxpos);
}
