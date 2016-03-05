#include <Servo.h>

Servo esc1;
Servo servo1;

void setup()
{
  servo1.attach(3);
  esc1.attach(10);
  delay(10);
  esc1.write(700);
  delay(2000);
}

int servoValue = 70;
int escValue = 60;
int servoDiff = 0;
int escDiff = 0;

void loop()
{
  escDiff++;
  escDiff %= 30;
  servoDiff ++;
  servoDiff %= 10;
  esc1.write(escValue + escDiff);
  servo1.write(85 + servoDiff); 
  delay(10);
}
