#include <Servo.h>

Servo blackESC;
Servo greenESC;

#define GREEN_PIN 10
#define GREEN_INIT 40
#define GREEN_START_VAL 80

#define BLACK_PIN 9
#define BLACK_INIT 700
#define BLACK_START_VAL 40

void setup()
{
  //setup green rotor
  greenESC.attach(GREEN_PIN);
  delay(10);
  greenESC.write(GREEN_INIT);
  delay(3000);
  
  //setup black rotor
  blackESC.attach(BLACK_PIN);
  delay(10);
  blackESC.write(BLACK_INIT);
  delay(5000);
}

void loop()
{
  greenESC.write(80);
  blackESC.write(60);
}
