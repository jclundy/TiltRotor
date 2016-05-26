#include <Servo.h>

#define BLACK_ESC_PIN 9
#define BLACK_SERVO_PIN 10

#define GREEN_ESC_PIN 5
#define GREEN_SERVO_PIN 6

#define ESC_INIT 40
#define ESC_STOP 20
#define ESC_SPEED 60

#define SERVO_INIT 90
#define SERVO_MAX 120
#define SERVO_MIN 60

Servo blackEsc;
Servo blackServo;
Servo greenEsc;
Servo greenServo;
String incomingString;

void setup()
{
  blackEsc.attach(BLACK_ESC_PIN);
  greenEsc.attach(GREEN_ESC_PIN);
  blackServo.attach(BLACK_SERVO_PIN);
  greenServo.attach(GREEN_SERVO_PIN);
  
  blackServo.write(SERVO_INIT);
  greenServo.write(SERVO_INIT);
  delay(10);
  blackEsc.write(ESC_INIT);
  greenEsc.write(ESC_INIT);
  delay(100);
  Serial.begin(9600);
  delay(100);
  Serial.begin(9600);
}

void loop()
{
  // If there is incoming value
  if(Serial.available() > 0)
  {
    // read the value
    char ch = Serial.read();
    if (ch != 10){
      Serial.print("I have received: ");
      Serial.print(ch, DEC);
      Serial.print('\n');
      incomingString += ch;
    }
    else
    {
      Serial.println("I am printing the entire string");
      Serial.println(incomingString);
      int val = incomingString.toInt();
      Serial.println("Printing the value: ");
      Serial.println(val);
      if(val == 0)
      {
        blackEsc.write(ESC_STOP);
        greenEsc.write(ESC_STOP);
        Serial.println("motors stopped");
      }
      else  if(val == 1)
      {
        blackEsc.write(ESC_SPEED);
        greenEsc.write(ESC_SPEED);
        Serial.println("starting motor");
      }
      else if (val > SERVO_MIN && val < SERVO_MAX)
     {
       Serial.println("Value is between 60 and 120");
       Serial.println("Writing to Servos");
       blackServo.write(val);
       greenServo.write(val);
     }
     else
     {
       Serial.println("Value is NOT between 60 and 120");
       Serial.println("Error with the input");
     }
      incomingString = "";
    }
  }
}

