// Need the Servo library
#include <Servo.h>

#define ESC_PIN 9
#define ESC_INIT 40
#define ESC_STOP 20
#define ESC_SPEED 60

#define SERVO_PIN 10
#define SERVO_INIT 90
#define SERVO_MAX 120
#define SERVO_MIN 60

Servo esc;
Servo servo;
String incomingString;

void setup()
{
  // Put the motor to Arduino pin #9
  esc.attach(ESC_PIN);
  servo.attach(SERVO_PIN);
  delay(10);
  servo.write(SERVO_INIT);
  delay(10);
  esc.write(ESC_INIT);
  delay(100);
  // Required for I/O from Serial monitor
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
        esc.write(ESC_STOP);
        Serial.println("motor stopped");
      }
      else  if(val == 1)
      {
        esc.write(ESC_SPEED);
        Serial.println("starting motor");
      }
      else if (val > SERVO_MIN && val < SERVO_MAX)
     {
       Serial.println("Value is between 60 and 120");
       Serial.println("Writing to Servo");
       servo.write(val);
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

