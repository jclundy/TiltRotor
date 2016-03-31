/*
*  This code is in the public domain.
*  (Do whatever you want with it.)
*/

// Need the Servo library
#include <Servo.h>

#define GREEN_PIN 9
#define GREEN_INIT 700

#define BLACK_PIN 10
#define BLACK_INIT 40
// This is our motor.
Servo greenMotor;
Servo blackMotor;
// This is the final output
// written to the motor.
String incomingString;


// Set everything up
int count = 0;
void setup()
{
  // Put the motor to Arduino pin #9
  greenMotor.attach(GREEN_PIN);
  blackMotor.attach(BLACK_PIN);
  delay(10);
  blackMotor.write(BLACK_INIT);
  delay(10);
  greenMotor.write(GREEN_INIT);
  delay(6000);
  // Required for I/O from Serial monitor
  Serial.begin(9600);
  // Print a startup message
   if(count % 2 == 0) {
    Serial.println("Write to Black Rotor");
  } else {
    Serial.println("Write to Green Rotor");
  }
}

void loop()
{
  // If there is incoming value
  if(Serial.available() > 0)
  {
    // read the value
    char ch = Serial.read();
  
    /*
    *  If ch isn't a newline
    *  (linefeed) character,
    *  we will add the character
    *  to the incomingString
    */
    if (ch != 10){
      // Print out the value received
      // so that we can see what is
      // happening
      Serial.print("I have received: ");
      Serial.print(ch, DEC);
      Serial.print('\n');
    
      // Add the character to
      // the incomingString
      incomingString += ch;
    }
    // received a newline (linefeed) character
    // this means we are done making a string
    else
    {
      // print the incoming string
      Serial.println("I am printing the entire string");
      Serial.println(incomingString);
    
      // Convert the string to an integer
      int val = incomingString.toInt();
    
      // print the integer
      Serial.println("Printing the value: ");
      Serial.println(val);
    
      /*
      *  We only want to write an integer between
      *  0 and 180 to the motor. 
      */
      if (val > 0 && val < 180)
     {
       // Print confirmation that the
       // value is between 0 and 180
       Serial.println("Value is between 0 and 180");
       // Write to Servo
       if(count % 2 == 0) {
        Serial.println("Writing to Black Rotor");
        blackMotor.write(val);        
      } else {
        Serial.println("Writing to Green Rotor");
        greenMotor.write(val);
      }
        count++;
        if(count % 2 == 0) {
          Serial.println("Write to Black Rotor");
        } else {
          Serial.println("Write to Green Rotor");
        }
     }
     // The value is not between 0 and 180.
     // We do not want write this value to
     // the motor.
     else
     {
       Serial.println("Value is NOT between 0 and 180");
      
       // IT'S a TRAP!
       Serial.println("Error with the input");
     }
    
      // Reset the value of the incomingString
      incomingString = "";
    }
  }
}

