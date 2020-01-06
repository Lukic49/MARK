/*
  Connect left encoder on pin 18 & 29
  Connect right encoder on pin 19 & 27

  This code will send on uart0 the encoders count each time it changes.
  For reset the counter, send a data.

  Wiki for lib encoder : https://www.pjrc.com/teensy/td_libs_Encoder.html
*/

#include <Encoder.h>
Encoder knobLeft(18, 29);
Encoder knobRight(27, 19 );

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
}
