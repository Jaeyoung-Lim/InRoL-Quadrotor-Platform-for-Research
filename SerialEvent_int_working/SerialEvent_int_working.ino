/*
  Serial Event example

 When new serial data arrives, this sketch adds it to a String.
 When a newline is received, the loop prints the string and
 clears it.

 A good test for this is to try it with a GPS receiver
 that sends out NMEA 0183 sentences.

 Created 9 May 2011
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/SerialEvent

 */

int inputString[26];         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int p=0;
int cmd_val=0;
int inChar=0;

void setup() {
  // initialize serial:
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  //inputString.reserve(200);
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    p=0; 
    while(inputString[p]!=0){
      cmd_val=10*cmd_val+(inputString[p]-48);
      p++;
    }
    
     Serial.print(cmd_val, DEC);//Monitor values
    
    //Reititialize buffer
    for(int i=0; i<26; i++){
    inputString[i] = 0;
    }
    cmd_val=0;
    stringComplete = false;
    p=0;
  }
  delay(500);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    inChar = Serial.read();
    // add it to the inputString:
    inputString[p] += inChar;
    p++;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\0') {
      stringComplete = true;
    }
  }
}
