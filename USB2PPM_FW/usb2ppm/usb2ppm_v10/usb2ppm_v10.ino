////////////////////////////////////////////////////////////////////////////////////////////
/*
This is a program written to run in Arduino Uno to interface with the trainer port of Turnigy 9X
Code provides 8 Channels using PPM, with resolution of 1024.

Written by Jaeyoung Lim
*/

int Ch1_Command=0;
  int Ch1_uS=0;
int Ch2_Command=0;
  int Ch2_uS=0;
int Ch3_Command=0;
  int Ch3_uS=0;
int Ch4_Command=0;
  int Ch4_uS=0;
int val=90;  
int Fixed_uS = 300;       // PPM frame fixed LOW phase
int pulseMin = 650;          // pulse minimum width minus start in uS

int outPinPPM = 6;


ISR(TIMER1_COMPA_vect) {
    ppmoutput(); // Jump to ppmoutput subroutine
}

void setup()
{
  Serial.begin(9600);
  pinMode(outPinPPM, OUTPUT);   // sets the digital pin as output
  
   // Setup timer
  TCCR1A = B00110001; // Compare register B used in mode '3'
  TCCR1B = B00010010; // WGM13 and CS11 set to 1
  TCCR1C = B00000000; // All set to 0
  TIMSK1 = B00000010; // Interrupt on compare B
  TIFR1  = B00000010; // Interrupt on compare B
  OCR1A = 22000; // 22mS PPM output refresh
  OCR1B = 1000;
}

void loop()
{
  
  Ch1_Command = val++;
  if(val>950){
    val=90;
  }
  Ch2_Command = 400;
  Ch3_Command = 600;
  Ch4_Command = 800;
  
  
 
   Ch1_uS = (map(Ch1_Command, 90, 950, 0,1023) * 1) + pulseMin;
   Ch2_uS = (map(Ch2_Command, 90, 950, 0,1023) * 1) + pulseMin;
   Ch3_uS = (map(Ch3_Command, 90, 950, 0,1023) * 1) + pulseMin;
   Ch4_uS = (map(Ch4_Command, 90, 950, 0,1023) * 1) + pulseMin;

   
  if (Ch1_uS <= 750) Ch1_uS = 750;     // Min
  if (Ch1_uS >= 1700) Ch1_uS = 1700;   // Max   
  if (Ch2_uS <= 750) Ch2_uS = 750;     // Min
  if (Ch2_uS >= 1700) Ch2_uS = 1700;   // Max   

  if (Ch3_uS <= 750) Ch3_uS = 750;     // Min
  if (Ch3_uS >= 1700) Ch3_uS = 1700;   // Max   

  if (Ch4_uS <= 750) Ch4_uS = 750;     // Min
  if (Ch4_uS >= 1700) Ch4_uS = 1700;   // Max   

 Serial.print("Ch1:");
  Serial.print("Ch2:");
   Serial.print("Ch3:");
    Serial.print("Ch4:");
  
 delay(10);
}

void ppmoutput() { // PPM output sub
  // Channel 1 - Aeleron
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Ch1_uS);  // Hold for Aeleron_uS microseconds      

 // Channel 2 - Elevator 
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Ch2_uS); // Hold for Elevator_uS microseconds      

 // Channel 3 - Throttle
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Ch3_uS); // Hold for Throttle_uS microseconds      

 // Channel 4 - Rudder
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Ch4_uS);   // Hold for Rudder_uS microseconds

 // Channel 5
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Ch1_uS);

 // Channel 6
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Ch2_uS);

 // Channel 7
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Ch3_uS);

 // Channel 8
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);
  delayMicroseconds(Ch4_uS);

 // Synchro pulse
  digitalWrite(outPinPPM, LOW);
  delayMicroseconds(Fixed_uS);    // Hold
  digitalWrite(outPinPPM, HIGH);  // Start Synchro pulse

}

