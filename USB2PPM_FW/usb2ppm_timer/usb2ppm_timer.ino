#define timer_correction_factor 1.00                        //timer correction factor. This is needed if your arduino is too fast or slow, like mine. :(
#define timer_framelength 22000 * timer_correction_factor   //Maximum framelength in counter ticks
#define timer_pause 300 * timer_correction_factor           //Pause between pluses in counter ticks

int timer_accumulator = 0;         //accumulator. Used to calculate the frame padding
int timer_ptr = 0;                 //timer array pointer
int pulses[8];
int count=650;
int number_of_outputs =8;

void setup() {
  // put your setup code here, to run once:
  TCCR1A = B00110001; // Compare register B used in mode '3'
  TCCR1B = B00010010; // WGM13 and CS11 set to 1
  TCCR1C = B00000000; // All set to 0
  TIMSK1 = B00000010; // Interrupt on compare B
  TIFR1  = B00000010; // Interrupt on compare B
  OCR1A = timer_framelength;
  OCR1B = timer_pause;

  pinMode(10, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
for(int i=0; i<8; i++){
  pulses[i]=count;
}

count++;
if(count>1800){
  count =650;
}

delay(20);
}

ISR(TIMER1_COMPA_vect)
{
 if (timer_ptr == number_of_outputs) {
   timer_ptr = 0;  //reset the pointer to 0
   OCR1A = timer_framelength - (timer_accumulator * timer_correction_factor); //calculate the padding
   timer_accumulator = 0;  //set the accumulator to 0
 } 
 else {
   OCR1A = (pulses[timer_ptr] + timer_pause) * timer_correction_factor; //set the pulse length
   timer_accumulator += pulses[timer_ptr] + timer_pause; //add the pulse length to the accumulator
   timer_ptr++;  //increment the pointer
 } 
}
