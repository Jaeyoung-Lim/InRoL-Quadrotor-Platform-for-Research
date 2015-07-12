
//Timer Initialization
#define timer_correction_factor 1.00                        //timer correction factor. This is needed if your arduino is too fast or slow, like mine. :(
#define timer_framelength 22000 * timer_correction_factor   //Maximum framelength in counter ticks
#define timer_pause 300 * timer_correction_factor           //Pause between pluses in counter ticks

#define ppmout_PIN 10

int timer_accumulator = 0;         //accumulator. Used to calculate the frame padding
int timer_ptr = 0;                 //timer array pointer

//String Variable initialization
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
char cmd_Char[26]="";
int val[4]={0, 0, 0, 0};

int pulses[8];
int count=650;
int number_of_outputs =8;

void setup() {
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  
  // put your setup code here, to run once:
  TCCR1A = B00110001; // Compare register B used in mode '3'
  TCCR1B = B00010010; // WGM13 and CS11 set to 1
  TCCR1C = B00000000; // All set to 0
  TIMSK1 = B00000010; // Interrupt on compare B
  TIFR1  = B00000010; // Interrupt on compare B
  OCR1A = timer_framelength;
  OCR1B = timer_pause;

  pinMode(ppmout_PIN, OUTPUT);
}

void loop() {
  //Read Commands
    serialEvent(); //call the function
  // print the string when a newline arrives:
  if (stringComplete) {
    Serial.print(inputString);
    //Decode and copy the packet
    decodePacket(inputString);
    
    for(int i=0; i<4; i++){
      pulses[i]=val[i];
    }
    
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

 
  //Calculate Commands for PPM
  for(int i=5; i<8; i++){
  pulses[i]=count;
}

count++;
if(count>1800){
  count =650;
}

delay(20);
}

