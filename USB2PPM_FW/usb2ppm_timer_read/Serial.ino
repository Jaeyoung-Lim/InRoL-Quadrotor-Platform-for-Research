/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
 
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\0') {
      stringComplete = true;
    }
  }
}

void decodePacket(String cmd_String){
  
  cmd_String.toCharArray(cmd_Char, 26);
  int j=5;
    for(int i=0; i<26; i++){
    if(cmd_Char[i] =='x')    j=0;
    else if(cmd_Char[i] =='y')    j=1;
    else if(cmd_Char[i] =='z')    j=2;
    else if(cmd_Char[i] =='k')    j=3;
    else if((cmd_Char[i] >= 'a') && (cmd_Char[i] <= 'f') ){
      val[j]=16*val[j]+10+(cmd_Char[i] - 'a');
    }
    else if((cmd_Char[i] >= '0') && (cmd_Char[i] <= '9') ){
      val[j]=16*val[j]+(cmd_Char[i] -  '0');      //change the value into integer values for common
    }
    else if (cmd_Char[i] == '\0') {
   break;
  }
}
}


