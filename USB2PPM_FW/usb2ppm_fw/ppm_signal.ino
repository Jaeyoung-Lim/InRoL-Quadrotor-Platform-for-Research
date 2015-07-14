

void ppm_command(){
  //Values from Channel 1 to 4 is copied by the received command 
  for(int i=0; i<4; i++){
    pulses[i] = cmd_val[i]; // Copy Pulse values
  }
  //Values from Channel 7 is defined by arming switch
  pulses[4]=arm_stat;
  
  //Calculate Commands for PPM
  for(int i=5; i<8; i++){
    pulses[i]=800;
  }

}

void ppm_minmax(){
  for(int i=0; i<8; i++){
  if(pulses[i]>1800){
    pulses[i]=1800;
  }
  else if(pulses[i]<650){
    pulses[i]=650;
  }
}
}
