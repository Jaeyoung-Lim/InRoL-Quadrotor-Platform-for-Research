

void ppm_command(){

  for(int i=0; i<4; i++){
    pulses[i] = cmd_val[i]; // Copy Pulse values
  }
   //Calculate Commands for PPM
  for(int i=4; i<8; i++){
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
