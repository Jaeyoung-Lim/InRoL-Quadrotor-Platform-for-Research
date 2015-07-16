void ppm_command(int mode){
  
  if(mode == mode_serial){ //Serial mode ppm generation
    for(int i=0; i<4; i++){ //Channel 1 to Channel 4 receives command values for control
      pulses[i] = map(cmd_val[i]+0.1*trim_val[i], 0, 1024, 650, 1800); // Copy Pulse values
    }
    }
  else if(mode == mode_trim ){
      //Channel 1 to Channel 4 receives command values only from trim levers
      pulses[0] = 1200 + trim_val[0];
      pulses[1] = 1200 + trim_val[1];
      pulses[2] = 650+trim_val[2];
      pulses[3] = 1200 + trim_val[3];
    }
     pulses[4] = 1000; // Channel 5
     pulses[5] = 1000; // Channel 6
  if(arm_stat==1){ //Values from Channel 7 is defined by arming switch
     pulses[6]=650;  
    }
    else {
     pulses[6]=1700;
    }
    pulses[7] = 1000; // Channel 8
}
    

void ppm_default(){
  for(int i=0; i<4; i++){
      cmd_val[i]=1000;
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
