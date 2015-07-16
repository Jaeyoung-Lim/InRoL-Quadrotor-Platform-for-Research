void sw_read(){
    if(digitalRead(armsw_PIN)==HIGH){
      arm_stat = 1;
    }
    else {
     arm_stat =0;
    }
    if(digitalRead(mode_PIN)==HIGH){
      mode = mode_trim;
    }
    else {
      mode = mode_serial;
    }
}

void sw_readTrim(){
    trim_val[0]=analogRead(A0);
    trim_val[1]=analogRead(A1);
    trim_val[2]=analogRead(A2);
    trim_val[3]=analogRead(A3);
}

void sw_led(){
 if(arm_stat == 1){
  digitalWrite(led_PIN, LOW); // LED On if armed
  }
  if(count>25){ //Blink if not armed
    digitalWrite(led_PIN, LOW); // LED On
  }
  else {
    digitalWrite(led_PIN, HIGH); // LED off
  }
}

