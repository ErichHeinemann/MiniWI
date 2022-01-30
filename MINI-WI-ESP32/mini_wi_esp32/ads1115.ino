// Some stuff to manage the calibration if a Jaystick is in use

void ads1115_calibration_pitchbend(){
  pitchBendAnalog = ads.readADC_SingleEnded( 2 );
  pbsLo_Thr = pitchBendAnalog * 0.96;  // 490 Low threshold for mod stick center
  pbsHi_Thr = pitchBendAnalog * 1.04;  // 510 High threshold for mod stick center
}

void ads1115_calibration_modulation(){
  modLevel = ads.readADC_SingleEnded( 3 ); // ADC03 on ADS1115  -2 to +996
  modsLo_Thr = modLevel * 0.96;  // Low threshold for mod stick center
  modsHi_Thr = modLevel * 1.04;  // High threshold for mod stick center
}



/*
void ads1115_read( int16_t & param_val0 , int16_t & param_val1 , int16_t & param_val2 , int16_t & param_val3  ){
  // store old value in a variable
  adc0_1 = adc0;
  adc1_1 = adc1;
  adc2_1 = adc2;
  adc3_1 = adc3;

  boolean do_update=false;
  // fetch new values with a slightly lowpass and better precision
  adc0 = ( adc0*2 + ads.readADC_SingleEnded(0)  ) / 3;
  // delay(1);
  adc1 = ( adc1*2 + ads.readADC_SingleEnded(1)  ) / 3;
  // delay(1);
  adc2 = ( adc2*2 + ads.readADC_SingleEnded(2)  ) / 3;
  // delay(1);
  adc3 = ( adc3*2 + ads.readADC_SingleEnded(3)  ) / 3;
  
  if( do_update == false && ( adc0 > adc0_1 + adc_slope || adc0<adc0_1 -adc_slope   ) ){
    do_update = true;    
  }
  if( do_update == false && ( adc1 > adc1_1 + adc_slope || adc1<adc1_1 - adc_slope  ) ){
    do_update = true;    
  }
  if( do_update == false && ( adc2 > adc2_1 + adc_slope || adc2<adc2_1 -adc_slope   ) ){
    do_update = true;    
  }
  if( do_update == false && ( adc3 > adc3_1 + adc_slope || adc3<adc3_1 -adc_slope   ) ){
    do_update = true;    
  }

  if( do_update == true ){
    param_val0 = map( adc0, 0, 17600, 0, 1023 );
    param_val1 = map( adc1, 0, 17600, 0, 1023 );
    param_val2 = map( adc2, 0, 17600, 0, 1023 );
    param_val3 = map( adc3, 0, 17600, 0, 1023 );
  }
  
}
*/
