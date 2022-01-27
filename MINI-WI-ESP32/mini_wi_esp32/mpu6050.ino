


void mpu6050_setup(){

  // Try to initialize!
  if (!mpu.begin( 0x68 , &I2Ctwo )){
#ifdef DEBUGGING    
    Serial.println("Failed to find MPU6050 chip");
#endif    
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  
}

// calibrate to normal position eg zero 
void mpu6050_calibrate(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  /* Print out the values */
  calib_gyro_x = (int)round( g.gyro.x *100 ); // (int)round(gyro_z*127 )
  calib_gyro_y = (int)round( g.gyro.y *100 );
  calib_gyro_z = (int)round( g.gyro.z *100 );
#ifdef DEBUGGING
  Serial.println("\n UMP6050 Calibration Done");
#endif  
}

void mpu6050_read(){
  
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyro_x = (int)round( g.gyro.x  *100 ); // move float-vals into the areea of MIDI-CC-Values )
  gyro_y = (int)round( g.gyro.y  *100 );
  gyro_z = (int)round( g.gyro.z  *100 );
  gyro_x = (gyro_x + old_gyro_x) /2 ;
  gyro_y = (gyro_y + old_gyro_y) /2 ;
  gyro_z = (gyro_z + old_gyro_z) /2 ;

  if( old_gyro_x != gyro_x ){
    // send CC
    //display_changed = true;
    old_gyro_x = gyro_x;
  }
  if( old_gyro_y != gyro_y ){
    // send CC
    //display_changed = true;
    old_gyro_y = gyro_y;
    
  }
  if( old_gyro_z != gyro_z ){
    // send CC
    //display_changed = true;
    old_gyro_z = gyro_z;
  }

  gyro_total = (abs(calib_gyro_x - gyro_x) +  abs(calib_gyro_y - gyro_y) +  abs(calib_gyro_z - gyro_z) + gyro_total_old )/2; 

if( gyro_total_old != gyro_total ){
  display_changed = true;
  gyro_total_old = gyro_total;
  if( gyro_total > gyro_total_max ) gyro_total_max = gyro_total;
  gyro_total_midi = map( gyro_total, 0, gyro_total_max, 0,127 );
} 


 
#ifdef DEBUGGING
  /* Print out the values */
/*  
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
*/
  Serial.print("Gyro: ");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print(g.gyro.z);
  Serial.println("");
#endif  
}
