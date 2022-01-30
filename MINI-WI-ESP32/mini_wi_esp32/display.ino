// Funktions for the Display SSD1306
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Ctwo, -1 );

void display_setup(){
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display_calibration();

}

/*
char d_line0[40];
char d_line1[40];
char d_line2[40];
char d_line3[40];
char d_line4[40];
char d_line5[40];
*/

// 1 Note
// 2 Oktave
// Pitchbend als Balken?
// Breath-CC als Balken + CC-Number?
// Current Menu

void display_calibration(){
  display.clearDisplay();

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F( d_line0 ) );
  display.setTextSize(1); 
  display.setCursor(0,25);             // Start at top-left corner
  display.println(F( "Calibration" ) );

  /*display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setCursor(0,30);             // Start at top-left corner
  display.println(F( d_line2 ) );

  display.setCursor(0,40);             // Start at top-left corner
  display.println(F( d_line3 ) );
  */
 // display.setCursor(0,50);             // Start at top-left corner
 // display.println(F( d_line4 ) );

 // display.setCursor(0,60);             // Start at top-left corner
 // display.println(F( d_line5 ) );

/*
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);
*/
  display.display();
  delay(1);
}

// Normal Refresh while playing
void update_display(){
  display.clearDisplay();
  // int num = (pressureSensor);
  if( modus == 0 ){
    display.setTextSize( 1 );             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor( 0, 0 );             // Start at top-left corner
    display.println( d_line0 );
  
    display.setTextSize( 2 ); 
    display.setCursor( 0, 12 );    
    byte octave = fingeredNote / 12;
    byte noteInOctave = fingeredNote % 12;   
    display.setCursor( 20, 12 );    
    display.println( noteNames[noteInOctave] );
    
    display.setTextSize(1);
    display.print( "Oct/Note: ");
    display.setCursor( 2,13 );    
    display.print( octave + 1 );
  
    display.setCursor( 0, 40 );             // Start at top-left corner
    display.println( "PB" );
    display.setCursor( 15, 40 );             // Start at top-left corner
    display.println( touchPin_Value_midi );
       
    display.setCursor( 60, 20 );             // Start at top-left corner
    display.println( "Breath" );    
    display.setCursor( 102, 20 );             // Start at top-left corner
    display.println( ( pressureSensor - breath_Thr ) ); 
    // Draw a Rectangle in the range of breath_max, breath_Thr with the value of pressureSensor
    // 64 Pixels
    int i = (int) round( 64.0 / ( breath_max  - breath_Thr ) * ( pressureSensor - breath_Thr ));
    if(i <1 ) i=1;
    display.fillRoundRect( 64,0, i, 6,  2, SSD1306_WHITE);
    // is connected: isConnected
    if( isConnected==true ){
      display.setCursor( 60, 9 );
      display.println( "BLE conn" );
    }else{
      display.setCursor( 60, 9 );
      display.println( "no BLE !!" );
    }
    

    
    display.setCursor( 60, 30 );             // Start at top-left corner
    display.println( "Finger" );
    display.setCursor( 102, 30 );             // Start at top-left corner
    display.println( lastTouchValue );
    
    display.setCursor( 60, 40 ); 
    display.println( "Touch" );
    display.setCursor( 98, 40 ); 
    display.println( touchPin_Value );
  
    display.setCursor( 0, 50 ); 
    display.println( gyro_total );
  }
 if( modus == 1 ){
    display.setTextSize( 2 );             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor( 0, 0 );             // Start at top-left corner
    display.println( "Menu 1" );
 }
 if( modus == 2 ){

    display.setTextSize( 2 );             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor( 0, 0 );             // Start at top-left corner
    display.println( "Menu 2" );
 }
   
  display.display();
}  
