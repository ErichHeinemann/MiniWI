#include <Arduino.h>

// #define MIDI5Pin

// ESP32_BLE_MIDI https://github.com/max22-/ESP32-BLE-MIDI
#define ESP32_BLE_MIDI

#ifdef ESP32_BLE_MIDI
#include <BLEMidi.h>
#endif

#define PitchbendTouch
// #define PitchbendAnalog

#define ModulationGyro
// #define ModulationAnalog

/*
 * 
 * // https://github.com/lathoub/Arduino-BLE-MIDI
#include <BLEMIDI_Transport.h>

// #include <hardware/BLEMIDI_ESP32_NimBLE.h>
#include <hardware/BLEMIDI_ESP32.h>
// #include <hardware/BLEMIDI_nRF52.h>
// #include <hardware/BLEMIDI_ArduinoBLE.h>

// BLEMIDI_CREATE_DEFAULT_INSTANCE()
BLEMIDI_CREATE_INSTANCE( "MIDI MINI WI", MIDI )
// 9c:9c:1f:e2:19:d2
*/

unsigned long t0 = millis();
bool isConnected = false;

uint8_t modus = 0; // 0 = Play-Mode 1= Menu

// GPIO 14, Touch6 as extra Touchpin used directly on ESP32
uint8_t touchpin = 14; // touchpin_val= touchRead( touchpin ); 
uint16_t touchPin_Value=0; // Value of readed touchpin
uint16_t touchPin_Value_Max = 70; // has to be evaluated
uint16_t touchPin_last_Value=0;
uint8_t touchPin_Value_midi=0;
uint8_t velocity_min = 50;
// #include "FS.h"
// #include <LITTLEFS.h>

#include <WiFi.h>

// #include <MIDI.h>
#define NORM127MUL  0.007874f

// #define MIDIRX_PIN 16
// #define MIDITX_PIN 17

  uint16_t touchValue = 0;
  uint16_t lastTouchValue = 0;

// struct Serial2MIDISettings : public midi::DefaultSettings{
//   static const long BaudRate = 31250;
//   static const int8_t RxPin  = MIDIRX_PIN;
//   static const int8_t TxPin  = MIDITX_PIN; // LED-Pin
// };

// HardwareSerial MIDISerial(2);
// MIDI_CREATE_CUSTOM_INSTANCE( HardwareSerial, MIDISerial, MIDI, Serial2MIDISettings );


//I2C
#include <SPI.h>
#include <Wire.h>

// Display
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// MPR121
#include <Adafruit_MPR121.h>

// ADS1115
#include <Adafruit_ADS1X15.h>

// Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
#define ADC_ADDRESS 0x48


// Lines for the SD1306 Display
char *d_line0="Mini WI"; // Titel
char *d_line1=" ";  // Mode
char *d_line2=" ";        // Param 1     
char *d_line3=" ";        // Param 2  
char *d_line4=" ";        // Note  
char *d_line5=" ";        // Breath  
   
byte octave = 0;
byte noteInOctave = 1;

// Accelration-Sensor / Tiltsensor for additional CC-Messages !!
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;
int gyro_x;
int gyro_y;
int gyro_z;
uint16_t gyro_total =0; // Summ of all movements forward or backwards
uint16_t gyro_total_old =0;
uint16_t gyro_total_max = 127; // The maximum Value of chaking the instrument detected by the Gyro-Sensor UMP6050
byte gyro_total_midi=0;
int calib_gyro_x;
int calib_gyro_y;
int calib_gyro_z;

int old_gyro_x;
int old_gyro_y;
int old_gyro_z;

bool display_changed = false; // if this is true, the display will get a refresh

// #define DEBUGGING  1

uint16_t prescaler_gyro_counter =0;
uint16_t prescaler_gyro_th = 60;

uint16_t prescaler_display_counter =0;
uint16_t prescaler_display_th = 200;

char * noteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "Bb", "H"};
 
// Source:
// https://hackaday.io/project/11843-miniwi-woodwind-midi-controller

// Gyro and Acceleration-Sensor?? MPU-6050 / GY571 ??

/*
NAME:                 MiniWI Cap Touch and Portamento ver.
WRITTEN BY:           JOHAN BERGLUND
CREDITS:              State machine from the Gordophone blog by GORDON GOOD
DATE:                 2016-06-01
FILE SAVED AS:        MiniWI-cap-pmt.ino
FOR:                  Arduino Pro Mini, ATmega328, version with breakouts for A6 and A7
CLOCK:                16.00 MHz CRYSTAL                                        
PROGRAMME FUNCTION:   Wind Controller with EWI style key setup (reduced) with optional Casio DH addition, 
                      Freescale MPX5010GP breath sensor, PS2 style thumb joysticks 
                      for octave selection and pb/mod control, capacitive touch keys, output to 5-pin DIN MIDI 

HARDWARE NOTES:
* For the MIDI connection, attach a MIDI out Female 180 Degree 5-Pin DIN socket to Arduino.
* Socket is seen from solder tags at rear.
* DIN-5 pinout is:                                         _______ 
*    pin 2 - GND                                          /       \
*    pin 4 - 220 ohm resistor to +5V                     | 1     3 |  MIDI jack
*    pin 5 - Arduino Pin 1 (TX) via a 220 ohm resistor   |  4   5  |
*    all other pins - unconnected                         \___2___/
*
* Left hand thumb joystick controls octaves.
* Up/down axis is connected to Arduino pin A6.
* 
*       ^
*     < o >
*       v
*      -1
* 
* A potentiometer connected to Arduino pin A7 sets the base octave -2 to +2 from startnote octave.
*
* Right hand thumb joystick controls pitch bend and modulation.
* Pitch bend and modulation are connected to Arduino pins A0 and A1,
* on DIP rows.
* 
*     PB up
*       ^
* Mod < o > Mod
*       v
*     PB dn
*     
* A potentiometer controls portamento speed setting.
* It is connected to Arduino pin A2.  
*     
* The Freescale MPX5010GP pressure sensor output (V OUT) is connected to Arduino pin A3.
* 
* 1: V OUT (pin with indent)
* 2: GND
* 3: VCC (to 5V)    
* 4: n/c
* 5: n/c
* 6: n/c
*     
* The cheapere Freescale MPX53DP pressure is not thee best option, because it uses 2 Pins on the ADS1115 
* 
* 1: GND (pin with indent)
* 2: +Vout to ADC0
* 
* * 3: +VSS  to 5Volts
* 4: -Vout to ADC
* 5: n/c
* 6: n/c
* 
* Midi panic on pin 11 and 12 (internal pullup, both pins low sends all notes off)
* 
*/

//_______________________________________________________________________________________________ DECLARATIONS

uint16_t breath_Thr = 120;     // Set threshold level before switching ON
#define  ON_Delay   15   // Set Delay after ON threshold before velocity is checked (wait for tounging peak)
uint16_t breath_max=140;  // Blowing as hard as you can // 5 Volt to the MPX DP53DP

#define modsLo_Thr 50  // Low threshold for mod stick center
#define modsHi_Thr 70  // High threshold for mod stick center

#define octsLo_Thr 50  // Low threshold for octave stick center
#define octsHi_Thr 70  // High threshold for octave stick center

/*
#define octsLo1_Thr 409  // Low threshold for octave select pot
#define octsHi1_Thr 613  // High threshold for octave select pot
#define octsLo2_Thr 205  // Low threshold 2 for octave select pot
#define octsHi2_Thr 818  // High threshold 2 for octave select pot
*/

#define PB_sens 4095    // Pitch Bend sensitivity 0 to 8191 where 8191 is full pb range

// The three states of our state machine

// No note is sounding
#define NOTE_OFF 1

// We've observed a transition from below to above the
// threshold value. We wait a while to see how fast the
// breath velocity is increasing
#define RISE_WAIT 2

// A note is sounding
#define NOTE_ON 3

// Send CC data no more than every CC_INTERVAL
// milliseconds
#define CC_INTERVAL 50 

//variables setup

int state;                         // The state of the state machine
unsigned long ccSendTime = 0L;     // The last time we sent CC values
unsigned long breath_on_time = 0L; // Time when breath sensor value went over the ON threshold
int16_t initial_breath_value;          // The breath value at the time we observed the transition

int16_t xOctaves;
int16_t yOctaves;

int potPinModulation = 34; // GPIO-PIn on ESP32 for Modulation Values from 0 to 4095!
  
long lastDebounceTime = 0;         // The last time the fingering was changed
long debounceDelay = 30;           // The debounce time; increase if the output flickers
int lastFingering = 0;             // Keep the last fingering value for debouncing

uint8_t MIDIchannel=0;                // MIDI channel 1

int modLevel;
int oldmod=0;

int16_t pitchBendAnalog;
int oldpb=8192;

int16_t portLevel;
int oldport=-1;

int16_t breathLevel=0;   // breath level (smoothed) not mapped to CC value

int16_t pressureSensor;  // pressure data from breath sensor, for midi breath cc and breath threshold checks
byte velocity;       // remapped midi velocity from breath sensor

int fingeredNote;    // note calculated from fingering (switches) and octave joystick position
byte activeNote;     // note playing
byte startNote=74;   // set startNote to D (change this value in steps of 12 to start in other octaves)

byte midistatus=0;
byte x;
byte LedPin = 2;    // select the pin for the LED, 2 on ESP32, 13 on Arduino

Adafruit_MPR121 touchSensor = Adafruit_MPR121(); // This is the 12-input touch sensor

     // Key variables, TRUE (1) for pressed, FALSE (0) for not pressed
byte LH1;   // Left Hand key 1 (pitch change -2) 
byte LHb;   // Daumen auf der Rückseite wie bei Blöckflöte
            // Left Hand bis key (pitch change -1 unless both LH1 and LH2 are pressed)
byte LH2;   // Left Hand key 2  (with LH1 also pressed pitch change is -2, otherwise -1)
byte LH3;   // Left Hand key 3 (pitch change -2)

byte LHp1;  // Left Hand pinky key 1 (pitch change -1)
byte LHp2;  // Left Hand pinky key 2 (pitch change +1)

byte RH1;   // Right Hand key 1 (with LH3 also pressed pitch change is -2, otherwise -1)
byte RH2;   // Right Hand key 2 (pitch change -1)
byte RH3;   // Right Hand key 3 (pitch change -2)
byte RHp1=0;// Right Hand pinky key 1 (pitch change -1) 
byte RHp2;  // Right Hand pinky key 2 (pitch change -1)

byte RH6;  // Right Hand Thumb or pinky key 3, Pitch - 12 to get a lower register

byte OCTup=0; // Octave switch key (pitch change +12) --- Not used in this version 
byte OCTdn=0; // Octave switch key (pitch change -12) --- Not used in this version

byte PortK;   // Portamento momentary on switch
byte oldportk;

int joyOct; // Octave shifting by joystick or potentiometer


// ADS1115 - Integration
// #include <Adafruit_ADS1X15.h>
// Adafruit_ADS1115 ads;  // Use this for the 16-bit version

// remove resistors 70 and 68  on AI Thinker Audio 2.2 - Board to free up Pin 5 and Pin 23!

#define SDA2 5
#define SCL2 23

TwoWire I2Ctwo = TwoWire(0);

#define ADC_ADDRESS 0x48

//  4 Analog-Inputs:
//   A0 = breath - Differential
//   A1 = breath - Differential
//   A2 = Joystick Y
//   A3 = Joystick X

int16_t adc0, adc1, adc2, adc3;
int16_t adc0_1, adc1_1, adc2_1, adc3_1;

uint16_t ads_prescaler = 0; 
uint16_t patch_edit_prescaler = 0; // Wait seconds after loading to sync the values

void onNoteOnESP32( uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp ){
#ifdef DEBUGGING
   Serial.printf("Received note on : channel %d, note %d, velocity %d (timestamp %dms)\n", channel, note, velocity, timestamp);
#endif
}

void onNoteOffESP32(uint8_t channel, uint8_t note, uint8_t velocity, uint16_t timestamp ){ 
#ifdef DEBUGGING   
   Serial.printf("Received note off : channel %d, note %d, velocity %d (timestamp %dms)\n", channel, note, velocity, timestamp);  
#endif
}

void onControlChangeESP32(uint8_t channel, uint8_t controller, uint8_t value, uint16_t timestamp){
#ifdef DEBUGGING
  Serial.printf("Received control change : channel %d, controller %d, value %d (timestamp %dms)\n", channel, controller, value, timestamp);
#endif  
}
void onConnectedESP32( ){
  isConnected = true;
#ifdef DEBUGGING
  Serial.println("Connected"); // (timestamp %dms)\n", timestamp);
#endif
}
void onDisconnectedESP32(  ){
  isConnected = false;
#ifdef DEBUGGING
  Serial.println("Disconnected");
#endif
}



//_______________________________________________________________________________________________ SETUP

void setup() {

#ifdef MIDI5Pin
//   Serial.begin(31250);  // start serial with midi baudrate 31250
#else
  Serial.begin(115200);  // start serial with midi baudrate 31250
  Serial.flush();
#endif
  // pinMode( MIDIRX_PIN , INPUT_PULLUP);  // GPIO16 
  // MIDISerial.begin( 31250, SERIAL_8N1, MIDIRX_PIN, MIDITX_PIN ); // midi port
  // MIDI_setup();

#if 0
    setup_wifi();
#else
//    WiFi.mode(WIFI_OFF);
#endif

  pinMode( LedPin, OUTPUT );   // declare the LED's pin as output

#ifdef DEBUGGING  
  Serial.printf("BLE-Setup\n");
  Serial.println("Initializing bluetooth");
#endif 
#ifdef ESP32_BLE_MIDI
  // BLE-MIDI
  BLEMidiServer.begin("MIDI MINI WI");
  BLEMidiServer.setNoteOnCallback( onNoteOnESP32 );
  BLEMidiServer.setNoteOffCallback( onNoteOffESP32 );
  BLEMidiServer.setControlChangeCallback( onControlChangeESP32 );
  BLEMidiServer.setOnConnectCallback( onConnectedESP32 );
  BLEMidiServer.setOnDisconnectCallback( onDisconnectedESP32 );  
  // BLEMidiServer.enableDebugging();
#endif  

#ifdef BLE_MIDI
  MIDI.begin();  
  BLEMIDI.setHandleConnected([](){
    Serial.println("Connected");
    isConnected = true;
    digitalWrite( LedPin, HIGH );
  });

  BLEMIDI.setHandleDisconnected([]() {
    Serial.println("Disconnected");
    isConnected = false;
    digitalWrite(LedPin, LOW);
  });

  MIDI.setHandleNoteOn([](byte channel, byte note, byte velocity) {
    digitalWrite(LedPin, LOW);
  });
  MIDI.setHandleNoteOff([](byte channel, byte note, byte velocity) {
    digitalWrite(LedPin, HIGH);
  });
#endif    

  state = NOTE_OFF;  // initialize state machine

#ifdef DEBUGGING
  Serial.printf("PinMode-Setup\n");
#endif

  // void Core0TaskSetup(){
  // I2C
  Serial.printf("I2C-Setup\n");
  
  pinMode( SDA2, INPUT_PULLUP ); 
  pinMode( SCL2, INPUT_PULLUP ); 
  I2Ctwo.begin( SDA2, SCL2, 50000 ); // SDA2 pin 0, SCL2 pin 5, works with 50.000 Hz if pinned to other Core0

  // Set up touch sensor
  // begin(uint8_t i2caddr = MPR121_I2CADDR_DEFAULT, TwoWire *theWire = &Wire,
  //           uint8_t touchThreshold = MPR121_TOUCH_THRESHOLD_DEFAULT,
  //           uint8_t releaseThreshold = MPR121_RELEASE_THRESHOLD_DEFAULT);
  Serial.printf("MPR121-Setup\n");
  if (!touchSensor.begin( 0x5A, &I2Ctwo )) {
    while (1);  // Touch sensor initialization failed - stop doing stuff
    Serial.printf("MPR121-Setup failed\n");
  }

  mpu6050_setup();
  
  display_setup();
  Serial.printf("LED-Blink\n");
  for (x=1; x<=4; x++){  // Do the flashy-flashy to say we are up and running
    digitalWrite( LedPin, HIGH );
    delay(300);
    digitalWrite( LedPin, LOW );
    delay(300);
  }

#ifdef DEBUGGING
  Serial.printf("ADS1115 Setup\n");
#endif
  // ADS1115

#ifdef DEBUGGING
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");
#endif
  ads.begin( ADC_ADDRESS,  &I2Ctwo   );
#ifdef DEBUGGING
  Serial.println("ADS1115 started");

  Serial.printf("Pressure Calibration\n");
#endif  
  pressure_calibration();
  
}

//
// calibration of Pressure Sensor while "no" pressure
//
void pressure_calibration(){
  ads.setGain( GAIN_SIXTEEN ); // GAIN_FOUR (for an input range of +/-1.024V) / GAIN_EIGHT (for an input range of +/-0.512V)
 // Start Calibration
  breath_Thr = ads.readADC_Differential_0_1();
    digitalWrite( LedPin, HIGH );
    delay(300);
    digitalWrite( LedPin, LOW );
    delay(300);
  breath_Thr += ads.readADC_Differential_0_1();
    digitalWrite( LedPin, HIGH );
    delay(300);
    digitalWrite( LedPin, LOW );
    delay(300);
  breath_Thr += ads.readADC_Differential_0_1();
    digitalWrite( LedPin, HIGH );
    delay(300);
    digitalWrite( LedPin, LOW );
    delay(300);
  breath_Thr += ads.readADC_Differential_0_1();
  breath_Thr = ( breath_Thr / 4 ) + 3; // +2 or +3 check if there are too many Sounds which don´t stop
  // Display the result of the calibration
}
//_______________________________________________________________________________________________ MAIN LOOP

void loop() {

#ifdef BLE_MIDI
   MIDI.read();
#endif


prescaler_gyro_counter +=1;
if( prescaler_gyro_counter > prescaler_gyro_th ){
  mpu6050_read();  
  prescaler_gyro_counter = 0;
}

prescaler_display_counter +=1;
if( display_changed== true || prescaler_display_counter > prescaler_display_th ){
  update_display();  
  prescaler_display_counter =0;
  display_changed = false;
}

// if(BLEMidiServer.isConnected()) { 

  // if both joystick buttons are pressed, send all notes off
  //if((digitalRead(11) == 0) && (digitalRead(12) == 0)){
  //  midiPanic();
  //}

  // read all analog signals from ADS1115
  // pressureSensor = analogRead(A3); // Get the pressure sensor reading from analog pin A3
  // pressureSensor = ads.readADC_SingleEnded(0);
  
  pressureSensor = ads.readADC_Differential_0_1();
  
   // Serial.print( "Breath: " );   
   // Serial.print( pressureSensor );
   // Serial.print( " > " );   
   // Serial.println(   ON_Thr );

  if( state == NOTE_OFF ){
    if( pressureSensor > breath_Thr ){
      // Value has risen above threshold. Move to the ON_Delay
      // state. Record time and initial breath value.
#ifdef DEBUGGING
      Serial.print( pressureSensor );
      Serial.print( " > " );   
      Serial.println(   breath_Thr );
#endif      
      breath_on_time = millis();
      initial_breath_value = pressureSensor;
      state = RISE_WAIT;  // Go to next state
      if( pressureSensor > breath_max ){ breath_max = pressureSensor ;}
      display_changed= true;
    }
  }else if( state == RISE_WAIT ){
    if( pressureSensor > breath_Thr ){
      // Has enough time passed for us to collect our second
      // sample?
      if( millis() - breath_on_time > ON_Delay ){
        if( pressureSensor > breath_max ){ breath_max = pressureSensor ;}
#ifdef DEBUGGING
        Serial.print( pressureSensor );
        Serial.print( " > " );   
        Serial.println(   breath_Thr );
#endif
        // Yes, so calculate MIDI note and velocity, then send a note on event
        readSwitches();
        //readOctaves();
        oldportk=2; // Set oldportk to a value other than 1 or 0 to make sure it always sends the data for new notes
        //portamento();
        // We should be at tonguing peak, so set velocity based on current pressureSensor value        
        // If initial value is greater than value after delay, go with initial value, constrain input to keep mapped output within 7 to 127
        velocity = map( constrain( max(pressureSensor,initial_breath_value), breath_Thr, breath_max ), breath_Thr, breath_max, velocity_min, 127 );
        breathLevel=constrain( max(pressureSensor, initial_breath_value), breath_Thr, breath_max );
 #ifdef MIDI5Pin       
        midiSend((0x90 | MIDIchannel), fingeredNote, velocity); // send Note On message for new note
#endif
#ifdef ESP32_BLE_MIDI
        BLEMidiServer.noteOn(  MIDIchannel, fingeredNote, velocity  );
#endif
#ifdef BLE_MIDI
        MIDI.sendNoteOn( fingeredNote, velocity,  MIDIchannel );
#endif        
        activeNote=fingeredNote;
        state = NOTE_ON;
        display_changed= true;        
      }
    }else{
      // Value fell below threshold before ON_Delay passed. Return to
      // NOTE_OFF state (e.g. we're ignoring a short blip of breath)
      state = NOTE_OFF;
      display_changed= true;   
    }
  }else if( state == NOTE_ON ){
    if( pressureSensor < breath_Thr){
      // Value has fallen below threshold - turn the note off
#ifdef MIDI5pin
      midiSend((0x80 | MIDIchannel), activeNote, 0); //  send Note Off message 
#endif
#ifdef ESP32_BLE_MIDI
      BLEMidiServer.noteOff(  MIDIchannel, activeNote, 0  );
#endif
#ifdef BLE_MIDI
      MIDI.sendNoteOff(  activeNote, 0,  MIDIchannel );
#endif      
      breathLevel=0;
      state = NOTE_OFF;
      display_changed= true;        
    }else{
      // Is it time to send more CC data?
      if( millis() - ccSendTime > CC_INTERVAL ){
         // deal with Breath, Pitch Bend and Modulation
         breath();
         // 2022 Modulation und Pitchbend deaktiviert
         pitch_bend();
         modulation();
         ccSendTime = millis();
      }
      readSwitches();
      // readOctaves();
      if( fingeredNote != lastFingering ){ //
        // reset the debouncing timer
        lastDebounceTime = millis();
      }
      if(( millis() - lastDebounceTime) > debounceDelay ){
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state
        if( fingeredNote != activeNote ){
          // Player has moved to a new fingering while still blowing.
          // Send a note off for the current note and a note on for the new note.
          // portamento();        
          velocity = map(constrain( pressureSensor, breath_Thr, breath_max ), breath_Thr, breath_max, velocity_min, 127 ); // set new velocity value based on current pressure sensor level
#ifdef MIDI5Pin
          midiSend(( 0x90 | MIDIchannel), fingeredNote, velocity ); // send Note On message for new note
#endif          
#ifdef ESP32_BLE_MIDI
          BLEMidiServer.noteOn(  MIDIchannel, fingeredNote, velocity  );
#endif
#ifdef BLE_MIDI
          MIDI.sendNoteOn(  fingeredNote, velocity,  MIDIchannel ); 
#endif

#ifdef MIDI5Pin
          midiSend(( 0x80 | MIDIchannel), activeNote, 0); // send Note Off message for previous note (legato)
#endif          
#ifdef ESP32_BLE_MIDI
          BLEMidiServer.noteOff(  MIDIchannel, activeNote, 0  );
#endif
#ifdef BLE_MIDI
          MIDI.sendNoteOff( activeNote, 0,  MIDIchannel );          
#endif
          activeNote=fingeredNote;
        }
      }
    }
    display_changed= true;  
  }
  lastFingering=fingeredNote; 
}
//_______________________________________________________________________________________________ FUNCTIONS

//  Send a three byte midi message  
void midiSend(byte midistatus, byte data1, byte data2) {
    digitalWrite(LedPin,HIGH);  // indicate we're sending MIDI data
#ifdef DEBUGGING
    Serial.print("Velocity: ");
    Serial.println( data2 );
#endif    
    Serial.write(midistatus);
    Serial.write(data1);
    Serial.write(data2);
    digitalWrite(LedPin,LOW);  // indicate we're sending MIDI data
}

//**************************************************************

void midiPanic(){
  for( int i = 0; i < 128; i++ ){
#ifdef MIDI5Pin    
    midiSend((0x80 | MIDIchannel), i, 0);
#endif    
#ifdef ESP32_BLE_MIDI
    BLEMidiServer.noteOff(  MIDIchannel, activeNote, velocity  );
#endif
#ifdef BLE_MIDI
    MIDI.sendNoteOff(  i, velocity, MIDIchannel );
#endif    
  }
}

//**************************************************************

void pitch_bend(){
  uint8_t pitchLSB;
  uint8_t pitchMSB;
#ifdef PitchbendTouch
  pitchBendAnalog = touchPin_Value_midi; // only positive eg Pitch-Up
  if( pitchBendAnalog > 5 ){
    pitchBendAnalog = pitchBendAnalog * 128;
    pitchBendAnalog = map( pitchBendAnalog,0,127,8192,16383); // go from 8192 to 16383 (full pb up) when off center threshold going up
  }else{
    pitchBendAnalog = 8192;
  }
#endif
#ifdef PitchbendAnalog
  // pitchBendAnalog = ads.readADC_SingleEnded(1) / 64;
  // pitchBendAnalog = analogRead(A0); // read voltage on analog pin A0
  if( pitchBendAnalog > modsHi_Thr ){
    pitchBendAnalog = map(pitchBendAnalog,modsHi_Thr,1023,8192,(8192 + PB_sens)); // go from 8192 to 16383 (full pb up) when off center threshold going up
  }else if( pitchBendAnalog < modsLo_Thr ){
    pitchBendAnalog = map( pitchBendAnalog,0,modsLo_Thr,(8191 - PB_sens),8192 ); // go from 8192 to 0 (full pb dn) when off center threshold going down
  }else{
    pitchBendAnalog = 8192; // 8192 is 0 pitch bend
  }
#endif  
  if( pitchBendAnalog != oldpb ){// only send midi data if pitch bend has changed from previous value
    pitchLSB = pitchBendAnalog & 0x007F;
    pitchMSB = (pitchBendAnalog >>7) & 0x007F; 
#ifdef MIDI5Pin    
    midiSend((0xE0 | MIDIchannel), pitchLSB, pitchMSB);
#endif    
#ifdef ESP32_BLE_MIDI
    BLEMidiServer.pitchBend(  MIDIchannel, (uint16_t) pitchBendAnalog  );
#endif
#ifdef BLE_MIDI
    MIDI.sendPitchBend(  pitchBendAnalog, MIDIchannel );
#endif    
    oldpb=pitchBendAnalog;
  }
}

//***********************************************************

void modulation(){
  // Analog-PIN from ESP32 GPIO 34
  // ggf in Kombination mit einem Touch-Sensor?? touchRead(4)
#ifdef ModulationGyro  
  modLevel = gyro_total_midi;
 if( modLevel > modsHi_Thr/8 ){
    modLevel = map( modLevel,modsHi_Thr,1023,0,127); // go from 0 to full modulation when off center threshold going right(?)
  }else{
    modLevel = 0; // zero modulation in center position
  }
#endif
#ifdef ModulationAnalog  
  modLevel = analogRead( potPinModulation )/4; // ESP32 ADC works from 0 to 4096
  // In den modLevel sollte auch die Blasstärke einfließen!!
  // modLevel = analogRead(A1); // read voltage on analog pin A1
  
  if( modLevel > modsHi_Thr ){
    modLevel = map( modLevel,modsHi_Thr,1023,0,127); // go from 0 to full modulation when off center threshold going right(?)
  }else if( modLevel < modsLo_Thr ){
    modLevel = map( modLevel,0,modsLo_Thr,127,0); // go from 0 to full modulation when off center threshold going left(?)
  }else{
    modLevel = 0; // zero modulation in center position
  }
#endif
  
  if( modLevel != oldmod ){  // only send midi data if modulation has changed from previous value
#ifdef MIDI5Pin    
    midiSend((0xB0 | MIDIchannel), 1, modLevel);
#endif    
#ifdef ESP32_BLE_MIDI
    BLEMidiServer.controlChange(  MIDIchannel, 1, modLevel );
#endif
#ifdef BLE_MIDI
    MIDI.sendControlChange(  1, modLevel, MIDIchannel );    
#endif    
    oldmod=modLevel;
  }
}

//***********************************************************

void portamento(){
  portLevel = ads.readADC_SingleEnded(3)/512;
  
  // portLevel = map( ads.readADC_SingleEnded(3)/128 ,0,1023,0,127);
  // portLevel = map(analogRead(A2),0,1023,0,127); // read voltage on analog pin A7 and map to midi value
  // portLevel = map( portLevel, 0, 1023, 0, 127 );
  
  if( portLevel != oldport ){  // only send midi data if level has changed from previous value
#ifdef MIDI5Pin    
    midiSend((0xB0 | MIDIchannel), 5, portLevel); 
#endif     
#ifdef ESP32_BLE_MIDI
    BLEMidiServer.controlChange(  MIDIchannel, 5, portLevel );
#endif
#ifdef BLE_MIDI
    MIDI.sendControlChange( 5, portLevel, MIDIchannel  );  
#endif    
    oldport=portLevel;
  }
  if( PortK != oldportk ){ // only send midi data if status has changed from previous value
    if( PortK ){ 
      midiSend((0xB0 | MIDIchannel), 65, 127); // send portamento on
#ifdef ESP32_BLE_MIDI
      BLEMidiServer.controlChange(  MIDIchannel, 65, 127 );
#endif
#ifdef BLE_MIDI
      MIDI.sendControlChange( 65, 127, MIDIchannel ); 
#endif      
    }
    else{
#ifdef MIDI5Pin      
      midiSend((0xB0 | MIDIchannel), 65, 0); // send portamento off  
#endif      
#ifdef ESP32_BLE_MIDI
    BLEMidiServer.controlChange(  MIDIchannel, 65, 0 );
#endif
#ifdef BLE_MIDI
      MIDI.sendControlChange(  65, 0 , MIDIchannel );        
#endif      
    }
    oldportk=PortK;
  }
}

//***********************************************************

void breath(){
  int breathCC;
  breathLevel = breathLevel*0.9+pressureSensor*0.1; // smoothing of breathLevel value
  breathCC = map(constrain( breathLevel, breath_Thr, breath_max), breath_Thr, breath_max, 0, 127);
// #ifdef MIDI5Pin
  midiSend( (0xB0 | MIDIchannel), 2, breathCC );
// #endif
#ifdef ESP32_BLE_MIDI
  BLEMidiServer.controlChange(  MIDIchannel, 2, breathCC );
#endif
#ifdef BLE_MIDI
  MIDI.sendControlChange(  2, breathCC, MIDIchannel );  
#endif  
}
//***********************************************************

void readOctaves(){
  // Read octave joystick and set octave of the fingered note (run after readSwitches)
  xOctaves = ads.readADC_SingleEnded( 2 ) / 64;
  // xOctaves = analogRead(A6); // read voltage on analog pin A6
  // yOctaves = analogRead(A7); // read voltage on analog pin A7 (this is now a separate potentiometer, not joystick axis)
  joyOct = 0;
  // xOctaves is up/down and the only used octave joystick direction in this version
  if (xOctaves > octsHi_Thr) {
    joyOct++;
  } else if (xOctaves < octsLo_Thr) {
    joyOct--;
  }
  // yOctaves in this version is a separate potentiometer setting base octave -2 to +2
  // if (yOctaves > octsHi1_Thr) {
  //  joyOct++; // ++ or -- depending on joystick orientation
  // } else if (yOctaves < octsLo1_Thr) {
  //   joyOct--; // ++ or -- depending on joystick orientation
  // }
  // if (yOctaves > octsHi2_Thr) {
  //  joyOct++; // ++ or -- depending on joystick orientation
  // } else if (yOctaves < octsLo2_Thr) {
  //  joyOct--; // ++ or -- depending on joystick orientation
  //}
  //calculate midi note number from octave shifts
  fingeredNote = fingeredNote + joyOct*12;
}
//***********************************************************



void readSwitches(){  
  // Read switches and put value in variables
  // MPR121 READ all Pins at ones
  touchValue = touchSensor.touched();
  // Read Value of Touchpin
  touchPin_Value = touchRead( touchpin ); // extra Touch-Pin directly connected to ESP32 0 - 77
  if( touchPin_Value_Max < touchPin_Value ){ touchPin_Value_Max = touchPin_Value; }


  touchPin_Value_midi = map( touchPin_Value, 0, touchPin_Value_Max, 127, 0 ); // * ( touchPin_Value_Max - touchPin_Value )  );
  // Call Function for Touchpin
  if( touchPin_Value_midi <5 ){
    touchPin_Value_midi =0;
  }
  
  // Zeigefinger
  LH1=((touchValue >> 0) & 0x01);
  // Als Back / Daumenbutton verkabelt, - daher andere Funktion
  LHb=((touchValue >> 1) & 0x01);
  // Mittelfinger
  LH2=((touchValue >> 2) & 0x01);  
  // Ringfinger
  LH3=((touchValue >> 3) & 0x01);
  // Pinky
  LHp1=((touchValue >> 4) & 0x01);
  LHp2=((touchValue >> 5) & 0x01);
#ifdef DEBUGGING
  Serial.print("TV ");
  Serial.print( touchValue );

if( LH1 ){
  Serial.print("LH1 ");
}  
if( LHb ){
  Serial.print("LH-Back ");
}  
if( LH2 ){
  Serial.print("LH2 ");
}  
if( LH3 ){
  Serial.print("LH3 ");
}  
if( LHp1 ){
  Serial.print("LHp1 ");
}  
if( LHp2 ){
  Serial.print("LHp21 ");
}  
Serial.println(" ");

#endif
  // Zeigefinger  
  RH1=((touchValue >> 6) & 0x01);
  // Mittelfinger
  RH2=((touchValue >> 7) & 0x01);
  // Ringfinger
  RH3=((touchValue >> 8) & 0x01);
  // Pinky-Finger
  RHp1=((touchValue >> 9) & 0x01);
  RHp2=((touchValue >> 10) & 0x01);

  // Thumb right Hand
  RH6=((touchValue >> 11) & 0x01);
  // Portamento
  // PortK=((touchValue >> 5) & 0x01); // portamento key

#ifdef DEBUGGING
  if( RH1 ){
    Serial.print("RH1 ");
  }  
  if( RH2 ){
    Serial.print("RH2 ");
  }  
  if( RH3 ){
    Serial.print("RH3 ");
  }  
  //if( RH4 ){
  //  Serial.print("RH4 ");
  //}  
  if( RHp1 ){
    Serial.print("RHp1 ");
  }  
  if( RHp2 ){
    Serial.print("RHp2 ");
  }  
  if( RH6 ){
    Serial.print("RH6 ");
  }  
  Serial.println(" ");
#endif

if( touchValue== 3138 && lastTouchValue!= 3138  ){
  modus = modus +1;
  if( modus == 2 ){
    modus = 0;
  }  
}
if( lastTouchValue!=3072
&& lastTouchValue!=3072 ){
  modus = 0;
}

if( modus == 0 ){
    //calculate midi note number from pressed keys
    if( LHb ){
      // Fingering             H         A              G     Gb    G#       F#   F            E    D      C#     C    Oktav-Variable
      fingeredNote=startNote-3*LH1 -LH2-(LH2 && LH1) -2*LH3 -LHp1 +LHp2  -RH1-(RH1 && LH3) -RH2  -2*RH3  -RHp1 -RHp2 +12*OCTup-12*OCTdn;
    }else{
      // Left hand thumb is pressed, - Oktave höher
      // LHb=1;
      fingeredNote= 12 + startNote-3*LH1 -LH2-(LH2 && LH1) -2*LH3 -LHp1 +LHp2  -RH1-(RH1 && LH3) -RH2  -2*RH3  -RHp1 -RHp2 +12*OCTup-12*OCTdn;
    }
#ifdef DEBUGGING    
    Serial.println( fingeredNote );
#endif
    // Correction of fingerings which are used on any classic recorder ( Blockflötengriffe )
    switch( touchValue ){
      case 6: // C'
        fingeredNote=startNote-2;
        break;
      case 22: // C' + pinky 1
        fingeredNote=startNote-3;
        break;
      case 38: // C' + pinky 2
        fingeredNote=startNote-1;
        break;
      case 2054:// C'
        fingeredNote=startNote-2;
        break;
      case 2070:// C' + pinky 1  + RH-Thumb
        fingeredNote=startNote-3;
        break;
      case 2086:// C' + pinky 2  + RH-Thumb
        fingeredNote=startNote-1;
        break;
      case 3: // H
        fingeredNote=startNote-3;
        break;
      case 2051:// H  + RH-Thumb
        fingeredNote=startNote-3;
        break;
      case 11: // Bb
        fingeredNote=startNote-4;
        break;
      case 2059:// Bb  + RH-Thumb
        fingeredNote=startNote-4;
        break;
      case 4: // D'
        fingeredNote=startNote;
        break;
      case 20:// D' + pinky 1
        fingeredNote=startNote-1;
        break;
      case 36:// D' + pinky 2
        fingeredNote=startNote+1;
        break;
      case 2052: // D'  + RH-Thumb
        fingeredNote=startNote;
        break;
      case 2068:// D' + pinky 1  + RH-Thumb
        fingeredNote=startNote-1;
        break;
      case 2084:// D' + pinky 2 + RH-Thumb
        fingeredNote=startNote+1;
        break;
      default:
        // Statement(s)
        break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
    }
    if( RH6 ){
      fingeredNote= fingeredNote-12;
    }
    
#ifdef DEBUGGING    
    octave = fingeredNote / 12;
    noteInOctave = fingeredNote % 12;
    Serial.print("Oct/Note: ");
    Serial.print(octave + 1);
    Serial.print(" ");
    Serial.println(noteNames[noteInOctave]);
#endif
     
  } // modus 0
  lastTouchValue = touchValue;
}
