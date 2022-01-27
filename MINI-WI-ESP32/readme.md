# MINI WI ESP32

I replaced the Arduino by an ESP32, still use an MPR121 via I2C ( GPIO-5 for SDA, GPIO-23 for SCL ) and an ADS1115 to get the differential pressure related voltage.
While COVID and the "Chip-Missing" - Pandemic, I bought 2 (bad or worse) DP53DP which are Differential Pressure Mesurement Sensors which need 2 ADC-inputs on the ADS1115.

Video ( german ) https://www.youtube.com/watch?v=_GEmQJgfbyA
[![Youtube](https://github.com/ErichHeinemann/MiniWI/blob/master/MINI-WI-ESP32/mini_wi_esp32_yt.png?raw=true)](https://www.youtube.com/watch?v=_GEmQJgfbyA "Mini WI Windinstrument on Youtube (german)")

As I had already bad experience with the ADC of the ESP, I tried to use a ADS115 and would add a MPU6050 later in the sketch to send MIDI-CC-Messages.

I don´t want to support a physical MIDI-Out, - to clunky wires and I did think of an internal battery but prefer an external USB-Powerbank which I could use for dirrent purposes ... I mainly play guitar, not MINI-WI. I did not implement the Joysticks in the code yet because I wanted to use the fingering of a normal recorder and I didn´t had Joysticks laying around. ( update - got Joystick )

Basically, I am a guitar-player and not a flute or sax-player. In school, we all had to learn to play the recorder and so, I tried to change the fingering to be the same like for a recorder.
I moved one touch-sensor to the back, used 2 sensors for the left hand pinky and 2 for the right pinky finger and one sensor (RH6) for the thumb of the right hand. 
Currently the right-hand thumb (RH6), switches to a lower octave.

For the body of the Mini-WI I used this material:
- an old inner cardboard role/pipe r=3.1cm by 40cm , 
- pimped it up with adhesive film ( one in gold and one in orange )
- thinnest brass-tube (1 meter ) from the hardwarestore ( around 3mm diameter )
- Silicon-Tube ( 50cm ) ( aquarium accessories )
- 12 cheap brass (looking) upholstery nails from hardwarestore ( 1cm diameter )
- thin isolated wire
- some sticky tape to manage the wires

I added a OpenSCAD-File which I used to create different 3D-printed parts like rings for the:
- fingerholes, - finally not used
- mouthpiece(s) , - a thin version and a wider version 
- top and bottom to cover the holes of the tube
- a holder to be able to mount the pressure-sensor from the outside

There was no need to cut the cardboard-role from top to bottom! 

Step 1:
Cut the cardboard-role to Your prefered length ( use Your hands to get a good length and don´t forget the reserved place/area for the ESP32 and other modules, hold it like a flute and find the length which looks good. )

Step 2: ( optional )
Wrap the adhesive film around it, leave top and bottom open

Step 3: 
Find the best places for the captive sensors. Hold the pipe like You would hold it for playing it as a flute or oboe etc.
Mark the position of Your fingers. Keep in mind, the pinkys will get 2 sensors, don´t spread Your pinky to much to reach out to the second hole.
As the sensors are captive sensors, there has to be enough space between the fingers and sensors.
Sensors for the left pinkys should be wider, for the right-hand pinkys it should be more narrow to be able to get both with the pinky at once.

Step 4:
Press the brass-Nails a bit into the cardboard-pipe to create simple holes.

Step 5:
Find a good position ( in my case on the right-side ) where You would like to inlay the pressure-sensor into the pipe.
Print out or build Your own holder for the pressure sensor. Take mesasurement and cur out an rectangle to push the sensor to the inside of the box. 
I have used a fresh sharp box cutter and a lot of respect.

Step 6:
print out the mouthpiece, bend the brass-tube to let it look like to upper part of a sax.
cut it  to let match the length of the mini-wi from mouthpiece down to the bottom! - full length!
Print out the Split-Box and some Pipe-Holders.

Step 7:
Do a second cut of the Brass-Pipe some centimeters below the pressure sensor where You have to install the Split-Box.
The Splitbox splits the air, comming from the mouthpieces and pressing it back upwords to the pressure-sensor which is facing downwards. 
.. any spit will flow downwards into the second brass pipe to the bottom.
Only Your feet will get wet after a long jam-session, - not Your fingers.
use 2 or more pipeholders to fix the brass-pipe onto the cardboard-pipe.

Step 8:
Solder thin isolated wire to the tip of the (12) brass-nails and loop the wire to the inside of the pipe. 
The wires should all fall down to the bottom of Your mini-wi.
Start with the highest Wire ( High c hole )

Step 9:
Print out the Bottom and later the Top to cover the Cardboard-Pipe.
I have tried to design two versions for the bottom. 
a) the USB faces straigt downwards ... Your have to lay down the instrument to the side if You pause Your jam-session
b) the USB faces backwards to You and You get a flat bottom, - Instrument could stand on the bottom.

Step 10:
use some Breadbaord/Stripboard which has a size of around 8 x 4cm to hold the modules:
MPR121, ADS1115 and ESP32 or only MPR121 and ADS1115 ( depends on the bottom-version a) or b) ) 
Use sockets for the modules or not, it is up to You.

Step 11:
Solder all modules ( depends on version of bottom ) to the Stripboards

Step 12:
Create the connection for VCC/VDD 3.3Volts, GND, SDA and SCL between all modules.
Connect the Wires from the brass-nails ( Touch Sensors ) from Bottom Down to the 12 Inputs 0 to 11 of the MPR121 module.

Step 13: 
Insert the MPX DP53DP (Pressure-Sensor ) via the silicon-tube to the Split-Box.
solder 4 wires to the DP52DP ( V+ goes to 3.3V or 5 Volts ...  I had measured very low voltages and decided to go to 5 Volts.
Out + / Out - have to be connected to the ADS1115 ADC0 and ADC1.
Check the documentation for YOUR pressure-seensor as I did not find a genral pinout of theem. they are different from modell to model!

Step 14:
Measure the connections to the ESP32 with a multimeter, - are there any shorts between SDA/SCL or +VDD and GND??
Check the Connection from the Touch-Sensors to the MPR121

Step 15:
Check the airpressure, - blow into the mouthpiece and check for any leakages. Air and spit should come out at the end of the brass-pipe at the bottom of the mini-wi.

Step 15:
Perhaps You have to install all the needed librarys or the Arduino IDE.
Upload the Core to the ESP32 ( perhaps You have to press "BOOT" to get it) 
I have used the Adafruit-Libraries for ADS1115 and the MPR121 and finally the "ESP32 BLE MIDI" - Library

Step 16:
Start an App on Your Smartphone which support MIDI BLE. 
Power up the Mini-WI, after the LED flashes stopps, blow into the moutpiece and press some brass-nails.

Step 17:
optimize it Your way, add Joysticks, more Touch-Sensors to the ESP32. 
Implement Your funktion when RH6 is pressed or Joysticks are moved.

There are a lot videos which explain how to connect a BLE - MIDI-Controller to Your Smartphone or PC.
