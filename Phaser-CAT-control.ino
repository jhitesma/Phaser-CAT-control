//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - uno_phaser_cat 2020-04-08 - VE3GTC
//
// - simple proof of concept sketch for control of Dave Benson's K1SWL Phaser digimode transceiver
//
// - this is a derivative of the example provided in the FT857D CAT Library, by Pavel Milanes, CO7WT, pavelmc@gmail.com
//   with the inclusion of I2C control for the si5351a frequency synthesizer as used in the Phaser.
//
// - Pavel describes his library as having been built with the review of various sources:
// - James Buck, VE3BUX, FT857D arduino Lib [http://www.ve3bux.com]
// - Hamlib source code
// - FLRig source code
// - Chirp source code
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - requires the ft857d library
// - https://github.com/pavelmc/FT857d/
// - this library can be installed from the Arduino IDE library function or manually from github
//
// - requires Jason NT7S etherkit si5351a library
// - https://github.com/etherkit/Si5351Arduino
// - this library can be installed from the Arduino IDE library function or manually from github
//
// - of note, Pavel has a fork of the NT7S si5351a library which has some improvements and may be worth experimenting with
// - https://github.com/pavelmc/Si5351mcu
//
// - also of note, ADAFRUTI has their own si5351a library which may also be used but I have not yet tried
// - https://github.com/adafruit/Adafruit_Si5351_Library
// - this library can be install from the Arduino IDE library function or manualy from github
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - this program uses the Arduino Wire library that is initiated from within the si5351 library
// - default Wire library enables internal pull ups on I2C SCA and SCL lines
// - the Wire library twi.c has been modified so that these two lines do not have their internal pull ups enabled
// - I2C therefore requires external pull
// - C:\apps\arduino-1.8.10\hardware\arduino\avr\libraries\Wire\src\utility
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DEBUG flag, uncomment it if you want to test it by hand
// #define DEBUG true

#define Version "0.0.1"

#include <LiquidCrystal.h>
#include <si5351.h>
#include <string.h>
#include <Wire.h>

#include <ft857d.h>

#define TX_ON                    1
#define TX_OFF                   0

#define oscillatorFrequency    25000000         // for phaser
#define frequencyCorrection    -25              // - determined emperically for current si5351a and 25MHz crystal

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - the inclusion of the following mode/freuquencies are only included here for convenience and is not good technique as
//   this should be part of an included file i.e.  xxxxxx.h 
//
// - will likely be changed in future version but OK here for now.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define FT4_DialFrequency_160m    1840000;
#define FT4_DialFrequency_80m     3575000;
#define FT4_DialFrequency_60m     5357000;
#define FT4_DialFrequency_40m     7047500;
#define FT4_DialFrequency_30m    10140000;
#define FT4_DialFrequency_20m    14080000;
#define FT4_DialFrequency_17m    18104000;
#define FT4_DialFrequency_15m    21140000;
#define FT4_DialFrequency_12m    24919000;
#define FT4_DialFrequency_10m    28180000;
#define FT4_DialFrequency_6m     50318000;

#define FT8_DialFrequency_160m    1840000;
#define FT8_DialFrequency_80m     3573000;
#define FT8_DialFrequency_60m     5357000;
#define FT8_DialFrequency_40m     7074000;
#define FT8_DialFrequency_30m    10136000;
#define FT8_DialFrequency_20m    14074000;
#define FT8_DialFrequency_17m    18100000;
#define FT8_DialFrequency_15m    21074000;
#define FT8_DialFrequency_12m    24915000;
#define FT8_DialFrequency_10m    28074000;
#define FT8_DialFrequency_6m     50313000;

#define JS8_DialFrequency_160m    1842000;
#define JS8_DialFrequency_80m     3578000;
#define JS8_DialFrequency_40m     7078000;
#define JS8_DialFrequency_30m    10130000;
#define JS8_DialFrequency_20m    14078000;
#define JS8_DialFrequency_17m    18104000;
#define JS8_DialFrequency_15m    21078000;
#define JS8_DialFrequency_12m    24922000;
#define JS8_DialFrequency_10m    28078000;
#define JS8_DialFrequency_6m     50318000;

#define defaultFrequency  FT8_DialFrequency_20m; 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// si5351a Class instantiation
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Si5351 si5351;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ft857d Class instantiation
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ft857d radio = ft857d();


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// lcd Class instantiation
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//LCD pin to Arduino
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_d4 = 4; 
const int pin_d5 = 5; 
const int pin_d6 = 6; 
const int pin_d7 = 7; 
const int pin_BL = 10; 

LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);


// Global variables

unsigned long vfoFrequency = defaultFrequency;
boolean ptt = false;
boolean splitActive = false;
boolean vfoAActive = true;
byte mode = 0;

// radio modes
#define MODE_LSB 00
#define MODE_USB 01
#define MODE_CW  02


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - function to set a freq from CAT
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - still need to devise a method of frequency calibration
//
/////////////////////////////////////////////////////////////
void catSetFreq(long f) {
    // the var freq follows the value passed, but you can do a few more thing here
    vfoFrequency = f;

    si5351.set_freq( ( vfoFrequency * 4 ) * 100ULL, SI5351_CLK0 );

    lcd.setCursor(0,1);
    lcd.print (vfoFrequency);
    lcd.setCursor(11,1);
    lcd.print ("Mhz");
  
    #if defined (DEBUG)
    // debug
    Serial.print("Active VFO freq is: ");
    Serial.println(vfoFrequency);
    #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - function to read a freq from CAT, returns frequency the Phaser is currently set to
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

long catGetFreq() {
    // this must return the freq as an unsigned long in Hz, you must prepare it before

    #if defined (DEBUG)
    // debug
    Serial.println("Asked for freq");
    #endif

    // pass it away
    return vfoFrequency;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - the remaining fuctions do nothing for the Phaser
// - include or remove at your discretion, most will likely disappear in future versions as their utility or necessity is
//   is determined
// - there may also be new functions depending on needs.
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - part of ft857d library example and is only temporarily included, does nothing for the Phaser radio
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// function to run when we must put radio on TX/RX
//
//////////////////////////////////////////////////////

void catGoPtt(boolean pttf) {
    // the var ptt follows the value passed, but you can do a few more thing here
    ptt = pttf;

    #if defined (DEBUG)
    // debug
    Serial.print("PTT Status is: ");
    Serial.println(ptt);
    #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - part of ft857d library example and is only temporarily included, does nothing for the Phaser radio
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// function to run when VFOs A/B are toggled
//
//////////////////////////////////////////////////////

void catGoToggleVFOs() {
    // here we simply toggle the value
    vfoAActive = !vfoAActive;

    #if defined (DEBUG)
    // debug
    Serial.print("VFO A active?: ");
    Serial.println(vfoAActive);
    #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - part of ft857d library example and is only temporarily included, does nothing for the Phaser radio
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// function to set the mode from the cat command
//
//////////////////////////////////////////////////////

void catSetMode(byte m) {
    // the var mode follows the value passed, but you can do a few more thing here
    mode = m;

    #if defined (DEBUG)
    // debug
    Serial.print("Active VFO mode is: ");
    Serial.println(mode);
    #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - part of ft857d library example and is only temporarily included, does nothing for the Phaser radio
// - may decide to hardcode this to always return USB
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  function to pass the mode to the cat library
//
//////////////////////////////////////////////////////

byte catGetMode() {
    // this must return the mode in the wat the CAT protocol expect it

    #if defined (DEBUG)
    // debug
    Serial.println("Asked for mode");
    #endif

    // pass it away
    return mode;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - part of ft857d library example and is only temporarily included, does nothing for the Phaser radio
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// function to pass the smeter reading in RX mode
//
//////////////////////////////////////////////////////

byte catGetSMeter() {
    // this must return a byte in with the 4 LSB are the S meter data
    // so this procedure must take care of convert your S meter and scale it
    // up to just 4 bits

    #if defined (DEBUG)
    // debug
    Serial.println("Asked for S meter");
    #endif

    // pass it away (fixed here just for testing)
    return byte(4);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - part of ft857d library example and is only temporarily included, does nothing for the Phaser radio
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// function to pass the TX status
//
//////////////////////////////////////////////////////

byte catGetTXStatus() {
    /*
     * this must return a byte in wich the different bits means this:
     * 0b abcdefgh
     *  a = 0 = PTT off
     *  a = 1 = PTT on
     *  b = 0 = HI SWR off
     *  b = 1 = HI SWR on
     *  c = 0 = split on
     *  c = 1 = split off
     *  d = dummy data
     *  efgh = PO meter data
     */

    #if defined (DEBUG)
    // debug
    Serial.println("Asked for TX status");
    #endif

    // you have to craft the byte from your data, we will built it from
    // our data
    byte r = 0;
    // we fix the TX power to half scale (8)
    r = ptt<<7 + splitActive<<5 + 8;

    return r;
}


void setup() {
  // preload the vars in the cat library
  radio.addCATPtt(catGoPtt);
  radio.addCATAB(catGoToggleVFOs);
  radio.addCATFSet(catSetFreq);
  radio.addCATMSet(catSetMode);
  radio.addCATGetFreq(catGetFreq);
  radio.addCATGetMode(catGetMode);
  radio.addCATSMeter(catGetSMeter);
  radio.addCATTXStatus(catGetTXStatus);

  // now we activate the library
  radio.begin(9600, SERIAL_8N1);

  #if defined (DEBUG)
  // serial welcome
  Serial.println("CAT Serial Test Ready");
  #endif

  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other than 25 MHz
  // last value is a frequency correction value if I remember correctly
  si5351.init( SI5351_CRYSTAL_LOAD_8PF, oscillatorFrequency, 0 );

  // Set CLK0 output
  si5351.drive_strength( SI5351_CLK0, SI5351_DRIVE_8MA );            // Set for max power if desired
  si5351.output_enable( SI5351_CLK0, TX_OFF );                       // Disable the clock initially

  si5351.output_enable( SI5351_CLK0, TX_ON );                        // turn on the output
  si5351.set_freq( ( vfoFrequency * 4 ) * 100ULL , SI5351_CLK0 );    // and set to our default frequency

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Phaser 20");
  lcd.setCursor(0,1);
  lcd.print (vfoFrequency);
  lcd.setCursor(11,1);
  lcd.print ("Mhz");

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - begin main loop
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
    radio.check();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - that's all Folks!
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
