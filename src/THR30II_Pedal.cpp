 /*THR30II Pedal using Teensy 3.6
* Martin Zwerschke 04/2021
* 
* Heavily modified by Buhjuhwuh
*
* Prerequesites:
*  - Library "USBHost_t3" for Teensy 3.6  
*  - Variant "2_8_Friendly_t3" of Library "ST7789_t3" for Teensy 3.6
*  
*  - Library "Bounce2"   Web: https://github.com/thomasfredericks/Bounce2
*  - Library "SPI" version=1.0  Web: http://www.arduino.cc/en/Reference/SPI  , architectures=sam
*  - Library "SDFAT"   (only if you want to use a SD-card for the patches, can be avoided with Teensy's big PROGMEM)
*  - Library "ArduinoJson"  https://arduinojson.org (Copyright Benoit Blanchon 2014-2021)
*  - Library "Adafruit_gfx.h" https://github.com/adafruit/Adafruit-GFX-Library
*	-Library "ArduinoQueue.h"  https://github.com/EinarArnason/ArduinoQueue
*
* IDE:
*  I use "PlatformIO"  IDE.
*  But genuine Arduino-IDE with additions for Teensy (Teensyduino) should work as well.
*  THR30II_Pedal.cpp
*
* last modified 19/02/2023 Buhjuhwuh
* Author: Martin Zwerschke
*/

#include <Arduino.h>
#include <USBHost_t36.h>    	//Without Paul Stoffregen's Teensy3.6-Midi library nothing will work! (Mind increasing rx queue size in  "USBHost_t3" :   RX_QUEUE_SIZE = 2048)
#include <SPI.h>    		  	//For communication with TFT-Display (SD-Card uses SDIO -SDHC-Interface on Teensy 3.6)
#include <Adafruit_GFX.h>   	//Virtual superclass for drawing (included in "2_8_Friendly_t3.h")
#include <TFT_eSPI.h>			//TFT screen library
#include <AceButton.h>			//Button library - debouncing and short/long press distinction
using namespace ace_button;
#include <Adafruit_NeoPixel.h>	//Status LED library
#include <ArduinoJson.h> 	  	//For patches stored in JSON (.thrl6p) format
#include <algorithm>  	 	  	//For the std::find_if function
#include <vector>			  	//In some cases we will use dynamic arrays - though heap usage is problematic
							  	//Where ever possibly we use std::array instead to keep values on stack instead
#include "THR30II_Pedal.h"
#include "Globals.h"		  	//For the global keys	
#include "THR30II.h"   			//Constants for THRII devices	  

// Locally supplied fonts
//#include "Free_Fonts.h"
#include "NotoSansBold15.h"
#include "NotoSansBold36.h"
#include "NotoSansMonoSCB20.h"
// The font names are arrays references, thus must NOT be in quotes ""
#define AA_FONT_SMALL NotoSansBold15
#define AA_FONT_LARGE NotoSansBold36
#define AA_FONT_MONO  NotoSansMonoSCB20 // NotoSansMono-SemiCondensedBold 20pt

// Old includes--------------------------------------------
// #include <ArduinoQueue.h>   //Include in THR30II_pedal.h (for queuing in- and outgoing SysEx-messages) 
// #include <2_8_Friendly_t3.h>  //For 240*320px Color-TFT-Display (important: use special DMA library here, because otherwise it is too slow)
// #include <Bounce2.h>	//Debouncing Library for the foot switch buttons
// #include <EasyButton.h>	//Better debouncing library	
// #include "rammon.h"  //(for development only!) Teensy 3.x RAM Monitor copyright by Adrian Hunt (c) 2015 - 2016
// #include <ST7789_t3.h>
// #include "TFT_Help.h"  		  	//Some local extensions to the ST7789 Library (Martin Zwerschke)
//---------------------------------------------------------

// Initialise tft
TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft); // Sprite class needs to be invoked

/* Set USE_SDCARD to 1 to use SD card reader to read from patch
   file. If USE_SDCARD is set to 0 then the file patches.h is included
   which needs to define a PROGMEM variable named patches.
*/
#define USE_SDCARD 1
//#define USE_SDCARD 0
#if USE_SDCARD
#include <SD.h>
const int sd_chipsel=BUILTIN_SDCARD;
File32 file;

// #define PATCH_FILE "patches.txt" //If you want to use a SD card, a file with this name contains the patches
#define PATCH_FILE "jsonSDtest.txt" 
#else
//#include <avr/pgmspace.h>
#include "patches.h"  //Patches located in PROGMEM  (for Teensy 3.6/4.0/4.1 there is enough space there for hundreds of patches)
#endif

// Include font definition file (belongs to Adafruit GFX Library )
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansOblique24pt7b.h>
#include <Fonts/FreeSansOblique18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

//Normal TRACE/DEBUG
#define TRACE_THR30IIPEDAL(x) x	// trace on
//#define TRACE_THR30IIPEDAL(x)	// trace off

//Verbose TRACE/DEBUG
#define TRACE_V_THR30IIPEDAL(x) x	// trace on
//#define TRACE_V_THR30IIPEDAL(x)	// trace off

// Define button input pins (14-23)
#define BUTTON_1_PIN  21    // Patch submit
#define BUTTON_2_PIN  14    // Amp select
#define BUTTON_3_PIN  15    // Exp mode
#define BUTTON_4_PIN  22    // Patch-
#define BUTTON_5_PIN  19    // Patch+
#define BUTTON_6_PIN  16    // Tempo/tuner
#define BUTTON_7_PIN  23    // Dynamics
#define BUTTON_8_PIN  20    // Effect
#define BUTTON_9_PIN  18    // Delay
#define BUTTON_10_PIN 17    // Reverb
int button_map[10] = {2,3,6,10,9,5,8,1,4,7};

// Define other input/output pins
#define LED_PIN       		12
#define PEDAL_1_PIN       	24
#define PEDAL_2_PIN       	25
#define PEDAL_1_SENSE_PIN 	26
#define PEDAL_2_SENSE_PIN 	27

int pedal_1_val = 0;
int pedal_2_val = 0;
int pedal_1_old_val = 0;
int pedal_2_old_val = 0;
int pedal_margin = 1;
bool pedal_1_sense = 0;
bool pedal_2_sense = 0;
enum ampSelectModes {COL, AMP, CAB};
ampSelectModes amp_select_mode = COL;
enum dynModes {Boost, Comp, Gate};
dynModes dyn_mode = Boost;
bool boost_activated = 0;

USBHost Usb;   //On Teensy3.6 this is the class for the native USB-Host-Interface  
//It is essential to increase rx queue size in  "USBHost_t3" :   RX_QUEUE_SIZE = 2048  should do the job, i use 4096 to be safe

MIDIDevice_BigBuffer midi1(Usb);  
//MIDIDevice midi1(Usb);  //Should work as well, as long as the RX_QUEUE_SIZE is big enough

bool complete=false;  //used in cooperation with the "OnSysEx"-handler's parameter "complete" 

//RamMonitor rm;  //During development to keep an eye on stack and heap usage

// Initialise button instances
AceButton button1(BUTTON_1_PIN);
AceButton button2(BUTTON_2_PIN);
AceButton button3(BUTTON_3_PIN);
AceButton button4(BUTTON_4_PIN);
AceButton button5(BUTTON_5_PIN);
AceButton button6(BUTTON_6_PIN);
AceButton button7(BUTTON_7_PIN);
AceButton button8(BUTTON_8_PIN);
AceButton button9(BUTTON_9_PIN);
AceButton button10(BUTTON_10_PIN);

// Forward reference to prevent compiler becoming confused.
void read_buttons(AceButton*, uint8_t, uint8_t);
int button_state = 0;

//Variables and constants for display 
int16_t x=0, y=0, xx1 = 0, yy1 = 0;
String s1("MIDI");
#define MIDI_EVENT_PACKET_SIZE  64 //for THR30II    was much bigger on old THR10

// LED initialisation
#define LED_COUNT 10
#define BRIGHTNESS 85 // Set BRIGHTNESS to about 1/3 (max = 255)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
int r, b, g = 0;
uint32_t rgbcolour = 0;

uint8_t buf[MIDI_EVENT_PACKET_SIZE];  //Receive buffer for incoming (raw) Messages from THR30II to controller
uint8_t currentSysEx[310]; //buffer for the currently processed incoming SysEx message (maybe coming in spanning over up to 5 consecutive buffers)
						   //was needed on Arduino Due. On Teensy3.6 64 could be enough

String patchname;  //limitation by Windows THR30 Remote is 64 Bytes.
String preSelName; //Global variable: Name of the pre selected patch

class THR30II_Settings THR_Values;			//actual settings of the connected THR30II
class THR30II_Settings stored_THR_Values;   //stored settings, when applying a patch (to be able to restore them later on)
class THR30II_Settings stored_Patch_Values; //stored settings of a (modified) patch, when applying a solo (to be able to restore them later on)

static volatile int16_t presel_patch_id;   	//ID of actually pre-selected patch (absolute number)
static volatile int16_t active_patch_id;   	//ID of actually selected patch     (absolute number)
// static volatile int8_t group_id; 		   	//DELETE? actual Patch Group ID  
// static volatile int8_t settings_id;        	//DELETE? ID of setting inside Patch group (relative number)
// static volatile int8_t max_group_id;       	//DELETE? depends on total number of patches (grouped by 5 normal + 5 Solo)
// static volatile int8_t max_settings_id;    	//DELETE? only <4, if even first group is not complete, otherwise ==4 for all groups except last  (relative number)
// static volatile int8_t max_solo_id;        	//DELETE? highest (0-based) id for solo in all groups except the last group (-1 if no dedicated solo file at all) (relative number)
// static volatile int8_t last_solo_id;       	//DELETE? highest (0-based) id for solo in the last group (-1 if no dedicated solo file in last patch) (relative number)
// static volatile int8_t last_settings_id;   	//DELETE? <4 if last group is incomplete  (relative number)
static volatile bool send_patch_now = false;//pre-select patch to send (false) or send immediately (true)

static std::vector <String> libraryPatchNames;  //all the names of the patches stored on SD-card or in PROGMEN

//class Outmessage OM_dummy(SysExMessage(nullptr,0),0,false,false);  //make a empty Outmessage as a dummy for use in some cases

static volatile uint16_t npatches = 0;   //counts the patches stored on SD-card or in PROGMEN

#if USE_SDCARD
	std::vector<std::string> patchesII;   //patches are read in dynamically, not as a static PROGMEM array
#else
	                                      //patchesII is declared in "patches.h" in this case
#endif



/////////////////////////////////////////////////////////////////////////////////////////////////
    // SETUP //
/////////////////////////////////////////////////////////////////////////////////////////////////

void setup()  //do preparations
{
    //rm.run();  //keep Memory Monitor up to date (during development to keep an eye on stack and heap usage)
	
	// this is the magic trick for printf to support float
  	asm(".global _printf_float");  //make printf work for float values on Teensy
	
	delay(100);
	uint32_t t_out=millis();
	while (!Serial)
	{
		if(millis()-t_out > 3000)   //Timeout, if no serial monitor is used  (make shorter in release version)
		{
			break;
		}
	}   // wait for Arduino Serial Monitor
    Serial.begin(230400);  //Debug via USB-Serial (Teensy's programming interface, where 250000 is maximum speed)
	Serial.println(F("THR30II Footswitch"));
	
	while( Serial.read() >0 ) {}; //make read buffer empty
    
	//SD-card setup
 	#if USE_SDCARD
	if (!SD.begin(sd_chipsel))
	{
       Serial.println(F("Card failed, or not present."));
    }
    else
	{
		Serial.println(F("Card initialized."));
		
			TRACE_THR30IIPEDAL( Serial.print(F("VolumeBegin->success. FAT-Type: "));)

			if(file.open(PATCH_FILE))  //Open the patch file "patches.txt" in ReadOnly mode
			{
				while(file.isBusy());
				uint64_t inBrack=0;  //actual bracket level
				uint64_t blockBeg=0, blockEnd=0;  //start and end point of actual patch inside the file
				bool error=false;
				
				Serial.println(F("\n\rFound \"patches.txt\"."));
				file.rewind();

				char c=file.peek();
				
				while(!error && file.available32()>0)
				{ 
					// TRACE_THR30IIPEDAL(Serial.printf("%c",c);)
				
					switch(c)
					{
						case '{':
							if(inBrack==0)  //top level {} block opens here
							{
								blockBeg = file.curPosition();
								TRACE_THR30IIPEDAL(Serial.printf("Found { at %d\n\r",blockBeg);)
							}
							inBrack++;
							// TRACE_THR30IIPEDAL(Serial.printf("+%d\n\r",inBrack);)
						break;

						case '}':
							if(inBrack==1)  //actually open top level {} block closes here
							{	
								blockEnd = file.curPosition();
								
								TRACE_THR30IIPEDAL(Serial.printf("Found } at %d\n\r",blockEnd);)
								
								error=!(file.seekSet(blockBeg-1));
								
								if(!error)
								{
									String pat = file.readString(blockEnd-blockBeg+1);

									if(file.getReadError()==0)
									{
										patchesII.emplace_back(std::string(pat.c_str()));
										//TRACE_V_THR30IIPEDAL(Serial.println(patchesII.back().c_str()) ;)
										npatches++;
										TRACE_V_THR30IIPEDAL(Serial.printf("Loaded npatches: %d",npatches) ;)
									}
									else
									{
										TRACE_THR30IIPEDAL(Serial.println("Read Error");)
										error=true;
									}
								}
								else
								{
									TRACE_THR30IIPEDAL( Serial.println("SeekSet(blockBeg-1) failed!" );)
								}

								error=!(file.seekSet(blockEnd));
								
								if(error)
								{
									TRACE_THR30IIPEDAL( Serial.println("SeekSet(blockEnd) failed!" );)	
								}
								else
								{
									TRACE_THR30IIPEDAL( Serial.println("SeekSet(blockEnd) succeeded." );)	
								}
							}
							else if(inBrack==0)  //block must not close, if not open!
							{
								TRACE_THR30IIPEDAL(Serial.printf("Unexpected } at %d\n\r",file.curPosition());)
								error=true; 	
								break;
							}
							inBrack--;
							// TRACE_THR30IIPEDAL(Serial.printf("-%d\n\r",inBrack);)
						break;
					}			

					if(!error)
					{
						if(file.available32()>0)
						{
							c=(char) file.read();
							while(file.isBusy());
						}
					}
					else
					{
						TRACE_THR30IIPEDAL( Serial.println("Error proceeding in file!" );)	
					}
					//TRACE_THR30IIPEDAL(Serial.println(inBrack);)

				}

				TRACE_THR30IIPEDAL(Serial.println("Read through \"patches.txt\".\n\r");)
				
				file.close();
			}
			else
			{
				Serial.println(F("File \"patches.txt\" was not found on SD-card."));
			}
	}
	#else
		npatches = patchesII.size();  //from file patches.h 
		TRACE_V_THR30IIPEDAL(Serial.printf("From PROGMEM: npatches: %d",npatches) ;)
	#endif	

    // Button setup
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);
    pinMode(BUTTON_4_PIN, INPUT_PULLUP);
    pinMode(BUTTON_5_PIN, INPUT_PULLUP);
    pinMode(BUTTON_6_PIN, INPUT_PULLUP);
    pinMode(BUTTON_7_PIN, INPUT_PULLUP);
    pinMode(BUTTON_8_PIN, INPUT_PULLUP);
    pinMode(BUTTON_9_PIN, INPUT_PULLUP);
    pinMode(BUTTON_10_PIN, INPUT_PULLUP);

    // Configure the ButtonConfig with the event handler, and enable Click and LongPress events.
    ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
    buttonConfig->setEventHandler(read_buttons);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
	buttonConfig->setClickDelay(500);		// short presses < 500ms (def 200ms)
  	buttonConfig->setLongPressDelay(500);	// long presses > 500ms (def 1000ms)

    // Display setup
	tft.begin();
  	tft.setRotation(3);
  	spr.setColorDepth(16); // 16 bit colour needed to show antialiased fonts
  	tft.fillScreen(TFT_BLACK);
	drawConnIcon(false);

  	// Initialise LED strip
 	strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  	strip.show();            // Turn OFF all pixels ASAP
  	strip.setBrightness(BRIGHTNESS);
	
	// Initialise patch status
	if(npatches>0) // patch(es) exist(s)
	{
		presel_patch_id = 1;   //preselect the first available patch (if available at all)
	}
	else // no patches
	{
		presel_patch_id = -1;   //no preselected patch possible (because none available at all)
	}
	active_patch_id =-1;   //always start up with local settings

	TRACE_THR30IIPEDAL( Serial.printf(F("\n\rThere are %d JSON patches in patches.h / patches.txt.\n\r"), npatches);)
	TRACE_THR30IIPEDAL( Serial.println(F("Fetching Library Patch Names:")); )

	DynamicJsonDocument djd (4096);   //Buffer size for JSON decoding of thrl6p-files is at least 1967 (ArduinoJson Assistant) 2048 recommended
	
	for(int i = 0; i<npatches; i++)   //get the patch names by de-serializing
	{
			using namespace std;
			djd.clear();
			DeserializationError dse= deserializeJson(djd, patchesII[i]);
			if(dse==DeserializationError::Ok)
			{
				libraryPatchNames.push_back( (const char*) (djd["data"]["meta"]["name"]) ) ;
				TRACE_THR30IIPEDAL( Serial.println((const char*) (djd["data"]["meta"]["name"])); )
			}
			else
			{
				Serial.print(F("deserializeJson() failed with code "));Serial.println(dse.f_str());
				libraryPatchNames.push_back("error");
				TRACE_THR30IIPEDAL( Serial.println("JSON-Deserialization error. PatchesII[i] contains:");)
				Serial.println(patchesII[i].c_str());
			}
	 }

	delay(250);  //could be reduced in release version

  	midi1.begin();
  	midi1.setHandleSysEx(OnSysEx);
	
	delay(250);  //could be reduced in release version

}  //End of Setup()



/////////////////////////////////////////////////////////////////////////////////////////////////
    // READ BUTTONS //
/////////////////////////////////////////////////////////////////////////////////////////////////
// When button is pressed, set button_state to value 1-10
// When button is held,    set button_state to value 11-20
void read_buttons(AceButton* button, uint8_t eventType, uint8_t buttonState)
{
  	switch (eventType) {
    	case AceButton::kEventClicked:
      		button_state = button_map[button->getPin() - 14];
	  		TRACE_V_THR30IIPEDAL(Serial.println();)
			TRACE_V_THR30IIPEDAL(Serial.println("==============================");)
			TRACE_V_THR30IIPEDAL(Serial.print(F("Button "));)
			TRACE_V_THR30IIPEDAL(Serial.print(button_state);)
			TRACE_V_THR30IIPEDAL(Serial.println(F(" clicked"));)
      	break;
    	case AceButton::kEventLongPressed:
			button_state = button_map[button->getPin() - 14];
			TRACE_V_THR30IIPEDAL(Serial.println();)
			TRACE_V_THR30IIPEDAL(Serial.println("==============================");)
			TRACE_V_THR30IIPEDAL(Serial.print(F("Button "));)
			TRACE_V_THR30IIPEDAL(Serial.print(button_state);)
			button_state += 10;
			TRACE_V_THR30IIPEDAL(Serial.println(F(" held"));)
      	break;
  	}
}   //End of read_buttons()

//global variables, because values must be stored in between two calls of "parse_thr"
//volatile bool in_sysex = false ; //set, if a SysExMessage has begun, reset if it's end is found (was neccessary for ArduinoDUE / old THR10)
								   //A SysEx may span over up to 5 consecutive buffers, it is handled on incoming of the last buffer

uint16_t cur = 0;      //current index in actual SysEx
uint16_t cur_len = 0;  //length of actual SysEx if completed

volatile static byte midi_connected = false;
//uint32_t static received;        //number of bytes received
static byte maskUpdate = false; //set, if a value changed and the display mask should be updated soon
static byte maskActive = false; //set, if local THR-settings or modified patch settings are valid
								 // (e.g. after init or after turning a knob or restored local settings)
static byte preNameActive = false; //pre-selected name is shown, not settings mask nor active patchname

//enum UIStates { UI_idle, UI_init_act_set, UI_act_vol_sol, UI_patch, UI_ded_sol, UI_pat_vol_sol}; //States for the patch/solo activation user interface
//enum UIStates {UI_idle, UI_home_amp, UI_home_patch, UI_edit, UI_save, UI_name, UI_init_act_set, UI_act_vol_sol, UI_patch, UI_ded_sol, UI_pat_vol_sol};
UIStates _uistate = UI_idle;  //Always begin with idle state until actual settings are fetched



/////////////////////////////////////////////////////////////////////////////////////////////////
    // TIMING //
/////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t tick1=millis();  //Timer for regular mask update
uint32_t tick2=millis();  //Timer for background mask update for forgotten values
uint32_t tick3=millis();  //Timer for switching back to Grid after showing preselected patch
uint32_t tick4=millis();  //unused timer for regularly requesting all values 
uint32_t tick5=millis();  //unused timer for resetting the other timers 

void timing()
{
     if(millis()>tick1+40)  //mask update if it was required because of changed values
     {
		 if(maskActive && maskUpdate)
		 {
		  THR_Values.updateStatusMask(0,85);
		  tick1=millis();  //start new waiting period
		  return;
		 }
	 }

	 if(millis()>tick2+1000)  //force mask update to avoid "forgotten" value changes
	 {
	    if(maskActive)
		{
		//  THR_Values.updateConnectedBanner();
		 THR_Values.updateStatusMask(0,85);
		 tick2=millis();  //start new waiting
		 return;
		}
	 }

	 if(millis()>tick3+1500)  //switch back to mask or selected patchname after showing preselected patchname for a while
	 {		
		 if(preNameActive)
		 {
			preNameActive=false;  //reset showing pre-selected name
			
			//patch is not active => local Settings mask should be activated again
			maskActive=true;
			// drawStatusMask(0,85);
			maskUpdate=true;
			
			//tick3=millis();  //start new waiting not necessary, because it is a "1-shot"-Timer
			return;	
		 }
	 }

	 if(millis()>tick4+3500)  //force to get all actual settings by dump request
	 {
	    if(maskActive)
		{
		 //send_dump_request();  //not used - THR-Remote does not do this as well
		 tick4=millis();  //start new waiting
		 return;
		}
	 }

	 if(millis()>tick5+15000)
	 {
		//TRACE_THR30IIPEDAL( Serial.println("resetting all timers (15s-timeout)"); )
		// tick5=millis();
		// tick4=millis();
	 	// tick3=millis();
		// tick2=millis();
		// tick1=millis();
	 }
}



/////////////////////////////////////////////////////////////////////////////////////////////////
    // LOOP //
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop()  //infinite working loop:  check midi connection status, poll buttons & pedal inputs, run state machine
{
  	if(midi1)   //Is a device connected?
	{
		midi1.read();   //Teensy 3.6 Midi-Library needs this to be called regularly
			//Hardware IDs of USB-Device (should be  VendorID = 0499, ProductID = 7183)   Version = 0100
			//sprintf((char*)buf, "VID:%04X, PID:%04X", Midi.vid, Midi.pid);  
			//Serial.println((char*)buf);	
		
		if(complete && midi1.getSysExArrayLength() >=0)
		{
			//printf("\r\nMidi Rec. res. %0d.\t %1d By rec.\r\n",midi1.getSysExArrayLength(),(int)received);
			//Serial.println(String(midi1.getSysExArrayLength())+String(" Bytes received"));
			//Serial.println(String(cur_len)+String(" Bytes received."));
	
			inqueue.enqueue(SysExMessage(currentSysEx,cur_len));  //feed the queue of incoming messages
			
			complete = false;  
			
		} //of "if(complete && ..."
		else  
		{
			if(!midi_connected)
			{
				drawConnIcon(midi_connected);
				
				Serial.println(F("\r\nSending Midi-Interface Activation..\r\n")); 
				send_init();  //Send SysEx to activate THR30II-Midi interface
				//ToDo: only set true, if SysEx's were acknowledged
				midi_connected=true;  //A Midi Device is connected now. Proof later, if it is a THR30II
			
				drawConnIcon(midi_connected);

				drawPatchName(ST7789_VIOLET, "THR Panel");

				maskActive=true;  //tell GUI, that it must show the settings mask
				// drawStatusMask(0,85); //y-position results from height of the bar-diagram, that is drawn bound to lowest display line
				maskUpdate=true;  //tell GUI to update settings mask one time because of changed settings		
			}
		} //of "MIDI not connected so far"
	} //of "if(midi1)"  means: a device is connected
    else if (midi_connected)  //no device is connected (any more) but MIDI was connected before =>connection got lost 
    {
        //Display "MIDI" in RED (because connection was lost)
	 	midi_connected = false;  //this will lead to Re-Send-Out of activation SysEx's
		_uistate=UI_idle; //re-initialize UI state machine
		THR_Values.ConnectedModel=0x00000000;

		drawConnIcon(midi_connected);

    }//of: if midi_connected

	//uint8_t a=0;
		
	timing(); //GUI timing must be called regularly enough. Locate the call in the working loop	

	WorkingTimer_Tick(); //timing for MIDI-message send/receive is shortest loop

    // Poll buttons - should be called every 4-5ms or faster, for the default debouncing time of ~20ms.
    button1.check();
    button2.check();
    button3.check();
    button4.check();
    button5.check();
    button6.check();
    button7.check();
    button8.check();
    button9.check();
    button10.check();
	
	// Poll pedal inputs
    pedal_1_sense = !digitalRead(PEDAL_1_SENSE_PIN);  //sense whether pedal 1 present
	if (pedal_1_sense) {
		pedal_1_old_val = pedal_1_val;
		pedal_1_val = 1023-analogRead(PEDAL_1_PIN);   //read in pedal 1 input
	}
	else {
		pedal_1_val = 0;
	}
	if ((pedal_1_val >= (pedal_1_old_val - pedal_margin)) && (pedal_1_val <= (pedal_1_old_val + pedal_margin))) {
		maskUpdate=true;  //request display update to show new states quickly
	}

  	pedal_2_sense = !digitalRead(PEDAL_2_SENSE_PIN);  //sense whether pedal 2 present
  	if (pedal_2_sense) {
		pedal_2_old_val = pedal_2_val;
		pedal_2_val = 1023-analogRead(PEDAL_2_PIN);   //read in pedal 2 input
	}
	else {
		pedal_2_val = 0;
	}
	if ((pedal_2_val >= (pedal_2_old_val - pedal_margin)) && (pedal_2_val <= (pedal_2_old_val + pedal_margin))) {
		maskUpdate=true;  //request display update to show new states quickly
	}
	
	// Pedal modes


	/////////////////////////////////////////////////////////////////////////////////////////
		// STATE MACHINE //
	/////////////////////////////////////////////////////////////////////////////////////////

	//                0        1            2              3        4        5        6                7
	// enum UIStates {UI_idle, UI_home_amp, UI_home_patch, UI_edit, UI_save, UI_name, UI_init_act_set, UI_act_vol_sol, UI_patch, UI_ded_sol, UI_pat_vol_sol};

	if(button_state!=0) //A foot switch was pressed
	{
		TRACE_V_THR30IIPEDAL(Serial.println();)
		TRACE_V_THR30IIPEDAL(Serial.println("button_state: " + String(button_state));)
		TRACE_V_THR30IIPEDAL(Serial.println("Old UI_state: " + String(_uistate));)
		TRACE_V_THR30IIPEDAL(Serial.println("Old THR_Values.sendChangestoTHR: " + String(THR_Values.sendChangestoTHR));)
		TRACE_V_THR30IIPEDAL(Serial.println("Old stored_THR_Values.sendChangestoTHR: " + String(stored_THR_Values.sendChangestoTHR));)
		switch (_uistate)
		{
			case UI_idle:
			case UI_home_amp:
			case UI_home_patch:
				switch (button_state)
				{
					case 0:

					break;

					case 1:	// Submit pre-selected patch
						switch (_uistate)
						{
							case UI_home_amp: //no patch is active  (but patch key was pressed)
								if(npatches>0) //if at least one patch-settings file is available
								{ //store actual settings and activate presel patch
									TRACE_V_THR30IIPEDAL(Serial.println(F("Storing local settings..."));)
									stored_THR_Values = THR_Values;  //THR30II_Settings class is deep copyable
									patch_activate(presel_patch_id);
								}
							break;

							case UI_home_patch: //a patch is active already
								if(presel_patch_id != active_patch_id)  //a patch is active, but it is a different patch than pre-selected
								{
									//send presel patch
									Serial.printf("\n\rActivating pre-selected patch %d instead of the activated %d...\n\r",presel_patch_id, active_patch_id );
									patch_activate(presel_patch_id);
								}
								else  // return to local settings
								{
									patch_deactivate();	//restore actual settings
								}						
							break;
				
							default:	//UI_idle
								//do nothing
							break;
						}
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 2:	// Rotate col/amp/cab, depending on which amp_select_mode is active
						switch(amp_select_mode)
						{
							case COL:
								switch(THR_Values.col)
								{
									case CLASSIC:	THR_Values.col = BOUTIQUE;	break;
									case BOUTIQUE:	THR_Values.col = MODERN; 	break;
									case MODERN:	THR_Values.col = CLASSIC; 	break;
								}
								Serial.println("Amp collection switched to: "+String(THR_Values.col));
							break;

							case AMP:
								switch(THR_Values.amp)
								{
									case CLEAN:		THR_Values.amp = CRUNCH;	break;
									case CRUNCH:	THR_Values.amp = LEAD;		break;
									case LEAD:		THR_Values.amp = HI_GAIN;	break;
									case HI_GAIN:	THR_Values.amp = SPECIAL;	break;
									case SPECIAL:	THR_Values.amp = BASS;		break;
									case BASS:		THR_Values.amp = ACO;		break;
									case ACO:		THR_Values.amp = FLAT;		break;
									case FLAT:		THR_Values.amp = CLEAN;		break;
								}
								Serial.println("Amp type switched to: "+String(THR_Values.amp));
							break;

							case CAB:
								switch(THR_Values.cab)
								{
									case British_4x12:		THR_Values.cab = American_4x12;		break;
									case American_4x12:		THR_Values.cab = Brown_4x12;		break;
									case Brown_4x12:		THR_Values.cab = Vintage_4x12;		break;
									case Vintage_4x12:		THR_Values.cab = Fuel_4x12;			break;
									case Fuel_4x12:			THR_Values.cab = Juicy_4x12;		break;
									case Juicy_4x12:		THR_Values.cab = Mods_4x12;			break;
									case Mods_4x12:			THR_Values.cab = American_2x12;		break;
									case American_2x12:		THR_Values.cab = British_2x12;		break;
									case British_2x12:		THR_Values.cab = British_Blues;		break;
									case British_Blues:		THR_Values.cab = Boutique_2x12;		break;
									case Boutique_2x12:		THR_Values.cab = Yamaha_2x12;		break;
									case Yamaha_2x12:		THR_Values.cab = California_1x12;	break;
									case California_1x12:	THR_Values.cab = American_1x12;		break;
									case American_1x12:		THR_Values.cab = American_4x10;		break;
									case American_4x10:		THR_Values.cab = Boutique_1x12;		break;
									case Boutique_1x12:		THR_Values.cab = Bypass;			break;
									case Bypass:			THR_Values.cab = British_4x12;		break;
								}
								Serial.println("Cabinet switched to: "+String(THR_Values.cab));
							break;
						}
						THR_Values.createPatch();
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 3:
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 4:	// Decrement patch
						presel_patch_id--;	//decrement pre-selected patch-ID
						if (presel_patch_id < 1)	//detect wraparound
						{
							presel_patch_id = npatches;	//wrap back round to last patch in library
						}
						Serial.printf("Patch #%d pre-selected\n\r", presel_patch_id);
						//if immediate mode selected & new patch selected, send immediately
						if ((send_patch_now) && (presel_patch_id != active_patch_id))	
						{
							patch_activate(presel_patch_id);
						}
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;
						
					case 5:	// Increment patch
						presel_patch_id++;	//increment pre-selected patch-ID
						if (presel_patch_id > npatches)	//detect wraparound
						{
							presel_patch_id = 1;	//wrap back round to first patch in library
						}
						Serial.printf("Patch #%d pre-selected\n\r", presel_patch_id);
						//if immediate mode selected & new patch selected, send immediately
						if ((send_patch_now) && (presel_patch_id != active_patch_id))
						{
							patch_activate(presel_patch_id);
						}
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 6:
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 7:	// Toggle dynamics
						switch(dyn_mode)
						{
							case Comp:
								if(THR_Values.unit[COMPRESSOR])	//check compressor unit status
								{
									THR_Values.Switch_On_Off_Compressor_Unit(false); //if on, switch off
									Serial.println("Compressor unit switched off");
								}
								else
								{
									THR_Values.Switch_On_Off_Compressor_Unit(true);	//if off, switch on
									Serial.println("Compressor unit switched on");
								}
							break;

							case Boost:
								if(boost_activated)
								{
									undo_gain_boost();
									Serial.println("Gain boost deactivated");
								}
								else
								{
									do_gain_boost();
									Serial.println("Gain boost activated");

								}
							break;

							case Gate:
								if(THR_Values.unit[GATE])	//check gate unit status
								{
									THR_Values.Switch_On_Off_Gate_Unit(false); //if on, switch off
									Serial.println("Gate unit switched off");
								}
								else
								{
									THR_Values.Switch_On_Off_Gate_Unit(true);	//if off, switch on
									Serial.println("Gate unit switched on");
								}
							break;
						}
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 8:		// Toggle Effect
						if(THR_Values.unit[EFFECT])	//check effect unit status
						{
							THR_Values.Switch_On_Off_Effect_Unit(false); //if on, switch off
							Serial.println("Effect unit switched off");
						}
						else
						{
							THR_Values.Switch_On_Off_Effect_Unit(true);	//if off, switch on
							Serial.println("Effect unit switched on");
						}
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;
					
					case 9:		// Toggle Echo
						if(THR_Values.unit[ECHO])
						{
							THR_Values.Switch_On_Off_Echo_Unit(false);
							Serial.println("Echo unit switched off");
						}
						else
						{
							THR_Values.Switch_On_Off_Echo_Unit(true);
							Serial.println("Echo unit switched on");
						}
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 10:	// Toggle Reverb
						if(THR_Values.unit[REVERB])
						{
							THR_Values.Switch_On_Off_Reverb_Unit(false);
							Serial.println("Reverb unit switched off");
						}
						else
						{
							THR_Values.Switch_On_Off_Reverb_Unit(true);
							Serial.println("Reverb unit switched on");
						}
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 11:
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 12:	// Rotate amp select mode (COL -> AMP -> CAB -> )
						switch(amp_select_mode)
						{
							case COL:	amp_select_mode = AMP;	break;
							case AMP:	amp_select_mode = CAB; 	break;
							case CAB:	amp_select_mode = COL; 	break;
						}
						Serial.println("Amp Select Mode: "+String(amp_select_mode));
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 13:
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 14:	// Toggle patch selection mode (pre-select or immediate)
						send_patch_now = !send_patch_now;
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 15:	// Increment patch to first entry in next bank of 5
						presel_patch_id = ((presel_patch_id-1)/5 + 1) * 5 + 1;	//increment pre-selected patch-ID to first of next bank
						if (presel_patch_id > npatches)	//detect if last patch selected
						{
							presel_patch_id = 1;	//wrap back round to first patch in library
						}
						Serial.printf("Patch #%d pre-selected\n\r", presel_patch_id);
						//if immediate mode selected & new patch selected, send immediately
						if ((send_patch_now) && (presel_patch_id != active_patch_id))
						{
							patch_activate(presel_patch_id);
						}
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 16:
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 17:	// Rotate dynamics mode (Boost -> Comp -> Gate -> )
						switch(dyn_mode)
						{
							case Boost:	dyn_mode = Comp; 	break;
							case Comp:	dyn_mode = Gate;	break;
							case Gate:	dyn_mode = Boost; 	break;
						}
						Serial.println("Dynamics Mode: "+String(dyn_mode));
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 18:	// Rotate Effect Mode (Chorus > Flanger > Phaser > Tremolo > )
						switch(THR_Values.effecttype)
						{
							case CHORUS:
								THR_Values.EffectSelect(FLANGER);
								Serial.println("Effect unit switched from Chorus to Flanger");
							break;

							case FLANGER:
								THR_Values.EffectSelect(PHASER);
								Serial.println("Effect unit switched from Flanger to Phaser");
							break;

							case PHASER:
								THR_Values.EffectSelect(TREMOLO);
								Serial.println("Effect unit switched from Phaser to Tremolo");
							break;

							case TREMOLO:
								THR_Values.EffectSelect(CHORUS);
								Serial.println("Effect unit switched from Tremolo to Chorus");
							break;
						}
						THR_Values.createPatch();
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;
					
					case 19:	// Rotate Echo Mode (Tape Echo > Digital Delay > )
						switch(THR_Values.echotype)
						{
							case TAPE_ECHO:
								THR_Values.EchoSelect(DIGITAL_DELAY);
								Serial.println("Echo unit switched from Tape Echo to Digital Delay");
							break;

							case DIGITAL_DELAY:
								THR_Values.EchoSelect(TAPE_ECHO);
								Serial.println("Effect unit switched from Digital Delay to Tape Echo");
							break;
						}
						THR_Values.createPatch();
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;

					case 20:	// Rotate Reverb Mode (Spring > Room > Plate > Hall > )
						switch(THR_Values.reverbtype)
						{
							case SPRING:
								THR_Values.ReverbSelect(ROOM);
								Serial.println("Reverb unit switched from Spring to Room");
							break;

							case ROOM:
								THR_Values.ReverbSelect(PLATE);
								Serial.println("Reverb unit switched from Room to Plate");
							break;

							case PLATE:
								THR_Values.ReverbSelect(HALL);
								Serial.println("Reverb unit switched from Plate to Hall");
							break;

							case HALL:
								THR_Values.ReverbSelect(SPRING);
								Serial.println("Reverb unit switched from Hall to Spring");
							break;
						}
						THR_Values.createPatch();
						maskUpdate=true;  //request display update to show new states quickly
						button_state=0;  //remove flag, because it is handled
					break;
					
					default:

					break;
				}
			break;

			case UI_edit:

			break;

			case UI_save:

			break;

			case UI_name:

			break;

			default:

			break;
		}
		TRACE_V_THR30IIPEDAL(Serial.println("New UI_state: " + String(_uistate));)
		TRACE_V_THR30IIPEDAL(Serial.println("New THR_Values.sendChangestoTHR: " + String(THR_Values.sendChangestoTHR));)
		TRACE_V_THR30IIPEDAL(Serial.println("New stored_THR_Values.sendChangestoTHR: " + String(stored_THR_Values.sendChangestoTHR));)

	} //button_state!=0

}//end of loop()


void send_init()  //Try to activate THR30II MIDI 
{    
	//For debugging on old Arduino Due version: print USB-Endpoint information 
	// for(int i=0; i<5;i++)
	// {
	//  printf("\nEndpoint Index %0d: Dev.EP#%1lu, Pipe#%2lu, PktSize:%3lu, Direction:%4d\r\n"
	//  ,i, Midi.epInfo[i].deviceEpNum,Midi.epInfo[i].hostPipeNum,Midi.epInfo[i].maxPktSize,(byte)Midi.epInfo[i].direction );
	// }
	
	while(outqueue.item_count()>0) outqueue.dequeue(); //clear Queue (in case some message got stuck)
	
	Serial.println(F("\r\nOutque cleared.\r\n"));
	// PC to THR30II Message:
	//F0 7E 7F 06 01 F7  (First Thing PC sends to THR using "THR Remote" (prepares unlocking MIDI!) )
	//#S1
	//--------------------------------------------  Put outgoing  messages into the queue 
	//Universal Identity request
	outqueue.enqueue( Outmessage(SysExMessage( (const byte[6]) { 0xF0, 0x7E, 0x7F, 0x06, 0x01, 0xF7}, 6 ), 1, true, true) );  //Acknowledge means receiving 1st reply to Univ. Discovery, Answerded is receiving 2nd. Answer 
	
	//THR30II answers: F0 7E 7F 06 02 00 01 0C 24 00 02 00 63 00 1E 01 F7                   (1.30.0c)  (63 = 'c')
	// After Firmware-Update:   F0 7E 7F 06 02 00 01 0C 24 00 02 00 6B 00 1F 01 F7    (1.31.0k)  (6B = 'k')
	// After Firmware-Update:   F0 7E 7F 06 02 00 01 0C 24 00 02 00 61 00 28 01 F7    (1.40.0a)  (61 = 'a')
	TRACE_V_THR30IIPEDAL( Serial.println(F("Enqued Inquiry.\r\n")));

	//and afterwards a message containing Versions
	// like "L6ImageType:mainL6ImageVersion:1.3.0.0.c"

	/*  f0 00 01 0c 24 02 7e 7f 06 02 4c 36 49 6d 61 67 65 54 79 70 65 3a 6d 61 69 6e 00 4c 36 49 6d 61
		67 65 56 65 72 73 69 6f 6e 3a 31 2e 33 2e 30 2e 30 2e 63 00 f7
		(L6ImageType:mainL6ImageVersion:1.3.0.0.c)
	*/
	
	//#S2 invokes #R3 (ask Firmware-Version) For activating MIDI only necessary to get the right magic key for #S3/#S4  (not needed, if key is known)            
    outqueue.enqueue( Outmessage( SysExMessage( (const byte[29]) { 0xf0, 0x00, 0x01, 0x0c, 0x22, 0x02, 0x4d, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf7 },29), 2, false, true)); //answer will be the firmware version);
	TRACE_V_THR30IIPEDAL( Serial.println(F("Enqued Ask Firmware-Version.\r\n")));

    //#S3 + #S4 will follow, when answer to "ask Firmware-Version" comes in 
	//#S3 + #S4  will invoke #R4 (seems always the same) "MIDI activate" (works only after ID_Req!)														   

}//end of send_init

void do_gain_boost()
{
	std::copy( THR_Values.control.begin(),THR_Values.control.end(),THR_Values.control_store.begin());  //save volume and tone related settings

	THR_Values.sendChangestoTHR=true;
	THR_Values.SetControl(CTRL_GAIN, min(THR_Values.GetControl(CTRL_GAIN)+40, 100));
	THR_Values.sendChangestoTHR=false;
	boost_activated = true;
}

void undo_gain_boost()
{
	THR_Values.sendChangestoTHR=true;
	THR_Values.SetControl(CTRL_GAIN, THR_Values.control_store[CTRL_GAIN]);
	THR_Values.sendChangestoTHR=false;
	boost_activated = false;
}

// void do_volume_patch()   //increases Volume and/or Tone settings for SOLO to be louder than actual patch
// {
// 	std::copy( THR_Values.control.begin(),THR_Values.control.end(),THR_Values.control_store.begin());  //save volume and tone related settings

// 	if(THR_Values.GetControl(CTRL_MASTER)<100/1.333333)  //is there enough headroom to increase Master Volume by 33% ?
// 	{
// 		THR_Values.sendChangestoTHR=true;
// 		THR_Values.SetControl(CTRL_MASTER, THR_Values.GetControl(CTRL_MASTER)*1.333333);  //do it
// 		THR_Values.sendChangestoTHR=false;
// 	}
// 	else //try to increase volume by simultaneously increasing MID/TREBLE/BASS
// 	{
// 		THR_Values.sendChangestoTHR=true;
// 		double margin=(100.0-THR_Values.GetControl(CTRL_MASTER))/THR_Values.GetControl(CTRL_MASTER);  //maxi MASTER multiplier? (e.g. 0.17)
// 		THR_Values.SetControl(CTRL_MASTER,100); //use maximum MASTER
// 		double max_tone= std::max(std::max(THR_Values.GetControl(CTRL_BASS),THR_Values.GetControl(CTRL_MID)),THR_Values.GetControl(CTRL_TREBLE)) ; //highest value of the tone settings 
// 		double tone_margin=(100.0-max_tone)/max_tone;  //maxi equal TONE-Settings multiplier? (e.g. 0.28)
// 		if(tone_margin > 0.333333-margin)  //can we increase the TONE settings simultaneously to reach the missing MASTER-increasement?
// 		{
// 			THR_Values.SetControl(CTRL_BASS,THR_Values.GetControl(CTRL_BASS)*(1+0.333333-margin));
// 			THR_Values.SetControl(CTRL_MID, THR_Values.GetControl(CTRL_MID)*(1+0.333333-margin));
// 			THR_Values.SetControl(CTRL_TREBLE,THR_Values.GetControl(CTRL_TREBLE)*(1+0.333333-margin));
// 		} 
// 		else //increase as much as simultaneously possible (one tone setting reaches 100% this way)
// 		{
// 			THR_Values.SetControl(CTRL_BASS,THR_Values.GetControl(CTRL_BASS)*(1+tone_margin));
// 			THR_Values.SetControl(CTRL_MID, THR_Values.GetControl(CTRL_MID)*(1+tone_margin));
// 			THR_Values.SetControl(CTRL_TREBLE,THR_Values.GetControl(CTRL_TREBLE)*(1+tone_margin));
// 		}
// 		THR_Values.sendChangestoTHR=false;
// 	}
// }

// void undo_volume_patch()	//restore volume related settings
// {
// 	THR_Values.sendChangestoTHR=true;
// 	THR_Values.SetControl( CTRL_GAIN, 	THR_Values.control_store[CTRL_GAIN] );
// 	THR_Values.SetControl( CTRL_MASTER, THR_Values.control_store[CTRL_MASTER] );
// 	THR_Values.SetControl( CTRL_BASS,   THR_Values.control_store[CTRL_BASS] );
// 	THR_Values.SetControl( CTRL_MID,    THR_Values.control_store[CTRL_MID] );
// 	THR_Values.SetControl( CTRL_TREBLE, THR_Values.control_store[CTRL_TREBLE] );
// 	THR_Values.sendChangestoTHR=false;
// }

void patch_deactivate()
{
	//restore local settings
	THR_Values = stored_THR_Values;   //THR30II_Settings class is deep copyable!
	TRACE_THR30IIPEDAL(Serial.println(F("Patch_deactivate(): Restored local settings mirror..."));)
	
	//activate local settings in THRxxII again
	THR_Values.createPatch();  //Create a patch from all the local settings and send it completely via MIDI-patch-upload
	
	TRACE_THR30IIPEDAL(Serial.println(F("Patch_deactivate(): Local settings activated on THRII."));)
				
	//sending/activating local settings means returning to the initial actual settings
	_uistate = UI_home_amp;
}

void patch_activate(uint16_t pnr)  //check patchnumber and send patch as a SysEx message to THR30II
{
	if(pnr <= npatches)  //this patch-id is available
	{
		//activate the patch (overwrite either the actual patch or the local settings)
		TRACE_THR30IIPEDAL(Serial.printf("Patch_activate(): Activating patch #%d \n\r", pnr);)
		send_patch(pnr); //now send this patch as a SysEx message to THR30II 
		active_patch_id = pnr;
		boost_activated = false;
		_uistate = UI_home_patch;  //State "patch" reached
	} 
	else
	{
		TRACE_THR30IIPEDAL(Serial.printf("Patch_activate(): invalid pnr");)
	}
}

void send_patch(uint8_t patch_id)  //Send a patch from preset library to THRxxII
{ 
	DynamicJsonDocument djd(4096);  //contains accessible values from a JSON File (2048?)
	
	DeserializationError dse= deserializeJson(djd, patchesII[patch_id-1]);	//patchesII is zero-indexed
	
	if(dse==DeserializationError::Ok)
	{
		TRACE_THR30IIPEDAL( Serial.print(F("Send_patch(): Success-Deserializing. We fetched patch: "));)
		TRACE_THR30IIPEDAL( Serial.println( djd["data"]["meta"]["name"].as<const char*>() );)
		//Here we must copy values to local THRII-Settings and / or send them to THRII
		THR_Values.SetLoadedPatch(djd); //set all local fields from thrl6p-JSON-File 
	}
	else
	{
		 THR_Values.SetPatchName("error",-1);  //-1 = actual settings
		 TRACE_THR30IIPEDAL(Serial.println(F("Send_patch(): Error-Deserial"));)
	}
	
} //End of send_patch()

int THR30II_Settings::SetLoadedPatch(const DynamicJsonDocument &djd ) //invoke all settings from a loaded patch
{
	if (!MIDI_Activated)
	{
		TRACE_THR30IIPEDAL(Serial.println("Midi Sync is not ready!");)
		return -1;
	}

	sendChangestoTHR = false;  //We apply all the settings in one shot with a MIDI-Patch-Upload to THRII via "createPatch()" 
                               //and not each setting separately. So we use the setters only for our local fields!

	TRACE_THR30IIPEDAL(Serial.println(F("SetLoadedPatch(): Setting loaded patch..."));)

	std::map <String,uint16_t>  & glob = Constants::glo;
	
	//Patch name
	SetPatchName( djd["data"]["meta"]["name"].as<const char*>(), presel_patch_id);
	
	//unknown global parameter
	Tnid = djd["data"]["meta"]["tnid"].as<uint32_t>();

	//Unit states
	Switch_On_Off_Compressor_Unit(djd["data"]["tone"]["THRGroupFX1Compressor"]["@enabled"].as<bool>());
	Switch_On_Off_Gate_Unit(djd["data"]["tone"]["THRGroupGate"]["@enabled"].as<bool>());
	Switch_On_Off_Effect_Unit(djd["data"]["tone"]["THRGroupFX2Effect"]["@enabled"].as<bool>());
	Switch_On_Off_Echo_Unit(djd["data"]["tone"]["THRGroupFX3EffectEcho"]["@enabled"].as<bool>());
	Switch_On_Off_Reverb_Unit(djd["data"]["tone"]["THRGroupFX4EffectReverb"]["@enabled"].as<bool>());
	TRACE_V_THR30IIPEDAL(Serial.printf("Comp on/off: %d\n\r",unit[THR30II_UNITS::COMPRESSOR] );)
	TRACE_V_THR30IIPEDAL(Serial.printf("Gate on/off: %d\n\r",unit[THR30II_UNITS::GATE] );)
	TRACE_V_THR30IIPEDAL(Serial.printf("Eff on/off: %d\n\r",unit[THR30II_UNITS::EFFECT] );)
	TRACE_V_THR30IIPEDAL(Serial.printf("Echo on/off: %d\n\r",unit[THR30II_UNITS::ECHO] );)
	TRACE_V_THR30IIPEDAL(Serial.printf("Rev on/off: %d\n\r",unit[THR30II_UNITS::REVERB] );)

	//global parameter tempo
	ParTempo = djd["data"]["tone"]["global"]["THRPresetParamTempo"].as<uint32_t>();



	//FX1 Compressor (no different types)
	CompressorSetting(CO_SUSTAIN,  djd["data"]["tone"]["THRGroupFX1Compressor"]["Sustain"].as<double>() * 100);
	CompressorSetting(CO_LEVEL, djd["data"]["tone"]["THRGroupFX1Compressor"]["Level"].as<double>() * 100);
	//CompressorSetting(THR30II_COMP_SET.MIX, pat.data.tone.THRGroupFX1Compressor. * 100);  //No Mix-Parameter in Patch-Files
	


	//Amp / Collection
	uint16_t key = glob[djd["data"]["tone"]["THRGroupAmp"]["@asset"].as<const char*>() ];  //e.g. "THR10C_DC30" => 0x88
	setColAmp(key);
	TRACE_V_THR30IIPEDAL(Serial.printf("Amp: %x\n\r",key );)

	//Main Amp Controls
	SetControl(CTRL_GAIN,  djd["data"]["tone"]["THRGroupAmp"]["Drive"].as<double>() * 100);
	SetControl(CTRL_MASTER,  djd["data"]["tone"]["THRGroupAmp"]["Master"].as<double>() * 100);
	SetControl(CTRL_BASS, djd["data"]["tone"]["THRGroupAmp"]["Bass"].as<double>() * 100);
	SetControl(CTRL_MID,  djd["data"]["tone"]["THRGroupAmp"]["Mid"].as<double>() * 100);
	SetControl(CTRL_TREBLE,  djd["data"]["tone"]["THRGroupAmp"]["Treble"].as<double>() * 100);
	TRACE_V_THR30IIPEDAL(Serial.println( "Controls: G, M , B , Mi , T" );)
	TRACE_V_THR30IIPEDAL(Serial.println(control[0]);)
	TRACE_V_THR30IIPEDAL(Serial.println(control[1]);)
	TRACE_V_THR30IIPEDAL(Serial.println(control[2]);)
	TRACE_V_THR30IIPEDAL(Serial.println(control[3]);)
	TRACE_V_THR30IIPEDAL(Serial.println(control[4]);)

	//Cab Simulation             
	uint16_t val = djd["data"]["tone"]["THRGroupCab"]["SpkSimType"].as<uint16_t>(); //e.g. 10 
	SetCab((THR30II_CAB)val);  //e.g. 10 =>  Boutique_2x12
	TRACE_V_THR30IIPEDAL(Serial.printf("Cab: %x\n\r",val );)



	//Noise Gate
	double v = 100.0 - 100.0 / 96 * -  djd["data"]["tone"]["THRGroupGate"]["Thresh"].as<double>() ;   //change range from [-96dB ... 0dB] to [0...100]
	GateSetting(GA_THRESHOLD, v);
	GateSetting(GA_DECAY, djd["data"]["tone"]["THRGroupGate"]["Decay"].as<double>() * 100);
	TRACE_V_THR30IIPEDAL(Serial.println( "Gate Thresh:" );)
	TRACE_V_THR30IIPEDAL(Serial.println( v );)



	//FX2 Effect Settings
	String type (djd["data"]["tone"]["THRGroupFX2Effect"]["@asset"].as<const char*>());
	TRACE_V_THR30IIPEDAL(Serial.printf("EffType: %s\n\r",type.c_str() );)
	
	if(type == "StereoSquareChorus")
	{
		EffectSelect(CHORUS);
	}
	else if (type == "L6Flanger")
	{
		EffectSelect(FLANGER);
	}
	else if (type == "Phaser")
	{
		EffectSelect(PHASER);
	}
	else if (type == "BiasTremolo")
	{
		EffectSelect(TREMOLO);
	}
	EffectSetting(CHORUS, glob["Freq"], djd["data"]["tone"]["THRGroupFX2Effect"]["StereoSquareChorus"]["Freq"].as<double>()* 100);
	EffectSetting(CHORUS, glob["Depth"], djd["data"]["tone"]["THRGroupFX2Effect"]["StereoSquareChorus"]["Depth"].as<double>()*100);
	EffectSetting(CHORUS, glob["Pre"], djd["data"]["tone"]["THRGroupFX2Effect"]["StereoSquareChorus"]["Pre"].as<double>()* 100);
	EffectSetting(CHORUS, glob["Feedback"], djd["data"]["tone"]["THRGroupFX2Effect"]["StereoSquareChorus"]["Feedback"].as<double>()* 100);
	EffectSetting(CHORUS, glob["FX2Mix"], djd["data"]["tone"]["THRGroupFX2Effect"]["StereoSquareChorus"]["@wetDry"].as<double>()* 100);
	
	EffectSetting(FLANGER, glob["Freq"], djd["data"]["tone"]["THRGroupFX2Effect"]["L6Flanger"]["Freq"].as<double>() * 100);
	EffectSetting(FLANGER, glob["Depth"] ,djd["data"]["tone"]["THRGroupFX2Effect"]["L6Flanger"]["Depth"].as<double>() * 100);
	EffectSetting(FLANGER, glob["FX2Mix"], djd["data"]["tone"]["THRGroupFX2Effect"]["L6Flanger"]["@wetDry"].as<double>() * 100);
	
	EffectSetting(PHASER, glob["Speed"], djd["data"]["tone"]["THRGroupFX2Effect"]["Phaser"]["Speed"].as<double>() * 100);
	EffectSetting(PHASER, glob["Feedback"], djd["data"]["tone"]["THRGroupFX2Effect"]["Phaser"]["Feedback"].as<double>() * 100);
	EffectSetting(PHASER, glob["FX2Mix"], djd["data"]["tone"]["THRGroupFX2Effect"]["Phaser"]["@wetDry"].as<double>() * 100);
	
	EffectSetting(TREMOLO, glob["Speed"],  djd["data"]["tone"]["THRGroupFX2Effect"]["BiasTremolo"]["Speed"].as<double>()*100);
	EffectSetting(TREMOLO, glob["Depth"],  djd["data"]["tone"]["THRGroupFX2Effect"]["BiasTremolo"]["Depth"].as<double>()*100);
	EffectSetting(TREMOLO, glob["FX2Mix"], djd["data"]["tone"]["THRGroupFX2Effect"]["BiasTremolo"]["@wetDry"].as<double>()*100);



	//FX3 Echo Settings
	type=djd["data"]["tone"]["THRGroupFX3EffectEcho"]["@asset"].as<const char*>() ;
	TRACE_V_THR30IIPEDAL(Serial.printf("EchoType: %s\n\r",type.c_str() );)
	if (type == "TapeEcho")
	{
		EchoSelect(TAPE_ECHO);
	}
	else if (type == "L6DigitalDelay")
	{
		EchoSelect(DIGITAL_DELAY);
	}
	EchoSetting(TAPE_ECHO, glob["Time"], djd["data"]["tone"]["THRGroupFX3EffectEcho"]["TapeEcho"]["Time"].as<double>() * 100);
	EchoSetting(TAPE_ECHO, glob["Feedback"] , djd["data"]["tone"]["THRGroupFX3EffectEcho"]["TapeEcho"]["Feedback"].as<double>() * 100);
	EchoSetting(TAPE_ECHO, glob["Bass"], djd["data"]["tone"]["THRGroupFX3EffectEcho"]["TapeEcho"]["Bass"].as<double>() * 100);
	EchoSetting(TAPE_ECHO, glob["Treble"], djd["data"]["tone"]["THRGroupFX3EffectEcho"]["TapeEcho"]["Treble"].as<double>() * 100);
	EchoSetting(TAPE_ECHO, glob["FX3Mix"], djd["data"]["tone"]["THRGroupFX3EffectEcho"]["TapeEcho"]["@wetDry"].as<double>() * 100);
	
	EchoSetting(DIGITAL_DELAY, glob["Time"], djd["data"]["tone"]["THRGroupFX3EffectEcho"]["L6DigitalDelay"]["Time"].as<double>() * 100);
	EchoSetting(DIGITAL_DELAY, glob["Feedback"], djd["data"]["tone"]["THRGroupFX3EffectEcho"]["L6DigitalDelay"]["Feedback"].as<double>() * 100);
	EchoSetting(DIGITAL_DELAY, glob["Bass"], djd["data"]["tone"]["THRGroupFX3EffectEcho"]["L6DigitalDelay"]["Bass"].as<double>() * 100);
	EchoSetting(DIGITAL_DELAY, glob["Treble"], djd["data"]["tone"]["THRGroupFX3EffectEcho"]["L6DigitalDelay"]["Treble"].as<double>() * 100);
	EchoSetting(DIGITAL_DELAY, glob["FX3Mix"], djd["data"]["tone"]["THRGroupFX3EffectEcho"]["L6DigitalDelay"]["@wetDry"].as<double>() * 100);
	


	//FX4 Reverb Settings
	type=djd["data"]["tone"]["THRGroupFX4EffectReverb"]["@asset"].as<const char*>();
	TRACE_V_THR30IIPEDAL(Serial.printf("RevType: %s\n\r",type.c_str() );)
	if ( type ==  "StandardSpring")
	{
		ReverbSelect(SPRING);
	}
	else if (type=="SmallRoom1")
	{
		ReverbSelect(ROOM);
	}
	else if (type== "LargePlate1")
	{
		ReverbSelect(PLATE);
	}
	else if (type == "ReallyLargeHall")
	{
		ReverbSelect(HALL);
	}
	ReverbSetting(SPRING, glob["Time"], djd["data"]["tone"]["THRGroupFX4EffectReverb"]["StandardSpring"]["Time"].as<double>() * 100);
	ReverbSetting(SPRING, glob["Tone"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["StandardSpring"]["Tone"].as<double>() * 100);
	ReverbSetting(SPRING, glob["FX4WetSend"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["StandardSpring"]["@wetDry"].as<double>() * 100);

	ReverbSetting(ROOM, glob["Decay"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["SmallRoom1"]["Decay"].as<double>() * 100);
	ReverbSetting(ROOM, glob["Tone"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["SmallRoom1"]["Tone"].as<double>() * 100);
	ReverbSetting(ROOM, glob["PreDelay"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["SmallRoom1"]["PreDelay"].as<double>() * 100);
	ReverbSetting(ROOM, glob["FX4WetSend"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["SmallRoom1"]["@wetDry"].as<double>() * 100);

	ReverbSetting(PLATE, glob["Decay"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["LargePlate1"]["Decay"].as<double>() * 100);
	ReverbSetting(PLATE, glob["PreDelay"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["LargePlate1"]["PreDelay"].as<double>() * 100);
	ReverbSetting(PLATE, glob["Tone"],   djd["data"]["tone"]["THRGroupFX4EffectReverb"]["LargePlate1"]["Tone"].as<double>() * 100);
	ReverbSetting(PLATE, glob["FX4WetSend"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["LargePlate1"]["@wetDry"].as<double>() * 100);

	ReverbSetting(HALL, glob["Decay"], djd["data"]["tone"]["THRGroupFX4EffectReverb"]["ReallyLargeHall"]["Decay"].as<double>() * 100);
	ReverbSetting(HALL, glob["PreDelay"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["ReallyLargeHall"]["PreDelay"].as<double>() * 100);
	ReverbSetting(HALL, glob["Tone"],   djd["data"]["tone"]["THRGroupFX4EffectReverb"]["ReallyLargeHall"]["Tone"].as<double>() * 100);
	ReverbSetting(HALL, glob["FX4WetSend"],  djd["data"]["tone"]["THRGroupFX4EffectReverb"]["ReallyLargeHall"]["@wetDry"].as<double>() * 100);



	createPatch();  //send all settings as a patch dump SysEx to THRII
	
	TRACE_THR30IIPEDAL(Serial.println(F("SetLoadedPatch(): Done setting."));)

	return 0;
}

THR30II_Settings::States THR30II_Settings::_state = THR30II_Settings::States::St_idle;  //the actual state of the state engine for creating a patch

void THR30II_Settings::createPatch() //fill send buffer with actual settings, creating a valid SysEx for sending to THR30II
{
	//1.) Make the data buffer (structure and values) store the length.
	//    Add 12 to get the 2nd length-field for the header. Add 8 to get the 1st length field for the header.
	//2.) Cut it to slices with 0x0d02 (210 dez.) bytes (gets the payload of the frames)
	//    These frames will thus not have incomplete 8/7 groups after bitbucketing but total frame length is 253 (just below the possible 255 bytes).
	//    The remaining bytes will build the last frame (perhaps with last 8/7 group inclomplete)
	//    Store the length of the last slice (all others are fixed 210 bytes  long.)
	//3.) Create the header:
	//    SysExStart:  f0 00 01 0c 22 02 4d (always the same)
	//                 01   (memory command, not settings command)
	//                 Get actual counter for memory frames and append it here
	//                 00 01 0b  ("same frame counter" and payload size  for the header)
	//                 hang following 4-Byte values together:
	//
	//                 0d 00 00 00   (Opcode for memory write/send)
	//                 (len +8+12)   (1st length field = total length= data length + 3 values + type field/netto length )
	//                 FF FF FF FF   (user patch number to overwrite/ 0xFFFFFFFF for actual patch)
	//                 (len   +12)   (2nd length field = netto length= data length + the following 3 values)
	//                 00 00 00 00   (always the same, kind of opcode?)
	//                 01 00 00 00   (always the same, kind of opcode?)
	//                 00 00 00 00   (always the same, kind of opcode?)
	//
	//                 Bitbucket this group (gives no incomplete 8/7 group!)
	//                 and append it to the header
	//                 finish header with 0xF7
	//                 Header frame may already be sent out!
	//4.)              For each slice:
	//                 Bitbucket encode the slice
	//                 Build frame:
	//                 f0 00 01 0c 22 02 4d(always the same)
	//                 01   (memory command, not settings command)
	//                 Get actual counter for memory frames and append it here
	//                 append same frame counter (is the slice counter) as a byte
	//                 0d 01  (except for last frame, where we use it's stored length before bitbucketing)
	//                 bitbucketed data for this slice
	//                 0xF7
	//                 Send it out.
	TRACE_V_THR30IIPEDAL(Serial.println(F("Create_patch(): "));)

	std::array<byte,2000> dat;
	auto datlast =dat.begin();  //iterator to last element (starts on begin! )
	std::array<byte,300> sb;
	auto sblast = sb.begin();  //iterator to last element (starts on begin! )
	std::array<byte,300> sb2;
	auto sb2last = sb2.begin();  //iterator to last element (starts on begin! )

	std::map<String,uint16_t> &glob = Constants::glo;
	
	std::array<byte,4> toInsert4;
	std::array<byte,6> toInsert6;

	#define datback(x)  for(const byte &b : x ) { *datlast++ = b; };
	
	//Macro for converting a 32-Bit value to a 4-byte array<byte,4> and append it to "dat"
	#define conbyt4(x) toInsert4={ (byte)(x), (byte) ((x)>>8), (byte)((x)>>16) , (byte) ((x)>>24) };  datlast= std::copy( std::begin(toInsert4), std::end(toInsert4), datlast ); 
	//Macro for appending a 16-Bit value to "dat"
	#define conbyt2(x) *datlast++=(byte)(x); *datlast++= (byte)((x)>>8); 

	//1.) Make the data buffer (structure and values)

	datback( tokens["StructOpen"] );
	//Meta
	datback(tokens["Meta"] ) ;          
	datback(tokens["TokenMeta"] );
	toInsert6= { 0x00, 0x00, 0x00, 0x00, 0x04, 0x00 };
	datlast=std::copy( std::begin(toInsert6),std::end(toInsert6),datlast) ;    //number 0x0000, type 0x00040000 (String)
	
	conbyt4(  patchNames[0].length() + 1 ) // Length of patchname (incl. '\0')
		//32Bit-Value (little endian; low byte first) 
	
	std::string nam =patchNames[0u].c_str();
	datback( nam ); //copy the patchName  //!!ZWEZWE!! mind UTF-8 ?
	*datlast++='\0'; //append  '\0' to the patchname
	
	toInsert6={0x01, 0x00, 0x00, 0x00, 0x02, 0x00 };
	datlast=std::copy(std::begin(toInsert6), std::end(toInsert6),datlast );      //number 0x0001, type 0x00020000 (int)
	
	conbyt4(Tnid);     //32Bit-Value (little endian; low byte first) Tnid

	toInsert6={ 0x02, 0x00, 0x00, 0x00, 0x02, 0x00 };    //number 0x0002, type 0x00020000 (int)
	datlast=std::copy(std::begin(toInsert6), std::end(toInsert6),datlast ); 
	conbyt4(UnknownGlobal);	    //32Bit-Value (little endian; low byte first) Unknown Global

	toInsert6={ 0x03, 0x00, 0x00, 0x00, 0x03, 0x00 }; //number 0x0003, type 0x00030000 (int)
	datlast=std::copy(std::begin(toInsert6), std::end(toInsert6),datlast );
	conbyt4(ParTempo);  //32Bit-Value (little endian; low byte first) ParTempo (min=110 =0x00000000)
	
	datback(tokens["StructClose"]);
	//Data           
	datback(tokens["StructOpen"]);
	datback(tokens["Data"]);
	datback(tokens["TokenData"]);

	//unit GuitarProc
	datback(tokens["UnitOpen"]);
	conbyt2(glob["GuitarProc"]);
	datback(tokens["UnitType"]);
	datback(tokens["PseudoVal"]);
	conbyt2(glob["Y2GuitarFlow"]);
	datback(tokens["ParCount"]);
	datback(tokens["PseudoType"]);
	conbyt4(11u); //32-Bit value  number of parameters (here: 11)
	//1601 = FX1EnableState  (CompOn)
	conbyt2(glob["FX1EnableState"]);
	conbyt4(0x00010000u);//Type binary
	conbyt4( unit[COMPRESSOR] ? 0x01u : 0x00u );
	//1901 = FX2EnableState  (EffectOn)
	conbyt2(glob["FX2EnableState"]);
	conbyt4(0x00010000u);//Type binary
	conbyt4( unit[EFFECT] ? 0x01u : 0x00u );
	//1801 = FX2MixState     (Eff.Mix)
	conbyt2(glob["FX2MixState"]);				
	conbyt4(0x00030000u);//Type int
	
	conbyt4(ValToNumber(effect_setting[PHASER][PH_MIX])); //MIX is independent from the effect type!
	//1C01 = FX3EnableState  (EchoOn)
	conbyt2(glob["FX3EnableState"]);
	conbyt4(0x00010000u);//Type binary
	conbyt4(unit[ECHO] ? 0x01u : 0x00u );         
	//1B01 = FX3MixState     (EchoMix)
	conbyt2(glob["FX3MixState"]);
	conbyt4(0x00030000u);//Type int
	conbyt4(ValToNumber(echo_setting[TAPE_ECHO][TA_MIX])); //MIX is independent from the echo type!
	
	//1F01 = FX4EnableState  (RevOn)
	conbyt2(glob["FX4EnableState"]);		
	conbyt4(0x00010000u);//Type binary
	
	conbyt4(unit[REVERB] ? 0x01u : 0x00u );
	//2601 = FX4WetSendState (RevMix)
	conbyt2(glob["FX4WetSendState"]);
	conbyt4(0x00030000u);//Type int
	conbyt4(ValToNumber(reverb_setting[SPRING][SP_MIX]) );  //MIX is independent from the effect type!
	//2101 = GateEnableState (GateOn)
	conbyt2(glob["GateEnableState"]);
	conbyt4(0x00010000u);//Type binary
	conbyt4(unit[GATE] ? 0x01u : 0x00u );
	//2401 = SpkSimTypeState (Cabinet)
	conbyt2(glob["SpkSimTypeState"]);
	conbyt4(0x00020000u);//Type enum
	
	conbyt4((uint32_t)cab);  //Cabinet is a value from 0x00 to 0x10
	
	//F800 = DecayState      (GateDecay)
	conbyt2(glob["DecayState"]);
	conbyt4(0x00030000u);//Type int
	conbyt4(ValToNumber(gate_setting[GA_DECAY]) );
	//2501 = ThreshState     (GateThreshold)
	conbyt2(glob["ThreshState"]);			     
	conbyt4(0x00030000u);//Type int
	
	conbyt4(ValToNumber_Threshold(gate_setting[GA_THRESHOLD]) );
	
	//unit Compressor
	datback(tokens["UnitOpen"]);
	conbyt2(glob["FX1"]);						
	datback(tokens["UnitType"]);		
	datback(tokens["PseudoVal"]);		
	conbyt2(glob["RedComp"]);				
	datback(tokens["ParCount"]);			
	datback(tokens["PseudoType"]);			
	conbyt4(2u);  //32-Bit value  number of parameters (here: 2)
	
	//BF00 = Compressor Level(LevelState)
	conbyt2(glob["LevelState"]);	
	conbyt4(0x00030000u);//Type int
	conbyt4(ValToNumber(compressor_setting[CO_LEVEL]) );	
	//BE00 = Compressor Sustain(SustainState)
	conbyt2(glob["SustainState"]);						
	conbyt4(0x00030000u);//Type int
	conbyt4(ValToNumber(compressor_setting[CO_SUSTAIN]) );
	datback(tokens["UnitClose"] ); 	   //Close Compressor Unit
	
	//unit AMP (0x0A01)
	datback(tokens["UnitOpen"] );	
	conbyt2(glob["Amp"]);
	datback(tokens["UnitType"] );	
	datback(tokens["PseudoVal"] );	
	conbyt2(THR30IIAmpKeys[col_amp(col,amp)]);	
	datback(tokens["ParCount"] );	
	datback(tokens["PseudoType"] );			
	conbyt4(5u);  //32-Bit value  number of parameters (here: 5)
	
	//4f 00 CTRL BASS (BassState)
	conbyt2(glob["BassState"]);
	conbyt4(0x00030000u);//Type int
	conbyt4(ValToNumber(control[CTRL_BASS]) );
	//52 00 GAIN (DriveState)
	conbyt2(glob["DriveState"]);
	conbyt4(0x00030000u);//Type int
	conbyt4(ValToNumber(control[CTRL_GAIN]) );
	//53 00 MASTER (MasterState)
	conbyt2(glob["MasterState"]);
	conbyt4(0x00030000u);//Type int
	conbyt4(ValToNumber(control[CTRL_MASTER]) );
	//50 00 MID (MidState)
	conbyt2(glob["MidState"]);
	conbyt4(0x00030000u);//Type int
	conbyt4(ValToNumber(control[CTRL_MID]) );
	//51 00 CTRL TREBLE (TrebleState)
	conbyt2(glob["TrebleState"]);
	conbyt4(0x00030000u);//Type int
	conbyt4 (ValToNumber(control[CTRL_TREBLE]) );
	datback(tokens["UnitClose"] );  //close AMP Unit
	//unit EFFECT (FX2) (0x0E01)
	datback(tokens["UnitOpen"]) ;
	conbyt2(glob["FX2"]);
	datback(tokens["UnitType"] );	
	datback(tokens["PseudoVal"] );	
	conbyt2(THR30II_EFF_TYPES_VALS[effecttype].key );  //EffectType as a key value
	
	datback(tokens["ParCount"]);    
	datback(tokens["PseudoType"]);
		//Number and kind of parameters depend on the selcted effect type            
	switch (effecttype)
	{
		case PHASER:
			conbyt4(2u);  //32-Bit value  number of parameters (here: 2)
			conbyt2(glob["FeedbackState"]);      
			conbyt4(0x00030000u);//Type int
			
			conbyt4(ValToNumber(effect_setting[effecttype][PH_FEEDBACK]));
			conbyt2(glob["FreqState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(effect_setting[effecttype][PH_SPEED]) );
			break;
		case TREMOLO:
			conbyt4(2u);  //32-Bit value  number of parameters (here: 2)
			conbyt2(glob["DepthState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(effect_setting[effecttype][TR_DEPTH]) );
			conbyt2(glob["SpeedState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(effect_setting[effecttype][TR_SPEED]) );
			break;
		case FLANGER:
			conbyt4(2u);  //32-Bit value  number of parameters (here: 2)
			conbyt2(glob["DepthState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(effect_setting[effecttype][FL_DEPTH]));
			conbyt2(glob["FreqState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(effect_setting[effecttype][FL_SPEED]));
			break;
		case CHORUS:
			conbyt4(4u);  //32-Bit value  number of parameters (here: 4)
			conbyt2(glob["DepthState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(effect_setting[effecttype][CH_DEPTH]));
			conbyt2(glob["FeedbackState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(effect_setting[effecttype][CH_FEEDBACK]));
			conbyt2(glob["FreqState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(effect_setting[effecttype][CH_SPEED]));
			conbyt2(glob["PreState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(effect_setting[effecttype][CH_PREDELAY]));
			break;
	}
	datback(tokens["UnitClose"]); //close EFFECT Unit

	//0f 01 Unit ECHO (FX3)
	datback(tokens["UnitOpen"]);
	conbyt2(glob["FX3"]);
	datback(tokens["UnitType"]);
	datback(tokens["PseudoVal"]);
	conbyt2(THR30II_ECHO_TYPES_VALS[echotype].key );   //EchoType as a key value (variable since 1.40.0a)
	//dat.AddRange(BitConverter.GetBytes(glob["TapeEcho"]));  //before 1.40.0a "TapeEcho" was fixed type for Unit "Echo"
	datback(tokens["ParCount"]);
	datback(tokens["PseudoType"]);
	
	//Number and kind of parameters could depend on the selected echo type (in fact it does not)           
	switch (echotype)
	{
		case TAPE_ECHO:
			conbyt4(4u);  //32-Bit value  number of parameters (here: 4)
			conbyt2(glob["BassState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(echo_setting[echotype][TA_BASS]));
			conbyt2(glob["FeedbackState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(echo_setting[echotype][TA_FEEDBACK]));
			conbyt2(glob["TimeState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(echo_setting[echotype][TA_TIME]));
			conbyt2(glob["TrebleState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(echo_setting[echotype][TA_TREBLE]));
			break;
		case DIGITAL_DELAY:
			conbyt4(4u);  //32-Bit value  number of parameters (here: 4)
			conbyt2(glob["BassState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(echo_setting[echotype][DD_BASS]));
			conbyt2(glob["FeedbackState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(echo_setting[echotype][DD_FEEDBACK]));
			conbyt2(glob["TimeState"]);			
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(echo_setting[echotype][DD_TIME]));
			conbyt2(glob["TrebleState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(echo_setting[echotype][DD_TREBLE]));
			break;
	}
	datback(tokens["UnitClose"]);	//close ECHO Unit
	
	//12 01 Unit REVERB
	datback(tokens["UnitOpen"]);
	conbyt2(glob["FX4"]);
	datback(tokens["UnitType"]);
	datback(tokens["PseudoVal"]);
	conbyt2(THR30II_REV_TYPES_VALS[reverbtype].key); 	
	datback(tokens["ParCount"]);
	datback(tokens["PseudoType"]);
	//Number and kind of parameters depend on the selected reverb type            
	switch (reverbtype)
	{
		case SPRING:
			conbyt4(2u);  //32-Bit value  number of parameters (here: 2)
			conbyt2(glob["TimeState"]);	
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][SP_REVERB]));
			conbyt2(glob["ToneState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][SP_TONE]));
			break;
		case PLATE:
			conbyt4(3u);  //32-Bit value  number of parameters (here: 3)
			conbyt2(glob["DecayState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][PL_DECAY]));
			conbyt2(glob["PreDelayState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][PL_PREDELAY]));
			conbyt2(glob["ToneState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][PL_TONE]));
			break;
		case HALL:
			conbyt4(3u);  //32-Bit value  number of parameters (here: 3)
			conbyt2(glob["DecayState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][HA_DECAY]) );
			conbyt2(glob["PreDelayState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][HA_PREDELAY]));
			conbyt2(glob["ToneState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][HA_TONE]));
			break;
		case ROOM:
			conbyt4(3u);  //32-Bit value  number of parameters (here: 3)
			conbyt2(glob["DecayState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][RO_DECAY]));
			conbyt2(glob["PreDelayState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][RO_PREDELAY]));
			conbyt2(glob["ToneState"]);
			conbyt4(0x00030000u);//Type int
			conbyt4(ValToNumber(reverb_setting[reverbtype][RO_TONE]));
			break;
	}
	datback(tokens["UnitClose"]);   //close REVERB Unit
	
	datback(tokens["UnitClose"]);   //close Unit GuitarProcessor
	
	datback(tokens["StructClose"]);  //close Structure Data
	
	//print whole buffer as a chain of HEX
	//TRACE_V_THR30IIPEDAL( Serial.printf("\n\rRaw frame before slicing/enbucketing:\n\r"); hexdump(dat,datlast-dat.begin());)

	//    Now store the length.
	//    Add 12 to get the 2nd length-field for the header. Add 8 to get the 1st length field for the header.
	uint32_t datalen = (uint32_t) (datlast-dat.begin());
	
	//2.) cut to slices
	uint32_t lengthfield2 = datalen + 12;
	uint32_t lengthfield1 = datalen + 20;

	int numslices = (int)datalen / 210; //number of 210-byte slices
	int lastlen = (int)datalen % 210; //data left for last frame containing the rest
	TRACE_V_THR30IIPEDAL(Serial.print("Datalen: "); Serial.println(datalen);)
	TRACE_V_THR30IIPEDAL(Serial.print("Number of slices: "); Serial.println(numslices);)
	TRACE_V_THR30IIPEDAL(Serial.print("Length of last slice: "); Serial.println(lastlen);)

	//3.) Create the header:
	//    SysExStart:  f0 00 01 0c 22 02 4d (always the same)
	//                 01   (memory command, not settings command)
	//                 put following 4-Byte values together:
	//
	//                 0d 00 00 00   (Opcode for memory write/send)
	
	//Macro for converting a 32-Bit value to a 4-byte vector<byte,4> and append it to "sb"
	#define conbyt4s(x) toInsert4={ (byte)(x), (byte) ((x)>>8), (byte)((x)>>16) , (byte) ((x)>>24) };  sblast= std::copy( std::begin(toInsert4), std::end(toInsert4), sblast ); 
	//sblast starts at sb.begin()
	conbyt4s(0x0du);
	//                 (len +8+12)   (1st length field = total length= data length + 3 values + type field/netto length )
	conbyt4s(lengthfield1);
	//                 FF FF FF FF   (number of user patch to overwrite / 0xFFFFFFFF for actual patch)
	conbyt4s(0xFFFFFFFFu);
	//                 (len   +12)   (2nd length field = netto length= data length + the following 3 values)
	conbyt4s(lengthfield2);
	//                 00 00 00 00   (always the same, kind of opcode?)
	conbyt4s(0x0u);
	//                 01 00 00 00   (always the same, kind of opcode?)
	conbyt4s(0x1u);
	//                 00 00 00 00   (always the same, kind of opcode?)
	conbyt4s(0x0u);
	//
	//Bitbucket this group (gives no incomplete 8/7 group!)
	sb2last = Enbucket(sb2,sb,sblast); 
	//                 and append it to the header
	//                 finish header with 0xF7
	//                 Header frame may already be sent out!
	byte memframecnt = 0x06;
	//                 Get actual counter for memory frames and append it here

	//                 00 01 0b  ("same frame counter" and payload size  for the header)
	sblast=sb.begin(); //start the buffer to send
	
	std::array<byte,12> toInsert;
	toInsert=std::array<byte,12>({ 0xf0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, memframecnt++, 0x00, 0x01, 0x0b });
	sblast= std::copy( toInsert.begin(),toInsert.end(),sb.begin());
	sblast= std::copy( sb2.begin(), sb2last, sblast);

	*sblast++=0xF7; //SysEx end demarkation

	SysExMessage m ( sb.data(),sblast -sb.begin() );
	Outmessage om (m, 100, false, false);  //no Ack, no answer for the header 
	outqueue.enqueue(om);  //send header to THRxxII

	//4.)              For each slice:
	//                 Bitbucket encode the slice
	
	for (int i = 0; i < numslices; i++)
	{
		std::array<byte,210> slice;
		std::copy(dat.begin() + i*210, dat.begin() + i*210 +210, slice.begin() ) ;
		sb2last=Enbucket(sb2, slice, slice.end());
		sblast=sb.begin(); //start new buffer to send
		//toInsert=  { 0xF0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, memframecnt++, (byte)(i % 128), 0x0d, 0x01 };
		toInsert=  { 0xF0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, memframecnt, (byte)(i % 128), 0x0d, 0x01 };  //memframecount is not incremented inside sent patches
		sblast=std::copy(toInsert.begin(), toInsert.end(), sblast );
		sblast=std::copy( sb2.begin(),sb2last, sblast );
		*sblast++=0xF7;
		//print whole buffer as a chain of HEX
		// TRACE_V_THR30IIPEDAL(Serial.printf("\n\rFrame %d to send in \"createpatch\":\n\r",i);
		// 		hexdump(sb,sblast-sb.begin());
		// )
		m = SysExMessage(sb.data(), sblast -sb.begin() );
		om = Outmessage(m, (uint16_t)(101 + i), false, false);  //no Ack, for all the other slices 
		outqueue.enqueue(om);  //send slice to THRxxII
	}

	if (lastlen > 0) //last slice (could be the first, if it is the only one)
	{
		std::array<byte,210> slice;  //here 210 is a maximum size, not the true element count
		byte *slicelast=slice.begin();
		slicelast=std::copy(dat.begin() + numslices * 210, dat.begin() + numslices * 210 + lastlen, slicelast);
	
		sb2last=Enbucket(sb2, slice,slicelast);

		sblast=sb.begin(); //start new buffer to send
		toInsert={ 0xF0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, memframecnt++, (byte)((numslices) % 128), (byte)((lastlen - 1) / 16), (byte)((lastlen - 1) % 16) }; 
		sblast = std::copy(toInsert.begin(),toInsert.end(),sblast);
		sblast= std::copy(sb2.begin(),sb2last,sblast);
		*sblast++=0xF7;
		//print whole buffer as a chain of HEX
		// TRACE_V_THR30IIPEDAL(Serial.println(F("\n\rLast frame to send in \"createpatch\":"));
		// 		hexdump(sb,sblast-sb.begin());
		// )
		m =  SysExMessage(sb.data(),sblast-sb.begin());
		om = Outmessage(m, (uint16_t)(101 + numslices), true, false);  //Ack, but no answer for the header 
		outqueue.enqueue(om);  //send last slice to THRxxII
	}
	
	TRACE_THR30IIPEDAL(Serial.println(F("\n\rCreate_patch(): Ready outsending."));)

	userSettingsHaveChanged=false;
	activeUserSetting=-1;

	// Now we have to await acknowledgment for the last frame

	// on_ack_queue.enqueue(std::make_tuple(101+numslices,Outmessage(SysExMessage((const byte[29]) { 0xf0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, 0x02, 0x00, 0x00, 0x0b, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x3c, 0x00, 0x7f, 0x7f, 0x7f, 0x7f, 0x00, 0x00, 0xf7 },29), 
	//                      101+numslices+1, false, true),nullptr,false)); //answer will be the settings dump for actual settings

    // Summary of what we did just now:
	//                 Building the SysEx frame:
	//                 f0 00 01 0c 22 02 4d(always the same)
	//                 01   (memory command, not a settings command)
	//                 Get actual counter for memory frames and append it here
	//                 append same frame counter (is the slice counter) as a byte
	//                 set length field to  0d 01  (except for last frame, where we use it's stored length before bitbucketing)
	//                 append "bitbucketed" data for this slice
	//                 finish with 0xF7
	//                 Send it out.

} //End of THR30II_Settings::createPatch()

byte THR30II_Settings::UseSysExSendCounter() //returns the actual counter value and increments it afterwards
{
  return (SysExSendCounter++)%0x80;  //cycles to 0, if more than 0x7f
}

int8_t THR30II_Settings::getActiveUserSetting() //Getter for number of the active user setting
{
	return activeUserSetting;
}

bool THR30II_Settings::getUserSettingsHaveChanged() //Getter for state of user Settings
{
	return userSettingsHaveChanged;
}

//Tokens to find inside the patch data
std::map<String, std::vector<byte> > THR30II_Settings::tokens 
{
	{{"StructOpen"},  { 0x00, 0x00, 0x00, 0x80, 0x02, 0x00 }},
	{{"StructClose"}, { 0x02, 0x00, 0x00, 0x80, 0x00, 0x00 }},
	{{"UnitOpen"},    { 0x03, 0x00, 0x00, 0x80, 0x07, 0x00 }},
	{{"UnitClose"},   { 0x04, 0x00, 0x00, 0x80, 0x00, 0x00 }},
	{{"Data"},        { 0x01, 0x00, 0x00, 0x00, 0x01, 0x00 }},
	{{"Meta"}, 		  { 0x02, 0x00, 0x00, 0x00, 0x01, 0x00 }},
	{{"TokenMeta"},	  { 0x00, 0x80, 0x02, 0x00, 0x50, 0x53, 0x52, 0x50 }},
	{{"TokenData"},   { 0x00, 0x80, 0x02, 0x00, 0x54, 0x52, 0x54, 0x47 }},
	{{"UnitType"},	  { 0x00, 0x00, 0x05, 0x00 }},
	{{"ParCount"},	  { 0x00, 0x00, 0x06, 0x00 }},
	{{"PseudoVal"},	  { 0x00, 0x80, 0x07, 0x00 }},   
	{{"PseudoType"},  { 0x00, 0x80, 0x02, 0x00 }}
};

static std::map<uint16_t, string_or_ulong> globals;   //global settings from a patch-dump
std::map<uint16_t, Dumpunit> units;  //ushort value for UnitKey
uint16_t actualUnitKey = 0, actualSubunitKey = 0;
static String logg ;   //the log while analyzing dump

//helper for "patchSetAll()"
//check key and following token, if in state "Structure"
static THR30II_Settings::States checkKeyStructure(byte key[], byte * buf, int buf_len, uint16_t &pt)
{
	if (pt + 6 > buf_len - 8)  //if no following octet fits in the buffer
	{
		logg+="Error: Unexpected end of buffer while in Structure context!\n"; 
		return THR30II_Settings::States::St_error;  //Stay in state Structure
	}

	byte next_octet[8];

	memcpy(next_octet,buf+pt+6,8);  //Fetch following octet from buffer

	if (memcmp(key, THR30II_Settings::tokens["Data"].data(),6)==0)
	{
		logg+="Token Data found. ";
		if (memcmp(next_octet, THR30II_Settings::tokens["TokenData"].data(),8)==0)
		{
			logg+="Token Data complete. Data:\n\r";
			pt += 8;
			return THR30II_Settings::States::St_data;
		}
	}
	else if (memcmp(key, THR30II_Settings::tokens["Meta"].data(),6)==0)
	{
		logg+="Token Meta found. ";
		if (memcmp(next_octet, THR30II_Settings::tokens["TokenMeta"].data(),8)==0)
		{
			logg+="Token Meta complete. Global:\n\r";
			pt += 8;
			return THR30II_Settings::States::St_global;
		}
	}
	logg+="--->Structure:\n";
	return THR30II_Settings::States::St_structure;
}

//helper for "patchSetAll()"
//check key, if in state "Data"
THR30II_Settings::States checkKeyData(byte key[], byte * buf, int buf_len, uint16_t &pt)
{
	if (pt + 6 > buf_len)  //If no token follows (end of buffer)
	{
	  	logg+="Error: Unexpected end of buffer while in Data-Context!\n\r";
		return THR30II_Settings::States::St_error;
	}

	if (memcmp(key,THR30II_Settings::tokens["UnitOpen"].data(),6)==0)
	{
		logg+="token UnitOpen found.  Unit:\n\r";
		return THR30II_Settings::States::St_unit;
	}
	else if (memcmp(key, THR30II_Settings::tokens["StructClose"].data(),6)==0)  //This will perhaps never happen (Empty Data-Section)
	{
		logg+="Token StructClose found.  Idle:\n\r";
		return THR30II_Settings::States::St_idle;
	}
	logg+="Data:\n\r";
	return THR30II_Settings::States::St_data;
}

//helper for "patchSetAll()"
//check key and eventually following token, if in state "Unit"
THR30II_Settings::States checkKeyUnit(byte key[], byte* buf,int buf_len, uint16_t &pt)
{
	if (pt + 6 > buf_len)   //if buffer ends after this buffer
	{
		logg+="Error: Unexpected end of buffer while in a Unit!\n\r";
		return THR30II_Settings::States::St_error;
	}

	if (memcmp(key, THR30II_Settings::tokens["UnitOpen"].data(),6)==0)
	{
		logg+="token UnitOpen found. State Subunit:\n\r";
		return THR30II_Settings::States::St_subunit;
	}
	else if (memcmp(key+2, THR30II_Settings::tokens["UnitType"].data(),4)==0)
	{
		logg+="token UnitType found. ";
		uint16_t unitKey = (uint16_t)(key[0] + 256 * key[1]);
		if (pt + 6 > buf_len - 8)   //if no following octet fits in the buffer
		{
			logg+="Error: Unexpected end of buffer while in Unit "+String(unitKey) +" !\n\r";
			return THR30II_Settings::States::St_error;
		}
		pt += 10;
		uint16_t unitType = (uint16_t)(buf[pt] + 256 * buf[pt + 1]);
		uint16_t parCount = (uint16_t)(buf[pt + 10] + 256 * buf[pt + 11]);

		std::map<uint16_t, key_longval> emptyvals;
		std::map<uint16_t, Dumpunit> emptysubUnits;

		Dumpunit actualUnit =
		{
			unitType,
			parCount,
			emptyvals,      //values
			emptysubUnits   //subUninits
		};

		units.emplace(std::pair<uint16_t,Dumpunit>(unitKey, actualUnit));  //store actual unit as a top level unit
		pt += 8;

		actualUnitKey = unitKey;
		logg+="State ValuesUnit:\n\r";
		return THR30II_Settings::States::St_valuesUnit;
	}
	else if (memcmp(key, THR30II_Settings::tokens["UnitClose"].data(),6)==0)
	{
		logg+="token UnitClose found. State Data:\n\r";
		return THR30II_Settings::States::St_data;  //Should only happen at the end of last unit
	}
	logg+="State Error:\n\r";
	return THR30II_Settings::States::St_error;
}
//helper for "patchSetAll()"
//check key and eventually following token, if in state "SubUnit"
THR30II_Settings::States checkKeySubunit(byte key[], byte *buf, int buf_len, uint16_t &pt)
{
	if (pt + 6 > buf_len)  //if buffer ends after this unit
	{
		logg+="Error: Unexpected end of buffer while in a SubUnit!\n\r";
		return THR30II_Settings::States::St_error;
	}

	if (memcmp(key, THR30II_Settings::tokens["UnitOpen"].data(),6)==0)
	{
		logg+="token UnitOpen found.  State Error:\n\r";
		return THR30II_Settings::States::St_error;  //no Sub-SubUnits possible
	}
	else if (memcmp(key+2, THR30II_Settings::tokens["UnitType"].data(), 4)==0)
	{
		logg+="token UnitType found. ";
		uint16_t unitKey = (int16_t)(key[0] + 256 * key[1]);
		if (pt + 6 > buf_len - 8)  //if no following octet fits in buffer
		{
			logg+="Error: Unexpected end of buffer while in SubUnit " + String(unitKey) + " !\n\r";
			return THR30II_Settings::States::St_error;
		}
		pt += 10;
		uint16_t unitType = (uint16_t)(buf[pt] + 256 * buf[pt + 1]);
		uint16_t parCount = (uint16_t)(buf[pt + 10] + 256 * buf[pt + 11]);

		std::map<uint16_t, key_longval> emptyVals;
		std::map<uint16_t, Dumpunit> emptysubUnits;

		Dumpunit actualSubunit 
		{
			unitType,
			parCount,
		    emptyVals,  //values 
			emptysubUnits  //subUnits
		};

		units[actualUnitKey].subUnits.emplace(unitKey, actualSubunit);

		pt += 8;
		actualSubunitKey = unitKey;
		logg+="State ValuesSubunit:\n\r";
		return THR30II_Settings::States::St_valuesSubunit;
	}
	else if (memcmp(key, THR30II_Settings::tokens["UnitClose"].data(),6)==0)
	{
		logg+="token UnitClose found.  Unit:\n\r";
		actualSubunitKey = 0xFFFF;  //Not in a subunit any more
		return THR30II_Settings::States::St_unit;  //Should only happen at the end of last unit
	}
	logg+="Error: No allowed trigger in State SubUnit:\n\r";
	return THR30II_Settings::States::St_error;
}

//get data, when in state "Global"
//fetch one global setting from the patch dump (called several times, if there are several global values)
static void getGlobal(byte * buf, int buf_len, uint16_t &pt)  
{
	uint16_t lfdNr = (uint16_t)buf[pt + 0] + 256 * (uint16_t)buf[pt + 1];

	byte type = buf[pt + 4];  //get typ code for the global value

	if (type == 0x04)  //string (the patch name)
	{
		byte len = buf[pt + 6];  //ignore 3 High Bytes, because string is always shorter than 255 
		logg.append(String("found string of len ")+String((int)len)+String(" : "));

		String name;

		if (len > 0)
		{
			if (len > 64)
			{
			    char tmp[len];
				memcpy(tmp,buf+pt+10,len-1)	;
				name = String(tmp).substring(0,64);
			}
			else
			{
				char tmp[len];
				memcpy(tmp,buf+pt+10,len-1);
				name =  String(tmp);
			}
            logg.append(name + String("\n\r") );
		}

		globals.emplace(std::pair< uint16_t, string_or_ulong> ( lfdNr, (string_or_ulong){true, type, name, 0}) );

		pt += (uint16_t)len + 4;
	}
	else if (type == 0x02 || type == 0x03)
	{
		globals.emplace(std::pair<uint16_t,string_or_ulong> ( lfdNr, 
		                                                    (string_or_ulong) {false, type,
															(uint32_t)buf[pt + 6] + ((uint32_t)buf[pt + 7] << 8)
		                                                  + ((uint32_t)buf[pt + 8] << 16) + ((uint32_t)buf[pt + 9] << 24)} ));
		pt += 4;
	}
	else
	{
		logg.append("unknown type code " +String(type) + " for global value!\n\r" );
		pt += 4;
	}
	
}

//get data, when in state "Unit"
//get one of the units' settings from the patch dump (called several times, if there are several values)
THR30II_Settings::States getValueUnit(byte key[], byte * buf, int buf_len, uint16_t &pt) 
{
	if (memcmp(key, THR30II_Settings::tokens["UnitOpen"].data(),6)==0)
	{
		logg+="token UnitOpen found.  SubUnit:\n\r";
		return THR30II_Settings::States::St_subunit;
	}
	else if (memcmp(key, THR30II_Settings::tokens["UnitClose"].data(),6)==0)
	{
		logg+="token UnitClose found. Data:\n\r";
		return THR30II_Settings::States::St_data;
	}
	//No Open or Close => should be a value
	
	if(pt + 6 > buf_len - 4) //if no following quartet fits in the buffer
	{
		logg+="Error: Unexpected end of buffer while in ValueUnit context!\n\r"; 
		return THR30II_Settings::States::St_error;  //Stay in state Structure
	}
	uint16_t parKey = (uint16_t)(key[0] + 256 * key[1]); //get the key for this value

	byte type = buf[pt + 4];  //get the type key for the following 4-Byte-value
	//get the 4-byte-value itself
	uint32_t val = (uint32_t)(((uint32_t)buf[pt + 6]) + ((uint32_t)buf[pt + 7] << 8) + ((uint32_t)buf[pt + 8] << 16) + ((uint32_t)buf[pt + 9] << 24));
	units[actualUnitKey].values.emplace(parKey, (key_longval) {type, val} );
	pt += 4;
	//Stay in context ValueUint to read further value(s)
	logg+="ValuesUnit:\n\r";
	return THR30II_Settings::States::St_valuesUnit;
}

//helper for "patchSetAll()" - checks, if there is a value in the current subUnti and extract this value (4-Byte)
THR30II_Settings::States getValueSubunit(byte key[], byte* buf, int buf_len, uint16_t &pt)
{
	if (memcmp(key, THR30II_Settings::tokens["UnitOpen"].data(),6)==0)
	{
		logg+="token UnitOpen found.  Unit:\n\r";
		return THR30II_Settings::States::St_error;  //No Sub-SubUnits possible
	}
	else if (memcmp(key, THR30II_Settings::tokens["UnitClose"].data(),6)==0)
	{
		logg+="token UnitClose in Subunit found.  Unit:\n\r";
		actualSubunitKey = 0xFFFF;  // No Subunit any more
		return THR30II_Settings::States::St_unit;
	}
	
	//No Open or Close => should be a value
	if(pt + 6 > buf_len - 4) //if no following quartet fits in the buffer
	{
		logg+="Error: Unexpected end of buffer while in ValueSubUnit context!\n\r"; 
		return THR30II_Settings::States::St_error;  
	}

	uint16_t parKey = (uint16_t)(key[0] + 256 * key[1]); //get the key for this value
	
	byte type = buf[pt + 4];//get the type key for the following 4-Byte-value
	//get the 4-byte-value itself
	uint32_t val = (uint32_t)(((uint32_t)buf[pt + 6]) + ((uint32_t)buf[pt + 7] << 8) + ((uint32_t)buf[pt + 8] << 16) + ((uint32_t)buf[pt + 9] << 24));

	units[actualUnitKey].subUnits[actualSubunitKey].values.emplace(parKey, (key_longval) {type, val});

	pt += 4;
	//Stay in context ValueSubUnit to read further value(s)
	logg+="ValuesSubunit:\n\r";
	return THR30II_Settings::States::St_valuesSubunit;
}

//extract all settings from a received dump. returns error code
int THR30II_Settings::patch_setAll(uint8_t * buf, uint16_t buf_len)  
{
	uint16_t pt = 0; //index
	_state=States::St_idle;
	globals.clear();
	units.clear();
	actualUnitKey=0; actualSubunitKey=0;
	
	//Setting up a data structure to keep the retrieved dump data
	logg=String();
	//uint16_t patch_size = buf_len;
	
	//overwrite locally stored patch values with received buffer's data
	
	//Zustand "idle"
	logg.append("Idle:\n\r");

	while ((pt <= buf_len - 6) && _state!=St_error ) //walk through the patch data and look for token-sextetts
	{
		byte sextet[6];
		memcpy(sextet, buf+pt, 6); //fetch token sextet from buffer
		char tmp[36];
		sprintf(tmp, "Byte %d of %d : %02X%02X%02X%02X%02X%02X : ",pt,buf_len, sextet[0],sextet[1],sextet[2],sextet[3],sextet[4],sextet[5]);
		logg.append(tmp);
		
		if(_state == States::St_idle)
		{
			//From Idle-state we can reach the Structure-state, 
			//if we send the corresponding key as a trigger.
			if(memcmp(sextet, THR30II_Settings::tokens["StructOpen"].data(),6)==0)
			{ 
				//Reaches state "Structure"
				logg.append("Structure:\n\r");					
				_state=States::St_structure;
				//_last_state=St_idle;
			}
		}
		else if(_state== States::St_structure)
		{
			//Switching options from this state on
			_state = checkKeyStructure(sextet,buf,buf_len, pt);
			//_last_state=St_structure;
		}
		else if(_state== States::St_data)
		{
			//Switching options from this state on
			_state = checkKeyData(sextet,buf,buf_len,pt);
			//_last_state=St_data;
		}
		else if(_state== States::St_meta)
		{
			//Switching options from this state on
			if(memcmp(sextet, THR30II_Settings::tokens["StructClose"].data(),6)==0)
			{ 
				//Reaches state "Idle"
				logg.append("Idle:\n\r");
				_state=St_idle;
			}
			else
			{
				//Reaches state "Global"
				logg.append("Global:\n\r");
				_state=St_global;
			}
			//_last_state= St_meta;
		}
		else if(_state==States::St_global)
		{
			//Switching options from this state on
			if(memcmp(sextet, THR30II_Settings::tokens["StructClose"].data(),6)==0)
			{
				//Reaches state"Idle"
				logg.append("Idle:\n\r");
					_state=St_idle;
			}
			else
			{ 
				getGlobal(buf,buf_len, pt);
				
				//Reaches state "Global"
				logg.append("Global:\n\r");
				_state= St_global;
			}
			//_last_state=St_global;
		}
		else if(_state==States::St_unit)
		{
			//Switching options from this state on
			_state=checkKeyUnit(sextet,buf,buf_len,pt);
			//_last_state=St_unit;
		}
		else if(_state==States::St_valuesUnit )
		{
			//Switching options from this state on
			_state=getValueUnit(sextet,buf,buf_len,pt);
			//_last_state=St_valuesUnit;
		}
		else if(_state==States::St_subunit)
		{
			//Switching options from this state on
			_state=checkKeySubunit(sextet,buf,buf_len,pt);
			//_last_state=St_subunit;
		}
		else if (_state==States::St_valuesSubunit)
		{
			//Switching options from this state on
			_state=getValueSubunit(sextet,buf,buf_len,pt);
			//_last_state=St_valuesSubunit;
		}
		pt += 6; //advance in buffer
	}

	TRACE_THR30IIPEDAL(Serial.println("patch_setAll parsing ready - results: ");)
	TRACE_V_THR30IIPEDAL(Serial.println(logg);)     
	TRACE_THR30IIPEDAL(Serial.println("Setting the "+ String(globals.size())+" globals: ");)

	//Walk through the globals (Structur Meta)
	for( std::pair<uint16_t,string_or_ulong> kvp : globals)
	{
		if( kvp.first == 0x0000)
		{
			if (kvp.second.isString )
			{
				SetPatchName(kvp.second.nam,-1);
			}
		}
		else if(kvp.first == 0x0001)
		{
			if (!kvp.second.isString) //int
			{
				Tnid = kvp.second.val;
			}
		}
		if(kvp.first== 0x0002)
		{
			if (!kvp.second.isString)
			{
				UnknownGlobal = kvp.second.val;
			}
		}
		if(kvp.first==  0x0003)
		{
			if (!kvp.second.isString )
			{
				ParTempo = kvp.second.val;
			}
		}
	}

	TRACE_THR30IIPEDAL(Serial.println("... setting unit vals: ");)
	//Recurse through the whole data structure created while parsing the dump
	
	for (std::pair<uint16_t, Dumpunit> du : units)    //foreach!
	{
		if(du.first == THR30II_UNITS_VALS[GATE].key)  //Unit Gate also hosts MIX and Subunits COMP...REV
		{
			TRACE_V_THR30IIPEDAL(Serial.println("In dumpunit GATE/AMP");)

			if (du.second.parCount != 0)
			{
				std::map<uint16_t, Dumpunit> &dict2 = du.second.subUnits;  //Get SubUnits of GATE/MIX

				for (std::pair<uint16_t, Dumpunit> kvp : dict2)  //foreach
				{
					if(kvp.first== THR30II_UNITS_VALS[ECHO].key)   //If SubUnit "Echo"
					{
						TRACE_V_THR30IIPEDAL(Serial.println("In dumpSubUnit ECHO");)

						//Before we set Parameters we have to select the Echo-Type
						uint16_t t=kvp.second.type;

						auto result = std::find_if(
							THR30II_ECHO_TYPES_VALS.begin(),
							THR30II_ECHO_TYPES_VALS.end(),
							[t](const auto& mapobj) {return mapobj.second.key == t;}
						);
						if(result!=THR30II_ECHO_TYPES_VALS.end())
						{
							EchoSelect(result->first);
						}
						else  //Defaults to TAPE_ECHO
						{
							EchoSelect(THR30II_ECHO_TYPES::TAPE_ECHO);
						}
						for(std::pair<uint16_t, key_longval> p : kvp.second.values)     //Values contained in SubUnit "Echo"
						{
							uint16_t key = EchoMap(p.first);  //map the dump-keys to the MIDI-Keys 
							EchoSetting(key, NumberToVal(p.second.val));
						}
					}
					else if(kvp.first== THR30II_UNITS_VALS[EFFECT].key)   //If SubUnit "Effect"
					{
						TRACE_V_THR30IIPEDAL(Serial.println("In dumpSubUnit EFFECT");)

						//Before we set Parameters we have to select the Effect-Type
						uint16_t t=kvp.second.type;

						auto result = std::find_if(
							THR30II_EFF_TYPES_VALS.begin(),
							THR30II_EFF_TYPES_VALS.end(),
							[t](const auto& mapobj) {return mapobj.second.key == t;}
						);
						if(result!=THR30II_EFF_TYPES_VALS.end())
						{
							EffectSelect(result->first);
						}
						else  //Defaults to PHASER
						{
							EffectSelect(THR30II_EFF_TYPES::PHASER);
						}

						for(std::pair<uint16_t, key_longval> p : kvp.second.values)     //Values contained in SubUnit "Effect"
						{
							uint16_t key = EffectMap(p.first);  //map the dump-keys to the MIDI-Keys 
							EffectSetting(key, NumberToVal(p.second.val));
						}
					}
					else if(kvp.first== THR30II_UNITS_VALS[COMPRESSOR].key)   //If SubUnit "Compressor"
					{
						TRACE_V_THR30IIPEDAL(Serial.println("In dumpSubUnit COMPRESSOR");)

						for(std::pair<uint16_t, key_longval> p :kvp.second.values)  //Values contained in SubUnit "Compressor"
						{
							uint16_t key = CompressorMap(p.first);   //map the dump-keys to the MIDI-Keys
							//Find Setting for this key
							auto result = std::find_if(
								THR30II_COMP_VALS.begin(),
								THR30II_COMP_VALS.end(),
								[key](const auto& mapobj) {return mapobj.second == key;}
							);
							if(result!=THR30II_COMP_VALS.end())
							{
								CompressorSetting(result->first, NumberToVal(p.second.val) );
							}
							else  //Defaults to CO_SUSTAIN
							{
								CompressorSetting(CO_SUSTAIN, NumberToVal(p.second.val));
							}
						}
					}
					else if(kvp.first==THR30II_UNITS_VALS[THR30II_UNITS::REVERB].key)   //If SubUnit "Reverb"
					{
						TRACE_V_THR30IIPEDAL(Serial.println("In dumpSubUnit REVERB");)

						//Before we set Parameters we have to select the Reverb-Type
						uint16_t t=kvp.second.type;

						auto result = std::find_if(
							THR30II_REV_TYPES_VALS.begin(),
							THR30II_REV_TYPES_VALS.end(),
							[t](const auto& mapobj) {return mapobj.second.key == t;}
						);
						if(result!=THR30II_REV_TYPES_VALS.end())
						{
							ReverbSelect(result->first);
						}
						else  //Defaults to SPRING
						{
							ReverbSelect(SPRING);
						}

						for (std::pair<uint16_t, key_longval> p : kvp.second.values)     //Values contained in SubUnit "Reverb"
						{
							uint16_t key = ReverbMap(p.first);  //map the dump-keys to the MIDI-Keys 
							ReverbSetting(key, NumberToVal(p.second.val));
						}
					}
					else if(kvp.first== THR30II_UNITS_VALS[THR30II_UNITS::CONTROL].key)          //If SubUnit "Control/Amp"
					{
						TRACE_V_THR30IIPEDAL(Serial.println("In dumpSubUnit CTRL/AMP");)
						setColAmp(kvp.second.type);
						
						for(std::pair<uint16_t, key_longval> p : kvp.second.values)     //Values contained in SubUnit "Amp"
						{
							uint16_t key = controlMap[p.first];   //map the dump-keys to the MIDI-Keys
							
							//Find Setting for this key
							auto result = std::find_if(
								THR30II_CTRL_VALS.begin(),
								THR30II_CTRL_VALS.end(),
								[key](const auto& mapobj) {return mapobj.second == key;}
							);
							double val=0.0;
							if(result!=THR30II_CTRL_VALS.end())
							{   
								val=NumberToVal(p.second.val);
							    TRACE_THR30IIPEDAL(Serial.printf("Setting main control %d = %.1f\n\r", ((std::_Rb_tree_iterator<std::pair<const THR30II_CTRL_SET,uint16_t>>) result)->first, val);)
								SetControl(result->first, val);
							}
							else  //Defaults to  CTRL_GAIN
							{
								TRACE_THR30IIPEDAL(Serial.printf("Setting main control \"Gain\" (default) = %.1f\n\r", val);)
								SetControl(THR30II_CTRL_SET::CTRL_GAIN, NumberToVal(p.second.val));
							}
						}
					}
				} //end of foreach kvp in dict2  (Subunits of GuitarProc (Gate) an their params)
				
				std::map<uint16_t, key_longval> &dict = du.second.values;  //Values -directly- contained in Unit GATE/MIX

				for (std::pair<uint16_t, key_longval> kvp : dict)
				{
					if( unitOnMap[kvp.first] == THR30II_UNIT_ON_OFF_COMMANDS[EFFECT])
					{
						Switch_On_Off_Effect_Unit(kvp.second.val != 0);
					}
					else if( unitOnMap[kvp.first] == THR30II_UNIT_ON_OFF_COMMANDS[ECHO])
					{ 
						Switch_On_Off_Echo_Unit(kvp.second.val != 0);
					}
					else if( unitOnMap[kvp.first] == THR30II_UNIT_ON_OFF_COMMANDS[REVERB])
					{
						Switch_On_Off_Reverb_Unit(kvp.second.val != 0);
					}
					else if( unitOnMap[kvp.first] == THR30II_UNIT_ON_OFF_COMMANDS[COMPRESSOR])
					{ 
						Switch_On_Off_Compressor_Unit(kvp.second.val != 0);
					}
					else if( unitOnMap[kvp.first] == THR30II_UNIT_ON_OFF_COMMANDS[GATE])
					{   
						Switch_On_Off_Gate_Unit(kvp.second.val != 0);
					}
					else if(gateMap[kvp.first]== THR30II_GATE_VALS[GA_DECAY])
					{
						gate_setting[GA_DECAY] = NumberToVal(kvp.second.val);
					}
					else if(gateMap[kvp.first] == THR30II_GATE_VALS[GA_THRESHOLD])
					{
						gate_setting[GA_THRESHOLD] = NumberToVal_Threshold(kvp.second.val);
					}
					else if(reverbMap[kvp.first] == THR30II_INFO_REVERB[reverbtype]["MIX"].sk)
					{   
						ReverbSetting(reverbtype, reverbMap[kvp.first], NumberToVal(kvp.second.val));
					}
					else if(effectMap[kvp.first] == THR30II_INFO_EFFECT[effecttype]["MIX"].sk)
					{
						EffectSetting(effecttype, effectMap[kvp.first], NumberToVal(kvp.second.val));
					}
					else if(echoMap[kvp.first] == THR30II_INFO_ECHO[echotype]["MIX"].sk)
					{
						EchoSetting(echotype, echoMap[kvp.first], NumberToVal(kvp.second.val));
					}
					else if(compressorMap[kvp.first] == THR30II_COMP_VALS[CO_MIX])
					{
						CompressorSetting(CO_MIX, NumberToVal(kvp.second.val) );
					}
					else if(kvp.first == THR30II_CAB_COMMAND_DUMP)
					{
						SetCab((THR30II_CAB)kvp.second.val);
					}
					else if(kvp.first== Constants::glo["AmpEnableState"])//0x0120:   //AMP_EnableState (not used in patches to THRII)
					{																 //But occurs in dumps from THRII to PC
						TRACE_THR30IIPEDAL(Serial.printf("\"AmpEnableState\" %d.\n\r",kvp.second.val);)
					}
					
				} // of for (each) kvp in dict

			} //end of if count params !=0
		} //end of if "Unit GATE" (contains COMP...REV as subunits)
	} //end of foreach dumpunit

	return 0; //success
} //end of THR30II_settings::patch_setAll

//following all the setters for locally stored THR30II-Settings class

// void THR30II_Settings::SetAmp(uint8_t _amp)   //No separate setter necessary at the moment
// {

// }

void THR30II_Settings::SetColAmp(THR30II_COL _col, THR30II_AMP _amp)  //Setter for the Simulation Collection / Amp
{
	// bool changed = false;
	if (_col != col)
	{
		col = _col;
		// changed = true;
	}
	if (_amp != amp)
	{
		amp = _amp;
		// changed = true;
	}
	// if (changed)
	// {
	// 	if (sendChangestoTHR)
	// 	{
	// 		SendColAmp();
	// 	}
	// }
}

void THR30II_Settings::setColAmp(uint16_t ca)  //Setter by key
{	
	auto result = std::find_if(				//Lookup Key for this value
          THR30IIAmpKeys.begin(),
          THR30IIAmpKeys.end(),
          [ca](const auto& mapobj) {return mapobj.second == ca; });

	//RETURN VARIABLE IF FOUND
	if(result != THR30IIAmpKeys.end())
	{
		col_amp foundkey = result->first;	
		SetColAmp(foundkey.c, foundkey.a);
	}
}

void THR30II_Settings::SetCab(THR30II_CAB _cab)  //Setter for the Cabinet Simulation
{
	cab = _cab;

	if (sendChangestoTHR)
	{
		SendCab();
	}
}

void THR30II_Settings::EchoSetting(uint16_t ctrl, double value)  //Setter for the Echo Parameters
{
	 EchoSetting(echotype, ctrl, value);  //using setter with actual selected echo type
}

void THR30II_Settings::EchoSetting(THR30II_ECHO_TYPES type, uint16_t ctrl, double value)  //Setter for the Echo Parameters
{
	switch(type)
	{
		case TAPE_ECHO:
			THR30II_ECHO_SET_TAPE stt; //find setting by it's key
			for(auto it= THR30II_INFO_TAPE.begin(); it!=THR30II_INFO_TAPE.end(); it++)
			{
				if(it->second.sk==ctrl)
				{
					stt=it->first;
				}
			} 
			if (value >= THR30II_INFO_TAPE[stt].ll && value <= THR30II_INFO_TAPE[stt].ul)
			{
				echo_setting[TAPE_ECHO][stt] = value;  //all double values
				// if(stt== TA_MIX) //Mix is identical for both types!
				// {
				// 	echo_setting[DIGITAL_DELAY][DD_MIX] = value;
				// }

				if (sendChangestoTHR)
				{
					SendParameterSetting( un_cmd {THR30II_INFO_TAPE[stt].uk, THR30II_INFO_TAPE[stt].sk} ,type_val<double> {0x04, value} );  //send settings change to THR
				}
			}
			break;
		case DIGITAL_DELAY:
			THR30II_ECHO_SET_DIGI std; //find setting by it's key
			for(auto it= THR30II_INFO_DIGI.begin(); it!=THR30II_INFO_DIGI.end(); it++)
			{
				if(it->second.sk==ctrl)
				{
					std=it->first;
				}
			}

			if (value >= THR30II_INFO_DIGI[std].ll && value <= THR30II_INFO_DIGI[std].ul)
			{
				echo_setting[DIGITAL_DELAY][std] = value;  //all double values
				// if (std == DD_MIX) //Mix is identical for both types!
				// {
				// 	echo_setting[TAPE_ECHO][TA_MIX] = value;
				// }
				if (sendChangestoTHR)
				{
					SendParameterSetting(un_cmd {THR30II_INFO_DIGI[std].uk, THR30II_INFO_DIGI[std].sk }, type_val<double> {0x04, value} );  //send settings change to THR
				}
			}
			break;
	}
}

void THR30II_Settings::GateSetting(THR30II_GATE ctrl, double value) //Setter for gate effect parameters
{
	gate_setting[ctrl] = constrain(value, 0, 100);

	if (sendChangestoTHR)
	{
		SendParameterSetting((un_cmd){THR30II_UNITS_VALS[GATE].key, THR30II_GATE_VALS[ctrl]},(type_val<double>) {0x04, value});  //send reverb type change to THR
	}
}

void THR30II_Settings::CompressorSetting(THR30II_COMP ctrl, double value)  //Setter for compressor effect parameters
{
	compressor_setting[ctrl] = constrain(value, 0, 100);

	if (sendChangestoTHR)
	{
		if (ctrl == THR30II_COMP::CO_SUSTAIN)
		{
			SendParameterSetting((un_cmd){THR30II_UNITS_VALS[COMPRESSOR].key, THR30II_COMP_VALS[CO_SUSTAIN]}, (type_val<double>){0x04, value});  //send settings change to THR
		}
		else if (ctrl == THR30II_COMP::CO_LEVEL)
		{
			SendParameterSetting((un_cmd){THR30II_UNITS_VALS[COMPRESSOR].key, THR30II_COMP_VALS[CO_LEVEL]}, (type_val<double>){0x04, value});  //send settings change to THR
		}
		else if (ctrl == THR30II_COMP::CO_MIX)
		{
			SendParameterSetting((un_cmd){THR30II_UNITS_VALS[GATE].key, THR30II_COMP_VALS[CO_MIX]}, (type_val<double>){0x04, value});  //send settings change to THR
		}
	}
}

void THR30II_Settings::EffectSetting(uint16_t ctrl, double value) //using the internally selected actual effect type
{
		EffectSetting(effecttype, ctrl, value);
}

void THR30II_Settings::EffectSetting(THR30II_EFF_TYPES type, uint16_t ctrl, double value)  //Setter for Effect parameters by type and key
{
	switch (type)
	{
		case THR30II_EFF_TYPES::CHORUS:
			
				THR30II_EFF_SET_CHOR stc;   //find setting by it's key
				for( auto it= THR30II_INFO_CHOR.begin(); it!=THR30II_INFO_CHOR.end(); it++)
				{
                       if(it->second.sk==ctrl)
					   {
						   stc= it->first;
					   }
				}

				if (value >= THR30II_INFO_CHOR[stc].ll && value <= THR30II_INFO_CHOR[stc].ul)
				{
					effect_setting[CHORUS][stc] = value;  //all double values
					// if(stc== CH_MIX)   //MIX is identical for all types!
					// {
					// 	effect_setting[FLANGER][FL_MIX] = value;
					// 	effect_setting[PHASER][PH_MIX] = value;
					// 	effect_setting[TREMOLO][TR_MIX] = value;
					// }
					if (sendChangestoTHR)     //do not send back, if change results from THR itself
					{
						SendParameterSetting( (un_cmd) { THR30II_INFO_CHOR[stc].uk, THR30II_INFO_CHOR[stc].sk}, (type_val<double>) {0x04, value});  //send settings change to THR
					}
				}
	
			break;

		case THR30II_EFF_TYPES::FLANGER:

				THR30II_EFF_SET_FLAN stf; //find setting by it's key
				for( auto it= THR30II_INFO_FLAN.begin(); it!=THR30II_INFO_FLAN.end(); it++)
				{
                       if(it->second.sk==ctrl)
					   {
						   stf= it->first;
					   }
				}

				if (value >= THR30II_INFO_FLAN[stf].ll && value <= THR30II_INFO_FLAN[stf].ul)
				{
					effect_setting[FLANGER][stf] = value;  //all double values
					// if (stf == FL_MIX)   //MIX is identical for all types!
					// {
					// 	effect_setting[CHORUS][CH_MIX] = value;
					// 	effect_setting[PHASER][PH_MIX] = value;
					// 	effect_setting[TREMOLO][TR_MIX] = value;
					// }					
					if (sendChangestoTHR) //do not send back, if change results from THR itself
					{
						SendParameterSetting((un_cmd){THR30II_INFO_FLAN[stf].uk, THR30II_INFO_FLAN[stf].sk},(type_val<double>) {0x04, value});  //send settings change to THR
					}
				}

			break;

		case THR30II_EFF_TYPES::TREMOLO:

			THR30II_EFF_SET_TREM stt ; //find setting by it's key
			
			for( auto it= THR30II_INFO_TREM.begin(); it!=THR30II_INFO_TREM.end(); it++)
			{
					if(it->second.sk==ctrl)
					{
						stt= it->first;
					}
			}
			if (value >= THR30II_INFO_TREM[stt].ll && value <= THR30II_INFO_TREM[stt].ul)
			{
				effect_setting[TREMOLO][stt] = value;  //all double values
				//  if (stt == TR_MIX)   //MIX is identical for all types!
                //         {
                //             effect_setting[FLANGER][FL_MIX] = value;
                //             effect_setting[PHASER][PH_MIX] = value;
                //             effect_setting[CHORUS][CH_MIX] = value;
                //         }
				if (sendChangestoTHR)  //do not send back, if change results from THR itself
				{
					SendParameterSetting((un_cmd){THR30II_INFO_TREM[stt].uk, THR30II_INFO_TREM[stt].sk},(type_val<double>) {0x04, value});  //send settings change to THR
				}
			}

		break;
		case THR30II_EFF_TYPES::PHASER:
			THR30II_EFF_SET_PHAS stp ; //find setting by it's key
			for( auto it= THR30II_INFO_PHAS.begin(); it!=THR30II_INFO_PHAS.end(); it++)
			{
					if(it->second.sk==ctrl)
					{
						stp= it->first;
					}
			}

			if (value >= THR30II_INFO_PHAS[stp].ll && value <= THR30II_INFO_PHAS[stp].ul)
			{
				effect_setting[PHASER][stp] = value;  //all double values
				// if (stp == PH_MIX)   //MIX is identical for all types!
                //             {
                //                 effect_setting[FLANGER][FL_MIX] = value;
                //                 effect_setting[CHORUS][CH_MIX] = value;
                //                 effect_setting[TREMOLO][TR_MIX] = value;
                //             }
				if (sendChangestoTHR)  //do not send back, if changes result from THR itself
				{
					SendParameterSetting((un_cmd){THR30II_INFO_PHAS[stp].uk, THR30II_INFO_PHAS[stp].sk},(type_val<double>) {0x04, value});  //send settings change to THR
				}
			}
		break;
	}
}

void THR30II_Settings::ReverbSetting(THR30II_REV_TYPES type, uint16_t ctrl, double value)
{
	switch (type)
	{
		case THR30II_REV_TYPES::SPRING:
			THR30II_REV_SET_SPRI sts;  //find setting by it's key
			for( auto it= THR30II_INFO_SPRI.begin(); it!=THR30II_INFO_SPRI.end(); it++)
			{
					if(it->second.sk==ctrl)
					{
						sts= it->first;
					}
			}

			if (value >= THR30II_INFO_SPRI[sts].ll && value <= THR30II_INFO_SPRI[sts].ul)
			{
				reverb_setting[SPRING][sts] = value;  //all double values
				//  if(sts== SP_MIX)   //MIX is identical for all types!
                //             {
                //                 reverb_setting[PLATE][PL_MIX] = value;
                //                 reverb_setting[HALL][HA_MIX] = value;
                //                 reverb_setting[ROOM][RO_MIX] = value;
                //             }
				if (sendChangestoTHR)
				{
					SendParameterSetting((un_cmd){THR30II_INFO_SPRI[sts].uk, THR30II_INFO_SPRI[sts].sk},(type_val<double>) {0x04, value});  //send settings change to THR
				}
			}
			break;
		case THR30II_REV_TYPES::PLATE:
			THR30II_REV_SET_PLAT stp; //find setting by it's key
			for( auto it= THR30II_INFO_PLAT.begin(); it!=THR30II_INFO_PLAT.end(); it++)
			{
					if(it->second.sk==ctrl)
					{
						stp= it->first;
					}
			}

			if (value >= THR30II_INFO_PLAT[stp].ll && value <= THR30II_INFO_PLAT[stp].ul)
			{
				reverb_setting[PLATE][stp] = value;  //all double values
				//  if (stp == PL_MIX)   //MIX is identical for all types!
                //             {
                //                 reverb_setting[SPRING][SP_MIX] = value;
                //                 reverb_setting[HALL][HA_MIX] = value;
                //                 reverb_setting[ROOM][RO_MIX] = value;
                //             }
				if (sendChangestoTHR)
				{
					SendParameterSetting((un_cmd){THR30II_INFO_PLAT[stp].uk, THR30II_INFO_PLAT[stp].sk}, (type_val<double>){0x04, value});  //send settings change to THR
				}
			}
			break;
		case THR30II_REV_TYPES::HALL:
			THR30II_REV_SET_HALL sth; //find setting by it's key
			for( auto it= THR30II_INFO_HALL.begin(); it!=THR30II_INFO_HALL.end(); it++)
			{
					if(it->second.sk==ctrl)
					{
						sth= it->first;
					}
			}

			if (value >= THR30II_INFO_HALL[sth].ll && value <= THR30II_INFO_HALL[sth].ul)
			{
				reverb_setting[HALL][sth] = value;  //all double values
				// if (sth == HA_MIX)   //MIX is identical for all types!
				// {
				// 	reverb_setting[SPRING][SP_MIX] = value;
				// 	reverb_setting[PLATE][PL_MIX] = value;
				// 	reverb_setting[ROOM][RO_MIX] = value;
				// }
				if (sendChangestoTHR)
				{
					SendParameterSetting((un_cmd){THR30II_INFO_HALL[sth].uk, THR30II_INFO_HALL[sth].sk},(type_val<double>) {0x04, value});  //send settings change to THR
				}
			}
			break;
		case THR30II_REV_TYPES::ROOM:
			THR30II_REV_SET_ROOM str; //find setting by it's key
			for( auto it= THR30II_INFO_ROOM.begin(); it!=THR30II_INFO_ROOM.end(); it++)
			{
					if(it->second.sk==ctrl)
					{
						str= it->first;
					}
			}
			if (value >= THR30II_INFO_ROOM[str].ll && value <= THR30II_INFO_ROOM[str].ul)
			{
				reverb_setting[ROOM][str] = value;  //all double values
				// if (str == RO_MIX)   //MIX is identical for all types!
                //             {
                //                 reverb_setting[SPRING][SP_MIX] = value;
                //                 reverb_setting[PLATE][PL_MIX] = value;
                //                 reverb_setting[HALL][HA_MIX] = value;
                //             }
				if (sendChangestoTHR)
				{
					SendParameterSetting((un_cmd){THR30II_INFO_ROOM[str].uk, THR30II_INFO_ROOM[str].sk},(type_val<double>) {0x04, value});  //send settings change to THR
				}
			}
			break;
	}
}

void THR30II_Settings::ReverbSetting(uint16_t ctrl, double value)  //using internally selected actual reverb type
{
	ReverbSetting(reverbtype, ctrl, value);  //use actual selected reverbtype
}

void THR30II_Settings::Switch_On_Off_Gate_Unit(bool state)   //Setter for switching on / off the effect unit
{
	unit[GATE] = state;

	if (sendChangestoTHR)  //do not send back, if change results from THR itself
	{
		SendUnitState(GATE);
	}
}

void THR30II_Settings::Switch_On_Off_Echo_Unit(bool state)   //Setter for switching on / off the Echo unit
{
	unit[ECHO] = state;

	if (sendChangestoTHR)  //do not send back, if change results from THR itself
	{
		SendUnitState(ECHO);
	}
}

void THR30II_Settings::Switch_On_Off_Effect_Unit(bool state)   //Setter for switching on / off the effect unit
{
	unit[EFFECT] = state;

	if (sendChangestoTHR)  //do not send back, if change results from THR itself
	{
		SendUnitState(EFFECT);
	}
}

void THR30II_Settings::Switch_On_Off_Compressor_Unit(bool state)  //Setter for switching on /off the compressor unit
{
	unit[COMPRESSOR] = state;

	if (sendChangestoTHR) //do not send back, if change results from THR itself
	{
		SendUnitState(COMPRESSOR);
	}
}

void THR30II_Settings::Switch_On_Off_Reverb_Unit(bool state)   //Setter for switching on / off the reverb unit
{
	unit[REVERB] = state;
	
	if (sendChangestoTHR)   //do not send back, if change results from THR itself
	{
		SendUnitState(REVERB);
	}
}

void THR30II_Settings::SetControl(uint8_t ctrl, double value)
{
   control[ctrl]=value;
   if (sendChangestoTHR)
   {
       SendParameterSetting( un_cmd {THR30II_UNITS_VALS[CONTROL].key,THR30II_CTRL_VALS[(THR30II_CTRL_SET)ctrl] },type_val<double> {0x04,value});
   }
}

double THR30II_Settings::GetControl(uint8_t ctrl)
{
	return control[ctrl];
}

void THR30II_Settings::ReverbSelect(THR30II_REV_TYPES type)  //Setter for selection of the reverb type
{
	reverbtype = type;
	
	if (sendChangestoTHR)
	{
		SendTypeSetting(REVERB, THR30II_REV_TYPES_VALS[type].key);  //send reverb type change to THR
	}

	//Todo: await Acknowledge
}

void THR30II_Settings::EffectSelect(THR30II_EFF_TYPES type)   //Setter for selection of the Effect type
{
	effecttype = type;
	
	if (sendChangestoTHR)  //do not send back, if change results from THR itself
	{
		SendTypeSetting(EFFECT, THR30II_EFF_TYPES_VALS[type].key);  //send effect type change to THR
	}

	//Todo: await Acknowledge
}

void THR30II_Settings::EchoSelect(THR30II_ECHO_TYPES type)   //Setter for selection of the Effect type
{
	echotype = type;
	
	if (sendChangestoTHR)  //do not send back, if change results from THR itself
	{
		SendTypeSetting(ECHO, THR30II_ECHO_TYPES_VALS[type].key);  //send echo type change to THR
	}

	//Todo: await Acknowledge
}

THR30II_Settings::THR30II_Settings():patchNames( { "Actual", "UserSetting1", "UserSetting2", "UserSetting3", "UserSetting4", "UserSetting5" })  //Constructor
{
	dumpFrameNumber = 0; 
	dumpByteCount = 0;  //received bytes
	dumplen = 0;  //expected length in bytes
    Firmware = 0x00000000;
    ConnectedModel = 0x00000000; //FamilyID (2 Byte) + ModelNr.(2 Byte) , 0x0024_0002=THR30II
	unit[EFFECT]=false;unit[ECHO]=false;unit[REVERB]=false;unit[COMPRESSOR]=false;unit[GATE]=false;
}

void THR30II_Settings::SetPatchName(String nam, int nr)  //nr = -1 as default for actual patchname
{														 //corresponds to field "activeUserSetting"
	TRACE_THR30IIPEDAL(Serial.println(String("SetPatchName(): Patchnname: ") + nam + String(" nr: ") + String(nr) );)
	
	nr = constrain(nr, 0, 5);  //make 0 based index for array patchNames[]

	patchNames[nr] = nam.length()>64?nam.substring(0,64):nam; //cut, if necessary

	if (sendChangestoTHR)
	{
		CreateNamePatch();
	}
}

//---------METHOD FOR UPLOADING PATCHNAME TO THR30II -----------------

void THR30II_Settings::CreateNamePatch() //fill send buffer with just setting for actual patchname, creating a valid SysEx for sending to THR30II  
{                             //uses same algorithm as CreatePatch() - but will only be  o n e  frame!
	//1.) Make the data buffer (structure and values) store the length.
	//    Add 12 to get the 2nd length-field for the header. Add 8 to get the 1st length field for the header.
	//2.) Cut it to slices with 0x0d02 (210 dez.) bytes (gets the payload of the frames)
	//    These frames will thus not have incomplete 8/7 groups after bitbucketing but total frame length is 253 (just below the possible 255 bytes).
	//    The remaining bytes will build the last frame (perhaps with last 8/7 group incomplete)
	//    Store the length of the last slice (all others are fixed 210 bytes  long.)
	//3.) Create the header:
	//    SysExStart:  f0 00 01 0c 24 02 4d (always the same)
	//                 01   (memory command, not settings command)
	//                 Get actual counter for memory frames and append it here
	//                 00 01 0b  ("same frame counter" and payload size  for the header)
	//                 hang following 4-Byte values together:
	//
	//                 0d 00 00 00   (Opcode for memory write/send)
	//                 (len +8+12)   (1st length field = total length= data length + 3 values + type field/netto length )
	//                 FF FF FF FF   (user patch number to overwrite/ 0xFFFFFFFF for actual patch)
	//                 (len   +12)   (2nd length field = netto length= data length + the following 3 values)
	//                 00 00 00 00   (always the same, kind of opcode?)
	//                 01 00 00 00   (always the same, kind of opcode?)
	//                 00 00 00 00   (always the same, kind of opcode?)
	//
	//                 Bitbucket this group (gives no incomplete 8/7 group!)
	//                 and append it to the header
	//                 finish header with 0xF7
	//                 Header frame may already be sent out!
	//4.)              For each slice:
	//                 Bitbucket encode the slice
	//                 Build frame:
	//                 f0 00 01 0c 24 02 4d(always the same)
	//                 01   (memory command, not settings command)
	//                 Get actual counter for memory frames and append it here
	//                 append same frame counter (is the slice counter) as a byte
	//                 0d 01  (except for last frame, where we use it's stored length before bitbucketing)
	//                 bitbucketed data for this slice
	//                 0xF7
	//                 Send it out.

	TRACE_V_THR30IIPEDAL(Serial.println(F("Create_Name_Patch(): "));)
	
	std::array<byte,300> dat;
	auto datlast =dat.begin();  //iterator to last element (starts on begin! )
	std::array<byte,300> sb;
	auto sblast = sb.begin();   //iterator to last element (starts on begin! )
	std::array<byte,300> sb2;
	auto sb2last = sb2.begin();  //iterator to last element (starts on begin! )

	std::array<byte,4> toInsert4;
	std::array<byte,6> toInsert6;

	#define datback(x)  for(const byte &b : x ) { *datlast++ = b; };
	
	//Macro for converting a 32-Bit value to a 4-byte array<byte,4> and append it to "dat"
	#define conbyt4(x) toInsert4={ (byte)(x), (byte) ((x)>>8), (byte)((x)>>16) , (byte) ((x)>>24) };  datlast= std::copy( std::begin(toInsert4), std::end(toInsert4), datlast ); 
	//Macro for appending a 16-Bit value to "dat"
	#define conbyt2(x) *datlast++=(byte)(x); *datlast++= (byte)((x)>>8); 
	
	//1.) Make the data buffer (structure and values)

	datback( tokens["StructOpen"] );
	//Meta
	datback(tokens["Meta"] ) ;          
	datback(tokens["TokenMeta"] );
	toInsert6= { 0x00, 0x00, 0x00, 0x00, 0x04, 0x00 };
	datlast=std::copy( std::begin(toInsert6),std::end(toInsert6),datlast) ;    //number 0x0000, type 0x00040000 (String)
	
	conbyt4(  patchNames[0].length() + 1 ) // Length of patchname (incl. '\0')
		//32Bit-Value (little endian; low byte first) 
	
	std::string nam =patchNames[0u].c_str();
	datback( nam ); //copy the patchName  //!!ZWEZWE!! mind UTF-8 ?
	*datlast++='\0'; //append  '\0' to the patchname
	
	datback(tokens["StructClose"]); //close Structure Meta
	
	//print whole buffer as a chain of HEX
	TRACE_V_THR30IIPEDAL( Serial.printf("\n\rRaw frame before slicing/enbucketing:\n\r"); hexdump(dat,datlast-dat.begin());)

	//    Now store the length.
	//    Add 12 to get the 2nd length-field for the header. Add 8 to get the 1st length field for the header.
	uint32_t datalen = (uint32_t) (datlast-dat.begin());
	
	//2.) cut to slices
	uint32_t lengthfield2 = datalen + 12;
	uint32_t lengthfield1 = datalen + 20;

	int numslices = (int)datalen / 210; //number of 210-byte slices
	int lastlen = (int)datalen % 210; //data left for last frame containing the rest
	TRACE_V_THR30IIPEDAL(Serial.print("Datalen: "); Serial.println(datalen);)
	TRACE_V_THR30IIPEDAL(Serial.print("Number of slices: "); Serial.println(numslices);)
	TRACE_V_THR30IIPEDAL(Serial.print("Length of last slice: "); Serial.println(lastlen);)

	//3.) Create the header:
	//    SysExStart:  f0 00 01 0c 22 02 4d (always the same)
	//                 01   (memory command, not settings command)
	//                 put following 4-Byte values together:
	//
	//                 0d 00 00 00   (Opcode for memory write/send)
	
	//Macro for converting a 32-Bit value to a 4-byte vector<byte,4> and append it to "sb"
	#define conbyt4s(x) toInsert4={ (byte)(x), (byte) ((x)>>8), (byte)((x)>>16) , (byte) ((x)>>24) };  sblast= std::copy( std::begin(toInsert4), std::end(toInsert4), sblast ); 
	//sblast starts at sb.begin()
	conbyt4s(0x0du);
	//                 (len +8+12)   (1st length field = total length= data length + 3 values + type field/netto length )
	conbyt4s(lengthfield1);
	//                 FF FF FF FF   (number of user patch to overwrite / 0xFFFFFFFF for actual patch)
	conbyt4s(0xFFFFFFFFu);
	//                 (len   +12)   (2nd length field = netto length= data length + the following 3 values)
	conbyt4s(lengthfield2);
	//                 00 00 00 00   (always the same, kind of opcode?)
	conbyt4s(0x0u);
	//                 01 00 00 00   (always the same, kind of opcode?)
	conbyt4s(0x1u);
	//                 00 00 00 00   (always the same, kind of opcode?)
	conbyt4s(0x0u);
	//
	//Bitbucket this group (gives no incomplete 8/7 group!)
	sb2last = Enbucket(sb2,sb,sblast); 
	//                 and append it to the header
	//                 finish header with 0xF7
	//                 Header frame may already be sent out!
	byte memframecnt = 0x06;
	//                 Get actual counter for memory frames and append it here

	//                 00 01 0b  ("same frame counter" and payload size  for the header)
	sblast=sb.begin(); //start the buffer to send
	
	std::array<byte,12> toInsert;
	toInsert=std::array<byte,12>({ 0xf0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, memframecnt++, 0x00, 0x01, 0x0b });
	sblast= std::copy( toInsert.begin(),toInsert.end(),sb.begin());
	sblast= std::copy( sb2.begin(), sb2last, sblast);

	*sblast++=0xF7; //SysEx end demarkation

	SysExMessage m ( sb.data(),sblast -sb.begin() );
	Outmessage om (m, 100, false, false);  //no Ack, no answer for the header 
	outqueue.enqueue(om);  //send header to THRxxII

	//4.)              For each slice:
	//                 Bitbucket encode the slice
	
	for (int i = 0; i < numslices; i++)
	{
		std::array<byte,210> slice;
		std::copy(dat.begin() + i*210, dat.begin() + i*210 +210, slice.begin() ) ;
		sb2last=Enbucket(sb2, slice, slice.end());
		sblast=sb.begin(); //start new buffer to send
		//toInsert=  { 0xF0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, memframecnt++, (byte)(i % 128), 0x0d, 0x01 };
		toInsert=  { 0xF0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, memframecnt, (byte)(i % 128), 0x0d, 0x01 };  //memframecount is not incremented inside sent patches
		sblast=std::copy(toInsert.begin(), toInsert.end(), sblast );
		sblast=std::copy( sb2.begin(),sb2last, sblast );
		*sblast++=0xF7;
		//print whole buffer as a chain of HEX
		TRACE_V_THR30IIPEDAL(Serial.printf("\n\rFrame %d to send in \"Create_Name_Patch\":\n\r",i);
				hexdump(sb,sblast-sb.begin());
		)
		m = SysExMessage(sb.data(), sblast -sb.begin() );
		om = Outmessage(m, (uint16_t)(101 + i), false, false);  //no Ack, for all the other slices 
		outqueue.enqueue(om);  //send slice to THRxxII
	}

	if (lastlen > 0) //last slice (could be the first, if it is the only one)
	{
		std::array<byte,210> slice;  //here 210 is a maximum size, not the true element count
		byte *slicelast=slice.begin();
		slicelast=std::copy(dat.begin() + numslices * 210, dat.begin() + numslices * 210 + lastlen, slicelast);
	
		sb2last=Enbucket(sb2, slice,slicelast);

		sblast=sb.begin(); //start new buffer to send
		toInsert={ 0xF0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, memframecnt++, (byte)((numslices) % 128), (byte)((lastlen - 1) / 16), (byte)((lastlen - 1) % 16) }; 
		sblast = std::copy(toInsert.begin(),toInsert.end(),sblast);
		sblast= std::copy(sb2.begin(),sb2last,sblast);
		*sblast++=0xF7;
		//print whole buffer as a chain of HEX
		TRACE_V_THR30IIPEDAL(Serial.println(F("\n\rLast frame to send in \"Create_Name_Patch\":"));
				hexdump(sb,sblast-sb.begin());
		)
		m =  SysExMessage(sb.data(),sblast-sb.begin());
		om = Outmessage(m, (uint16_t)(101 + numslices), true, false);  //Ack, but no answer for the header 
		outqueue.enqueue(om);  //send last slice to THRxxII
	}
	
	TRACE_THR30IIPEDAL(Serial.println(F("\n\rCreate_Name_patch(): Ready outsending."));)

	// Now we have to await acknowledgment for the last frame

	// on_ack_queue.enqueue(std::make_tuple(101+numslices,Outmessage(SysExMessage((const byte[29]) { 0xf0, 0x00, 0x01, 0x0c, 0x24, 0x02, 0x4d, 0x01, 0x02, 0x00, 0x00, 0x0b, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x3c, 0x00, 0x7f, 0x7f, 0x7f, 0x7f, 0x00, 0x00, 0xf7 },29), 
	//                      101+numslices+1, false, true),nullptr,false)); //answer will be the settings dump for actual settings

    // Summary of what we did just now:
	//                 Building the SysEx frame:
	//                 f0 00 01 0c 22 02 4d(always the same)
	//                 01   (memory command, not a settings command)
	//                 Get actual counter for memory frames and append it here
	//                 append same frame counter (is the slice counter) as a byte
	//                 set length field to  0d 01  (except for last frame, where we use it's stored length before bitbucketing)
	//                 append "bitbucketed" data for this slice
	//                 finish with 0xF7
	//                 Send it out.



}


String THR30II_Settings::getPatchName()
{
   return patchNames[constrain(activeUserSetting,0,5)];
}

//Find the correct (col, amp)-struct object for this ampkey
col_amp THR30IIAmpKey_ToColAmp(uint16_t ampkey)
{ 

	for (auto it = THR30IIAmpKeys.begin(); it != THR30IIAmpKeys.end(); ++it)
	{
		if (it->second == ampkey)
		{
			return (col_amp) (it->first);
		}
	}
	return col_amp{CLASSIC,CLEAN};
}

//---------FUNCTION FOR SENDING COL/AMP SETTING TO THR30II -----------------
void THR30II_Settings::SendColAmp() //Send COLLLECTION/AMP setting to THR
{
		uint16_t ak = THR30IIAmpKeys[(col_amp){col, amp}];
		SendTypeSetting(THR30II_UNITS::CONTROL, ak);
}

//---------FUNCTION FOR SENDING CAB SETTING TO THR30II -----------------
void THR30II_Settings::SendCab() //Send cabinet setting to THR
{
	SendParameterSetting((un_cmd) {THR30II_UNITS_VALS[THR30II_UNITS::GATE].key, THR30II_CAB_COMMAND},  (type_val<uint16_t>) {0x02, (uint16_t)cab} );
}

//---------FUNCTION FOR SENDING UNIT STATE TO THR30II -----------------
void THR30II_Settings::SendUnitState(THR30II_UNITS un) //Send unit state setting to THR30II
{
	type_val<uint16_t> tmp = {(byte){0x03}, unit[un] };
	SendParameterSetting((un_cmd) {THR30II_UNITS_VALS[THR30II_UNITS::GATE].key, THR30II_UNIT_ON_OFF_COMMANDS[un]},tmp);
}

//---------FUNCTIONS FOR SENDING SETTINGS CHANGES TO THR30II -----------------

void THR30II_Settings::SendTypeSetting(THR30II_UNITS unit, uint16_t val) //Send setting to THR (Col/Amp or Reverbtype or Effecttype)
{
	//This method is not really necessary for the THR30II-Pedal
	//We need it on the PC where we can change settings separately
	//
	//Because with the the pedal settings are changed all together with complete MIDI-patches 
	//Perhaps for future use: If knobs for switching on/off the units are added?
 /*
	if (!MIDI_Activated)
		return;

	byte[] raw_msg_body = new byte[8]; //2 Ints:  Unit + Val
	byte[] raw_msg_head = new byte[8]; //2 Ints:  Opcode + Len(Body)
	raw_msg_head[0] = 0x08;   //Opcode for unit type change
	raw_msg_head[4] = (byte)raw_msg_body.Length;  //Length of the body

	ushort key = THR30II_UNITS_VALS[unit].key;

	raw_msg_body[0] = (byte)(key % 256);
	raw_msg_body[1] = (byte)(key / 256);
	raw_msg_body[4] = (byte)(val % 256);
	raw_msg_body[5] = (byte)(val / 256);

	//Prepare Message-Body
	byte[] msg_body = Helpers.Enbucket(raw_msg_body);
	byte[] sendbuf_body = new byte[PC_SYSEX_BEGIN.Length + 2 + 3 + msg_body.Length + 1];  //for "00" and frame-counter; 3 for Length-Field, 1 for "F7"

	Buffer.BlockCopy(PC_SYSEX_BEGIN, 0, sendbuf_body, 0, PC_SYSEX_BEGIN.Length);
	sendbuf_body[PC_SYSEX_BEGIN.Length] = 0x00;
	sendbuf_body[PC_SYSEX_BEGIN.Length + 2] = 0x00;  //only one frame in this message
	sendbuf_body[PC_SYSEX_BEGIN.Length + 3] = (byte)((raw_msg_body.Length - 1) / 16);  //Length Field (Hi)
	sendbuf_body[PC_SYSEX_BEGIN.Length + 4] = (byte)((raw_msg_body.Length - 1) % 16); //Length Field (Low)
	Buffer.BlockCopy(msg_body, 0, sendbuf_body, PC_SYSEX_BEGIN.Length + 5, msg_body.Length);
	sendbuf_body[sendbuf_body.GetUpperBound(0)] = SYSEX_STOP;

	//Prepare Message-Header
	byte[] msg_head = Helpers.Enbucket(raw_msg_head);
	byte[] sendbuf_head = new byte[PC_SYSEX_BEGIN.Length + 2 + 3 + msg_head.Length + 1];

	Buffer.BlockCopy(PC_SYSEX_BEGIN, 0, sendbuf_head, 0, PC_SYSEX_BEGIN.Length);
	sendbuf_head[PC_SYSEX_BEGIN.Length] = 0x00;
	sendbuf_head[PC_SYSEX_BEGIN.Length + 2] = 0x00;  //only one frame in this message
	sendbuf_head[PC_SYSEX_BEGIN.Length + 3] = (byte)((raw_msg_head.Length - 1) / 16);  //Length Field (Hi)
	sendbuf_head[PC_SYSEX_BEGIN.Length + 4] = (byte)((raw_msg_head.Length - 1) % 16); //Length Field (Low)
	Buffer.BlockCopy(msg_head, 0, sendbuf_head, PC_SYSEX_BEGIN.Length + 5, msg_head.Length);
	sendbuf_head[sendbuf_head.GetUpperBound(0)] = SYSEX_STOP;

	if (THR10_Editor.THR10_Settings.O_device == null)
	{
		return;
	}
	if (!THR10_Editor.THR10_Settings.O_device.IsOpen)
	{
		THR10_Editor.THR10_Settings.O_device?.Open();
	}
	sendbuf_head[PC_SYSEX_BEGIN.Length + 1] = THR30II_Window.SysExSendCounter++;
	THR10_Editor.THR10_Settings.O_device?.Send(new SysExMessage(sendbuf_head));
	sendbuf_body[PC_SYSEX_BEGIN.Length + 1] = THR30II_Window.SysExSendCounter++;
	THR10_Editor.THR10_Settings.O_device?.Send(new SysExMessage(sendbuf_body));

	if (THR10_Editor.THR10_Settings.O_device.IsOpen)
	{
		THR10_Editor.THR10_Settings.O_device.Close();
	}

	//ToDO:  Set Wait for ACK
 */
}

String THR30II_Settings::THR30II_MODEL_NAME()
{ 
	switch(ConnectedModel) 
	{
		case 0x00240002:
			return String("THR30II");
			break;
		
		case 0x00240001:
			return String("THR10II");
			break;
		default:
			return String( "None");
		break;
	}
}

ArduinoQueue<Outmessage> outqueue(30);  //FIFO Queue for outgoing SysEx-Messages to THRII

//Messages, that have to be sent out,
// and flags to change, if an awaited acknowledge comes in (adressed by their ID)
ArduinoQueue <std::tuple<uint16_t, Outmessage, bool *, bool> > on_ack_queue;

ArduinoQueue<SysExMessage> inqueue(40);



/////////////////////////////////////////////////////////////////////////////////////////////////
    // UI DRAWING FUNCTIONS //
/////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Draw boxes for the bars and values on the TFT
 *
 * \param x x-position (0) where to place top left corner of status mask
 * \param y y-position     where to place top left corner of status mask
 *
 * \return nothing
 */
// void drawStatusMask(uint8_t x, uint8_t y)
// {
//  tft.fillRect(x,y,tft.width()-x,tft.height()-y,ST7789_BLACK); //clear background

//  y+=25; //advance y - position to the bars
 //THR30II_UNITS { COMPRESSOR=0, CONTROL=1, EFFECT=2, ECHO=3, REVERB=4, GATE=5 };
//  uint16_t xp[6] = {230 ,0, 98, 142, 186, 274}; //upper left corner for effect units on/off rect frames
 //               Co      Ef   Ec   Re   Ga

//  tft.drawRect( 1+x,0+y,15,111,ST7789_WHITE);  //frame of gain bar
//  tft.drawRect(20+x,0+y,15,111,ST7789_WHITE);  //frame of master bar
//  tft.drawRect(40+x,0+y,54,111,ST7789_WHITE);  //frame of bass - treble bar box
//  tft.drawRect(98+x,0+y,220,118,ST7789_WHITE); //frame effect bars' box
 
//  tft.drawLine(57+x,1+y,57+x,109+y,ST7789_LIGHTSLATEGRAY); //separator lines in bass - treble bar box
//  tft.drawLine(74+x,1+y,74+x,109+y,ST7789_LIGHTSLATEGRAY); 

//  tft.drawLine(xp[ECHO]+x,1+y,xp[ECHO]+x,116+y,ST7789_LIGHTSLATEGRAY); //separator lines in effect bars' box
//  tft.drawLine(xp[REVERB]+x,1+y,xp[REVERB]+x,116+y,ST7789_LIGHTSLATEGRAY);
//  tft.drawLine(xp[COMPRESSOR]+x,1+y,xp[COMPRESSOR]+x,116+y,ST7789_LIGHTSLATEGRAY);
//  tft.drawLine(xp[GATE]+x,1+y,xp[GATE]+x,116+y,ST7789_LIGHTSLATEGRAY);

//  tft.drawLine(xp[EFFECT]+x,110+y,317+x,110+y,ST7789_WHITE); //top of effect's on/off boxes
//  tft.drawLine(xp[EFFECT]+x,87+y,317+x,87+y,ST7789_WHITE);   //top of effect type boxes
  
//  tft.loadFont(AA_FONT_SMALL); // Set a small current font
//  tft.setTextSize(1);
//  tft.setTextDatum(TL_DATUM);
 
//  tft.setTextColor(ST7789_MAGENTA, TFT_BLACK);
//  tft.drawString("G",1+x,112+y);//printAt(tft,1+x,112+y,"G");
//  tft.setTextColor(ST7789_YELLOW, TFT_BLACK);
//  tft.drawString("M",20+x,112+y);//printAt(tft,20+x,112+y,"M");
//  tft.setTextColor(ST7789_CORAL, TFT_BLACK);//Light RED
//  tft.drawString("B",42+x,112+y);//printAt(tft,42+x,112+y,"B"); 
//  tft.setTextColor(ST7789_GREEN, TFT_BLACK);
//  tft.drawString("Mi",57+x,112+y);//printAt(tft,57+x,112+y,"Mi");
//  tft.setTextColor(ST7789_LIGHTBLUE, TFT_BLACK);
//  tft.drawString("T",79+x,112+y);//printAt(tft,79+x,112+y,"T");

 //Set font to very small
//  tft.setTextSize(1);
//  tft.setTextColor(ST7789_ORANGE, TFT_BLACK);
//  tft.drawString("Eff",xp[EFFECT]+x+5,116+y);//printAt(tft,xp[EFFECT]+x+5,116+y,"Eff"); //Effect
//  tft.setTextColor(ST7789_BEIGE, TFT_BLACK);
//  tft.drawString("Echo",xp[ECHO]+x,116+y);//printAt(tft,xp[ECHO]+x,116+y,"Echo");   //Echo
//  tft.setTextColor(ST7789_YELLOWGREEN, TFT_BLACK);
//  tft.drawString("Rev",xp[REVERB]+x+5,116+y);//rintAt(tft,xp[REVERB]+x+5,116+y,"Rev"); //Reverb
//  tft.setTextColor(ST7789_RED, TFT_BLACK);
//  tft.drawString("Com",xp[COMPRESSOR]+x+5,116+y);//printAt(tft,xp[COMPRESSOR]+x+5,116+y,"Com"); //Compressor
//  tft.setTextColor(ST7789_BLUE, TFT_BLACK);
//  tft.drawString("Gate",xp[GATE]+x,116+y);//printAt(tft,xp[GATE]+x,116+y,"Gate");   //Gate 

//  tft.unloadFont();
// }

void drawPatchID(uint16_t fgcolour, int patchID) {
  	int x = 0, y = 0, w = 80, h = 80;
	uint16_t bgcolour = TFT_THRVDARKGREY;
	spr.createSprite(w, h);
	spr.fillSmoothRoundRect(0, 0, w-1, h-1, 3, bgcolour, TFT_BLACK);
	spr.setTextFont(7);
	spr.setTextSize(1);
	spr.setTextDatum(MR_DATUM);
	spr.setTextColor(TFT_BLACK);
	spr.drawNumber(88, w-7, h/2);
	spr.setTextColor(fgcolour);
	spr.drawNumber(patchID, w-7, h/2);
	spr.pushSprite(x, y);
	spr.unloadFont();
	spr.deleteSprite();
}

void drawPatchIconBank(int presel_patch_id, int active_patch_id) {
	uint16_t iconcolour = 0;
	int banksize = 5;
	int first = ((presel_patch_id-1)/banksize)*banksize+1;  // find number for 1st icon in row
	
	switch(_uistate) {
		case UI_home_amp:
			for (int i = 1; i <= 5; i++)
			{
				if (i == THR_Values.getActiveUserSetting()+1)
				{
					iconcolour = TFT_SKYBLUE;  // highlight active patch icon
				}
				else if (i == presel_patch_id)
				{
					iconcolour = ST7789_ORANGE;  // highlight pre-selected patch icon
				}
				else
				{
					iconcolour = TFT_THRBROWN;  // colour for unselected patch icons
				}
				if (i > npatches)
				{
					iconcolour = TFT_BLACK;
				}
				drawPatchIcon(60+20*i, 0, 20, 20, iconcolour, i);
			}
		break;
		
		case UI_home_patch:
			for (int i = first; i < first+banksize; i++)
			{
				if (i == active_patch_id)
				{
					iconcolour = TFT_THRCREAM;  // highlight active patch icon
				}
				else if (i == presel_patch_id)
				{
					iconcolour = ST7789_ORANGE;  // highlight pre-selected patch icon
				}
				else
				{
					iconcolour = TFT_THRBROWN;  // colour for unselected patch icons
				}
				if (i > npatches)
				{
					iconcolour = TFT_BLACK;
				}
				drawPatchIcon(60+20*(i-first+1), 0, 20, 20, iconcolour, i);
			}
		break;

		default:
		break;
	}
}

void drawPatchIcon(int x, int y, int w, int h, uint16_t colour, int patchID) {
  spr.createSprite(w, h);
  spr.fillSmoothRoundRect(0, 0, w-1, h-1, 3, colour, TFT_BLACK);
  spr.loadFont(AA_FONT_SMALL);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(TFT_BLACK, colour);
  spr.drawNumber(patchID, 10, 11);
  spr.pushSprite(x, y);
  spr.deleteSprite();
}

void drawPatchSelMode(uint16_t colour) {
  int x = 180, y = 0, w = 50, h = 20;
  spr.createSprite(w, h);
  spr.fillSmoothRoundRect(0, 0, w-1, h-1, 3, colour, TFT_BLACK);
  spr.loadFont(AA_FONT_SMALL);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(TFT_BLACK, colour);
  spr.drawString("NOW", w/2, h/2+1);
  spr.pushSprite(x, y);
  spr.deleteSprite();
}

void drawAmpSelMode(uint16_t colour, String string) {
  int x = 230, y = 0, w = 50, h = 20;
  spr.createSprite(w, h);
  spr.fillSmoothRoundRect(0, 0, w-1, h-1, 3, colour, TFT_BLACK);
  spr.loadFont(AA_FONT_SMALL);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(TFT_BLACK, colour);
  spr.drawString(string, w/2, h/2+1);
  spr.pushSprite(x, y);
  spr.deleteSprite();
}

void drawConnIcon(bool THRconnected) {
  spr.createSprite(40, 20);
  if (THRconnected) {
    // connected/green
    spr.fillSmoothRoundRect(0, 0, 40, 19, 3, TFT_GREEN, TFT_BLACK); // bg
    spr.drawWideLine(1, 9, 14, 9, 3, TFT_WHITE, TFT_GREEN);         // left wire
    spr.drawWideLine(26, 9, 38, 9, 3, TFT_WHITE, TFT_GREEN);        // right wire
    spr.fillSmoothRoundRect(12, 2, 8, 15, 3, TFT_WHITE, TFT_GREEN);   // plug
    spr.fillRect(16, 2, 4, 15, TFT_WHITE);   // plug
    spr.fillSmoothRoundRect(21, 2, 8, 15, 3, TFT_WHITE, TFT_GREEN);  // socket
    spr.fillRect(21, 2, 4, 15, TFT_WHITE);  // socket
    spr.fillRect(16, 4, 5, 3, TFT_WHITE);   // upper prong
    spr.fillRect(16, 12, 5, 3, TFT_WHITE);  // lower prong
  }
  else {
    // disconnected/red
    spr.fillSmoothRoundRect(0, 0, 40, 19, 3, TFT_RED, TFT_BLACK); // bg
    spr.drawWideLine(1, 9, 10, 9, 3, TFT_WHITE, TFT_RED);         // left wire
    spr.drawWideLine(30, 9, 38, 9, 3, TFT_WHITE, TFT_RED);        // right wire
    spr.fillSmoothRoundRect(8, 2, 8, 15, 3, TFT_WHITE, TFT_RED);   // plug
    spr.fillRect(12, 2, 4, 15, TFT_WHITE);   // plug
    spr.fillSmoothRoundRect(24, 2, 8, 15, 3, TFT_WHITE, TFT_RED);  // socket
    spr.fillRect(24, 2, 4, 15, TFT_WHITE);  // socket
    spr.fillRect(16, 4, 5, 3, TFT_WHITE);   // upper prong
    spr.fillRect(16, 12, 5, 3, TFT_WHITE);  // lower prong
  }
  spr.pushSprite(280, 0);
  spr.deleteSprite();
}

void drawPatchName(uint16_t fgcolour, String patchname) {
  int x = 80, y = 20, w = 240, h = 60;
  uint16_t bgcolour = TFT_THRVDARKGREY; 
  spr.createSprite(w, h);
  spr.fillSmoothRoundRect(0, 0, w, h-1, 3, bgcolour, TFT_BLACK);
  spr.loadFont(AA_FONT_LARGE);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(fgcolour, bgcolour);
  spr.drawString(patchname, w/2, h/2);
  spr.pushSprite(x, y);
  spr.unloadFont();
  spr.deleteSprite();
}

void drawBarChart(int x, int y, int w, int h, uint16_t bgcolour, uint16_t fgcolour, String label, int barh) {
  int tpad = 20; int pad = 0;
  spr.createSprite(w, h);
  spr.fillSmoothRoundRect(0, 0, w-1, h-1, 3, bgcolour, TFT_BLACK);
  spr.loadFont(AA_FONT_SMALL);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(fgcolour, bgcolour);
  spr.drawString(label, w/2-1, 11);
  spr.fillRect(pad, h-(h-tpad-pad)*barh/100, w-1-2*pad, (h-tpad-pad)*barh/100, fgcolour);
  spr.pushSprite(x, y);
  spr.unloadFont();
  spr.deleteSprite();
}

void drawEQChart(int x, int y, int w, int h, uint16_t bgcolour, uint16_t fgcolour, String label, int b, int m, int t) {
  int tpad = 20; int pad = 0;
  spr.createSprite(w, h);
  spr.fillSmoothRoundRect(0, 0, w-1, h, 3, bgcolour, TFT_BLACK);
  spr.loadFont(AA_FONT_SMALL);
  spr.setTextDatum(MC_DATUM);
  spr.setTextColor(fgcolour, bgcolour);
  spr.drawString(label, w/2, 11);
  spr.fillRect(pad, h-(h-tpad-pad)*b/100, w/3-1-2*pad, (h-tpad-pad)*b/100, fgcolour);
  spr.fillRect(w/3+pad, h-(h-tpad-pad)*m/100, w/3-1-2*pad, (h-tpad-pad)*m/100, fgcolour);
  spr.fillRect(2*w/3+pad, h-(h-tpad-pad)*t/100, w/3-1-2*pad, (h-tpad-pad)*t/100, fgcolour);
  spr.pushSprite(x, y);
  spr.unloadFont();
  spr.deleteSprite();
}

void drawAmpUnit(int x, int y, int w, int h, uint16_t bgcolour, uint16_t fgcolour, String label, THR30II_COL col, THR30II_AMP amp, THR30II_CAB cab) {
	int tpad = 2; int pad = 2;
	uint16_t ampcolour1 [3] = {TFT_THRRED, TFT_THRBLUE, TFT_THRGREEN};
	uint16_t ampcolour2 [3] = {TFT_THRLIGHTRED, TFT_THRLIGHTBLUE, TFT_THRLIGHTGREEN};
	spr.createSprite(w, h);
	spr.fillSmoothRoundRect(0, 0, w-1, h-1, 3, bgcolour, TFT_BLACK);	// Unit body
	spr.fillSmoothRoundRect(pad, tpad, w-1-2*pad, h-1-tpad-pad, 1, fgcolour, bgcolour);	// Draw area
	spr.drawSpot(16, 16, 7.5, ampcolour2[col], fgcolour);	// Col status light
	spr.drawSpot(16, 16, 6, ampcolour1[col], ampcolour2[col]);
	spr.drawSpot(19, 13, 1.5, ampcolour2[col], ampcolour1[col]);	// Col status light
	spr.loadFont(AA_FONT_MONO);
	spr.setTextDatum(ML_DATUM);
	spr.setTextColor(bgcolour, fgcolour);
	spr.drawString(THR30II_AMP_NAMES[amp], 28, 18);	// Unit label
	spr.drawString(THR30II_CAB_NAMES[cab], 12, 43);	// Unit label
	
	int cabcentrex = 210;
	int cabcentrey = 30;
	int cabw = 50;
	int cabh = 50;

	switch (cab)	// Draw cabinet icon depending on type
	{
		case 0 ... 6:
		case 14:
			spr.fillSmoothRoundRect(cabcentrex-cabw/2, cabcentrey-cabh/2, cabw-1, cabh-1, 5, bgcolour, fgcolour);
			spr.fillSmoothRoundRect(cabcentrex-cabw/2+2, cabcentrey-cabh/2+2, cabw-5, cabh-5, 3, fgcolour, bgcolour);
			spr.drawSpot(cabcentrex-cabw/4,   cabcentrey-cabh/4,   9, bgcolour, fgcolour);	// top left
			spr.drawSpot(cabcentrex-cabw/4,   cabcentrey-cabh/4,   7, fgcolour, bgcolour);
			spr.drawSpot(cabcentrex+cabw/4-2, cabcentrey-cabh/4,   9, bgcolour, fgcolour);	// top right
			spr.drawSpot(cabcentrex+cabw/4-2, cabcentrey-cabh/4,   7, fgcolour, bgcolour);
			spr.drawSpot(cabcentrex-cabw/4,   cabcentrey+cabh/4-2, 9, bgcolour, fgcolour);	// bottom left
			spr.drawSpot(cabcentrex-cabw/4,   cabcentrey+cabh/4-2, 7, fgcolour, bgcolour);
			spr.drawSpot(cabcentrex+cabw/4-2, cabcentrey+cabh/4-2, 9, bgcolour, fgcolour);	// bottom right
			spr.drawSpot(cabcentrex+cabw/4-2, cabcentrey+cabh/4-2, 7, fgcolour, bgcolour);
		break;

		case 7 ... 11:
			cabh = 30;
			spr.fillSmoothRoundRect(cabcentrex-cabw/2, cabcentrey-cabh/2, cabw-1, cabh-1, 5, bgcolour, fgcolour);
			spr.fillSmoothRoundRect(cabcentrex-cabw/2+2, cabcentrey-cabh/2+2, cabw-5, cabh-5, 3, fgcolour, bgcolour);
			spr.drawSpot(cabcentrex-cabw/4,   cabcentrey-1, 9, bgcolour, fgcolour);	// top left
			spr.drawSpot(cabcentrex-cabw/4,   cabcentrey-1, 7, fgcolour, bgcolour);
			spr.drawSpot(cabcentrex+cabw/4-2, cabcentrey-1, 9, bgcolour, fgcolour);	// top right
			spr.drawSpot(cabcentrex+cabw/4-2, cabcentrey-1, 7, fgcolour, bgcolour);
		break;

		case 12 ... 13:
		case 15:
			cabw = 40;
			cabh = 40;
			spr.fillSmoothRoundRect(cabcentrex-cabw/2, cabcentrey-cabh/2, cabw-1, cabh-1, 5, bgcolour, fgcolour);
			spr.fillSmoothRoundRect(cabcentrex-cabw/2+2, cabcentrey-cabh/2+2, cabw-5, cabh-5, 3, fgcolour, bgcolour);
			spr.drawSpot(cabcentrex-1, cabcentrey-1, 15, bgcolour, fgcolour);	// top left
			spr.drawSpot(cabcentrex-1, cabcentrey-1, 13, fgcolour, bgcolour);
		break;

		case 16:

		break;
	}

	spr.pushSprite(x, y);
	spr.unloadFont();
	spr.deleteSprite();
}

void drawUtilUnit(int x, int y, int w, int h, int bpad, uint16_t bgcolour, uint16_t fgcolour, String label, double utilparams[2]) {
	int tpad = 30; int pad = 2;
	int grx = pad; int gry = tpad; int grw = w-1-2*pad; int grh = h-1-tpad-pad;
	int barw = 0; int barh = grh/2;
	spr.createSprite(w, h);
	spr.fillSmoothRoundRect(0, 0, w-1, h-bpad, 3, bgcolour, TFT_BLACK);
	spr.loadFont(AA_FONT_SMALL);
	spr.setTextDatum(MC_DATUM);
	spr.setTextColor(fgcolour, bgcolour);
	spr.drawString(label, w/2, 16);
	spr.fillRect(grx, gry, grw, grh, fgcolour);	// draw graph area
	barw = grw * utilparams[0]/100;
	spr.fillRect(grx, gry, barw, barh, bgcolour);
	barw = grw * utilparams[1]/100;
	spr.fillRect(grx, gry+barh, barw, barh+1, bgcolour);
	spr.pushSprite(x, y);
	spr.unloadFont();
	spr.deleteSprite();
}

void drawFXUnit(int x, int y, int w, int h, uint16_t bgcolour, uint16_t fgcolour, String label, int nbars, double FXparams[5]) {
	int tpad = 30; int pad = 2;
	int grx = pad; int gry = tpad; int grw = w-1-2*pad; int grh = h-1-tpad-pad;
	int barw = grw/nbars;	// calculate bar width
	int barh = 0;
	spr.createSprite(w, h);
	spr.fillSmoothRoundRect(0, 0, w-1, h, 3, bgcolour, TFT_BLACK);	// draw FX unit
	spr.loadFont(AA_FONT_SMALL);
	spr.setTextDatum(MC_DATUM);
	spr.setTextColor(fgcolour, bgcolour);
	spr.drawString(label, w/2, 16);	// write label
	spr.fillRect(grx, gry, grw, grh, fgcolour);	// draw graph area
	for (int i = 0; i < nbars; i++)
	{
		barh = grh * FXparams[i]/100;	// calculate bar height
		if (nbars == 4) {
			barw = grw/nbars+1;
		}
		if (i == nbars-1) {	// widen last bar by 1px
			spr.fillRect(grx + i * barw, gry + grh - barh, barw+1, barh, bgcolour);
		} else {
			spr.fillRect(grx + i * barw, gry + grh - barh, barw, barh, bgcolour);
		}
	}
	spr.pushSprite(x, y);
	spr.unloadFont();
	spr.deleteSprite();
}

void drawPPChart(int x, int y, int w, int h, uint16_t bgcolour, uint16_t fgcolour, String label, int ped1, int ped2) {
	int tpad = 20; int pad = 0;
	int ped1height = ped1*(h-tpad-pad)/1023;
	int ped2height = ped2*(h-tpad-pad)/1023;
	spr.createSprite(w, h);
	spr.fillSmoothRoundRect(0, 0, w-1, h, 3, bgcolour, TFT_BLACK);
	spr.loadFont(AA_FONT_SMALL);
	spr.setTextDatum(MC_DATUM);
	spr.setTextColor(fgcolour, bgcolour);
	spr.drawString(label, w/2-1, 11);
	spr.fillRect(pad, h-ped1height, w/2-1-2*pad, ped1height, fgcolour);
	//spr.fillRect(pad, h-(h-tpad-pad)*ex/100, w/2-1-2*pad, (h-tpad-pad)*ex/100, fgcolour);
	spr.fillRect(w/2+pad, h-ped2height, w/2-1-2*pad, ped2height, fgcolour);
	spr.pushSprite(x, y);
	spr.unloadFont();
	spr.deleteSprite();
}

void THR30II_Settings::updateConnectedBanner() //Show the connected Model 
{
	// //Display "THRxxII" in blue (as long as connection is established)

	// s1=THR30II_MODEL_NAME();
	// if(ConnectedModel!=0)
	// {
	// 	tft.loadFont(AA_FONT_LARGE); // Set current font    
	// 	tft.setTextSize(1);
	// 	tft.setTextColor(TFT_BLUE, TFT_BLACK);
	// 	tft.setTextDatum(TL_DATUM);
	// 	tft.setTextPadding(160);
	// 	tft.drawString(s1, 0, 0);//printAt(tft,0,0, s1 ); // Print e.g. "THR30II" in blue in header position
	// 	tft.setTextPadding(0);
	// 	tft.unloadFont();
	// }
}

/**
 * \brief Redraw all the bars and values on the TFT
 *
 * \param x x-position (0) where to place top left corner of status mask
 * \param y y-position     where to place top left corner of status mask
 *
 * \return nothing
 */
void THR30II_Settings::updateStatusMask(uint8_t x, uint8_t y)
{
	// patch number
  	drawPatchID(TFT_THRCREAM, active_patch_id);
	
	// patch icon bank
	drawPatchIconBank(presel_patch_id, active_patch_id);

	// patch select mode (pre-select/immediate)
	if (send_patch_now)
	{
		drawPatchSelMode(TFT_THRCREAM);
		switch(_uistate) {
			case UI_home_amp:
				rgbcolour = strip.gamma32(strip.Color(127,127,95));	//Select colour (cream)
			break;
			case UI_home_patch:
				rgbcolour = strip.gamma32(strip.Color(0,95,127));	//Select colour (sky blue)
			break;
			default:
			break;
		}
		strip.setPixelColor(0, rgbcolour);	//Set pixel's color (in RAM)
		rgbcolour = strip.gamma32(strip.Color(127,127,95));	//Select colour (cream)
		strip.setPixelColor(4, rgbcolour);	//Set pixel's color (in RAM)
		strip.setPixelColor(5, rgbcolour);	//Set pixel's color (in RAM)
	}
	else
	{
		drawPatchSelMode(TFT_THRBROWN);
		if ((presel_patch_id == active_patch_id) && (_uistate == UI_home_patch))
		{
			rgbcolour = strip.gamma32(strip.Color(0,95,127));	//Select colour (sky blue)
		}
		else
		{
			rgbcolour = strip.gamma32(strip.Color(127,127,95));	//Select colour (cream)
		}
		strip.setPixelColor(0, rgbcolour);	//Set pixel's color (in RAM)
		rgbcolour = strip.gamma32(strip.Color(127,79,0));	//Select colour (yellowy-orange)
		strip.setPixelColor(4, rgbcolour);	//Set pixel's color (in RAM)
		strip.setPixelColor(5, rgbcolour);	//Set pixel's color (in RAM)
	}
	strip.show();	//Update strip to match
	
	// amp select mode (COL/AMP/CAB)
	switch (amp_select_mode)
	{
		case COL:
			drawAmpSelMode(TFT_THRCREAM, "COL");
		break;
		case AMP:
			drawAmpSelMode(TFT_THRCREAM, "AMP");
		break;
		case CAB:
			drawAmpSelMode(TFT_THRCREAM, "CAB");
		break;
	}

	//THR30II_UNITS { COMPRESSOR=0, CONTROL=1, EFFECT=2, ECHO=3, REVERB=4, GATE=5 };
	// uint16_t xp[6] = {231 ,0, 99, 143, 187, 275}; //upper left corner for effect units on/off rects
	//               Co     Ef   Ec   Re   Ga
	// uint8_t measure;	 //helpers for calculating bar lengths
	// uint16_t delta=100;  //Always the same range for THR30II, THR10II (THR10 has different ranges for parameters)
	// double val_scale1=108.0/delta;
	// double val_scale2=85.0/delta;

	String FXtitle;
	uint16_t FXbgcolour = 0;
	uint16_t FXfgcolour = 0;
	double FXparams[5] = {0};
	double utilparams[2] = {0};
	uint8_t FXx;
	uint8_t FXy;
	uint8_t FXw = 60;
	uint8_t FXh = 100;
	uint8_t nFXbars = 5;
	
	tft.loadFont(AA_FONT_SMALL); //Set a small current font
	tft.setTextSize(1);
	tft.setTextDatum(TL_DATUM);
	tft.setTextPadding(100);
	// tft.setTextColor(TFT_YELLOW, TFT_BLACK);
	// tft.drawString(THR30II_CAB_NAMES[cab],x,y);//printAt(tft,x,y,THR30II_CAB_NAMES[cab]);  //write CAB name
	 
	tft.setTextColor(TFT_WHITE, TFT_BLACK);
	// tft.drawString(THR30II_AMP_NAMES[amp],x+(tft.width()-x)/2,y);//printAt(tft,x+(tft.width()-x)/2,y,THR30II_AMP_NAMES[amp]);  //write AMP name
	// tft.drawString(THR30II_COL_NAMES[col], x+3*(tft.width()-x)/4, y);//printAt(tft,x+3*(tft.width()-x)/4,y, THR30II_COL_NAMES[col]);  //write COL name
	

	// y+=25; //advance y-position to the bars
	 
	// tft.fillRect(x+2,y+1,13,(100-(uint16_t)control[CTRL_GAIN])*val_scale1,ST7789_BLACK); //erase GainBar
	// tft.fillRect(x+2,y+110-(uint16_t)control[CTRL_GAIN]*val_scale1,13,(uint16_t)control[CTRL_GAIN]*val_scale1,ST7789_MAGENTA); //draw GainBar

	// tft.fillRect(x+21,y+1,13,(100-(uint16_t)control[CTRL_MASTER])*val_scale1,ST7789_BLACK);//erase MasterBar
	// tft.fillRect(x+21,y+110-(uint16_t)control[CTRL_MASTER]*val_scale1,13,(uint16_t)control[CTRL_MASTER]*val_scale1,ST7789_YELLOW); //draw MasterBar

	// tft.fillRect(x+41,y+1,15,(100-(uint16_t)control[CTRL_BASS])*val_scale1,ST7789_BLACK); //erase Bass-bar
	// tft.fillRect(x+41,y+110-(uint16_t)control[CTRL_BASS]*val_scale1,15,(uint16_t)control[CTRL_BASS]*val_scale1,ST7789_CORAL);	//draw Bass bar

	// tft.fillRect(x+58,y+1,15,(100-control[CTRL_MID])*val_scale1,ST7789_BLACK);		 //erase Middle-bar
	// tft.fillRect(x+58,y+110-(uint16_t)control[CTRL_MID]*val_scale1,15,(uint16_t)control[CTRL_MID]*val_scale1,ST7789_GREEN); //draw Middle bar

	// tft.fillRect(x+76,y+1,15,(100-control[CTRL_TREBLE])*val_scale1,ST7789_BLACK);	 //erase Treble-bar
	// tft.fillRect(x+76,y+110-(uint16_t)control[CTRL_TREBLE]*val_scale1,15,(uint16_t)control[CTRL_TREBLE]*val_scale1,ST7789_LIGHTBLUE); //draw Treble bar
		
	// gain/master
  	if(boost_activated) {
    	drawBarChart(0, 80, 15, 160, TFT_THRDIMORANGE, TFT_THRORANGE, "G", control[CTRL_GAIN]);
  	} else {
    	drawBarChart(0, 80, 15, 160, TFT_THRBROWN, TFT_THRCREAM, "G", control[CTRL_GAIN]);
  	}
  	drawBarChart(15, 80, 15, 160, TFT_THRBROWN, TFT_THRCREAM, "M", control[CTRL_MASTER]);

	// EQ (B/M/T)
  	drawEQChart(30, 80, 30, 160, TFT_THRBROWN, TFT_THRCREAM, "EQ", control[CTRL_BASS], control[CTRL_MID], control[CTRL_TREBLE]);



	// Amp/Cabinet
	switch(col)
	{
		case BOUTIQUE:
		tft.setTextColor(TFT_BLUE, TFT_BLACK);
		rgbcolour = strip.gamma32(strip.Color(0,0,127));	//Select colour
		break;
		case MODERN:
		tft.setTextColor(TFT_GREEN, TFT_BLACK);
		rgbcolour = strip.gamma32(strip.Color(0,127,0));	//Select colour
		break;
		case CLASSIC:
		tft.setTextColor(TFT_RED, TFT_BLACK);
		rgbcolour = strip.gamma32(strip.Color(127,0,0));	//Select colour
		break;
	}
  	drawAmpUnit(60, 80, 240, 60, TFT_THRCREAM, TFT_THRBROWN, "Amp", col, amp, cab);
	strip.setPixelColor(1, rgbcolour);	//Set pixel's color (in RAM)
	strip.show();	//Update strip to match



	// FX1 Compressor
	utilparams[0] = compressor_setting[CO_SUSTAIN];
	utilparams[1] = compressor_setting[CO_LEVEL];
  	if(THR_Values.unit[COMPRESSOR]) {
		drawUtilUnit(60, 140, 60, 50, 1, TFT_THRWHITE, TFT_THRDARKGREY, "Comp", utilparams);
	} else {
		drawUtilUnit(60, 140, 60, 50, 1, TFT_THRDARKGREY, TFT_THRVDARKGREY, "Comp", utilparams);
	}
	


  	// Gate
	utilparams[0] = gate_setting[GA_THRESHOLD];
	utilparams[1] = gate_setting[GA_DECAY];
  	if(THR_Values.unit[GATE]) {
		drawUtilUnit(60, 190, 60, 50, 0, TFT_THRYELLOW, TFT_THRDIMYELLOW, "Gate", utilparams);
	} else {
		drawUtilUnit(60, 190, 60, 50, 0, TFT_THRDIMYELLOW, TFT_THRVDARKGREY, "Gate", utilparams);
	}



	// FX2 Effect (Chorus/Flanger/Phaser/Tremolo)
	switch(effecttype)
	{
		case CHORUS:
			FXtitle = "Chor";	// Set FX unit title
			if(unit[EFFECT]) // if FX2 activated
			{
				rgbcolour = strip.gamma32(strip.Color(0,195,95));	//Select LED colour
				FXbgcolour = TFT_THRFORESTGREEN;
				FXfgcolour = TFT_THRDIMFORESTGREEN;
			}
			else // if FX2 deactivated
			{
				rgbcolour = strip.gamma32(strip.Color(0,69,44));	//Select LED colour
				FXbgcolour = TFT_THRDIMFORESTGREEN;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = effect_setting[CHORUS][CH_SPEED];
			FXparams[1] = effect_setting[CHORUS][CH_DEPTH];
			FXparams[2] = effect_setting[CHORUS][CH_PREDELAY];
			FXparams[3] = effect_setting[CHORUS][CH_FEEDBACK];
			FXparams[4] = effect_setting[CHORUS][CH_MIX];
			nFXbars = 5;  
		break;

		case FLANGER: 
			FXtitle = "Flan";	// Set FX unit title
			if(unit[EFFECT]) // if FX2 activated
			{
				rgbcolour = strip.gamma32(strip.Color(95,255,0));	//Select LED colour
				FXbgcolour = TFT_THRLIME;
				FXfgcolour = TFT_THRDIMLIME;
			}
			else // if FX2 deactivated
			{
				rgbcolour  = strip.gamma32(strip.Color(44,69,0));	//Select LED colour
				FXbgcolour = TFT_THRDIMLIME;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = effect_setting[FLANGER][FL_SPEED];
			FXparams[1] = effect_setting[FLANGER][FL_DEPTH];
			FXparams[2] = effect_setting[FLANGER][FL_MIX];
			FXparams[3] = 0;
			FXparams[4] = 0;
			nFXbars = 3;
		break;

		case PHASER:
			FXtitle = "Phas";	// Set FX unit title
			if(unit[EFFECT]) // if FX2 activated
			{
				rgbcolour = strip.gamma32(strip.Color(191,255,0));	//Select LED colour
				FXbgcolour = TFT_THRLEMON;
				FXfgcolour = TFT_THRDIMLEMON;
			}
			else // if FX2 deactivated
			{
				rgbcolour  = strip.gamma32(strip.Color(69,78,0));	//Select LED colour
				FXbgcolour = TFT_THRDIMLEMON;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = effect_setting[PHASER][PH_SPEED];
			FXparams[1] = effect_setting[PHASER][PH_FEEDBACK];
			FXparams[2] = effect_setting[PHASER][PH_MIX];
			FXparams[3] = 0;
			FXparams[4] = 0;
			nFXbars = 3;		  
		break;		

		case TREMOLO:
			FXtitle = "Trem";	// Set FX unit title
			if(unit[EFFECT]) // if FX2 activated
			{
				rgbcolour = strip.gamma32(strip.Color(255,191,0));	//Select LED colour
				FXbgcolour = TFT_THRMANGO;
				FXfgcolour = TFT_THRDIMMANGO;
			}
			else // if FX2 deactivated
			{
				rgbcolour  = strip.gamma32(strip.Color(78,55,0));	//Select LED colour
				FXbgcolour = TFT_THRDIMMANGO;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = effect_setting[TREMOLO][TR_SPEED];
			FXparams[1] = effect_setting[TREMOLO][TR_DEPTH];
			FXparams[2] = effect_setting[TREMOLO][TR_MIX];
			FXparams[3] = 0;
			FXparams[4] = 0;
			nFXbars = 3;
		break;
	}  //of switch(effecttype)
		
	strip.setPixelColor(7, rgbcolour);	//Set pixel's color (in RAM)
	strip.show();	//Update strip to match
	
	FXx = 120;	// set FX unit position
	FXy = 140;
	drawFXUnit(FXx, FXy, FXw, FXh, FXbgcolour, FXfgcolour, FXtitle, nFXbars, FXparams);



  	// FX3 Echo (Tape Echo/Digital Delay)
  	switch(echotype)
	{
		case TAPE_ECHO:
			FXtitle = "Tape";	// Set FX unit title
			if(unit[ECHO]) // if FX3 activated
			{
				rgbcolour = strip.gamma32(strip.Color(0,0,255));	//Select LED colour
				FXbgcolour = TFT_THRROYALBLUE;
				FXfgcolour = TFT_THRDIMROYALBLUE;
			}
			else // if FX2 deactivated
			{
				rgbcolour = strip.gamma32(strip.Color(0,0,127));	//Select LED colour
				FXbgcolour = TFT_THRDIMROYALBLUE;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = echo_setting[TAPE_ECHO][TA_TIME];
			FXparams[1] = echo_setting[TAPE_ECHO][TA_FEEDBACK];
			FXparams[2] = echo_setting[TAPE_ECHO][TA_BASS];
			FXparams[3] = echo_setting[TAPE_ECHO][TA_TREBLE];
			FXparams[4] = echo_setting[TAPE_ECHO][TA_MIX];
			nFXbars = 5;  
		break;

		case DIGITAL_DELAY:
			FXtitle = "D.D.";	// Set FX unit title
			if(unit[ECHO]) // if FX3 activated
			{
				rgbcolour = strip.gamma32(strip.Color(0,191,255));	//Select LED colour
				FXbgcolour = TFT_THRSKYBLUE;
				FXfgcolour = TFT_THRDIMSKYBLUE;
			}
			else // if FX3 deactivated
			{
				rgbcolour = strip.gamma32(strip.Color(0,95,127));	//Select LED colour
				FXbgcolour = TFT_THRDIMSKYBLUE;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = echo_setting[DIGITAL_DELAY][DD_TIME];
			FXparams[1] = echo_setting[DIGITAL_DELAY][DD_FEEDBACK];
			FXparams[2] = echo_setting[DIGITAL_DELAY][DD_BASS];
			FXparams[3] = echo_setting[DIGITAL_DELAY][DD_TREBLE];
			FXparams[4] = echo_setting[DIGITAL_DELAY][DD_MIX];
			nFXbars = 5;  
		break;
	}	//of switch(effecttype)
		
	strip.setPixelColor(8, rgbcolour);	//Set pixel's color (in RAM)
	strip.show();	//Update strip to match
	
	FXx = 180;	// set FX unit position
	FXy = 140;
	drawFXUnit(FXx, FXy, FXw, FXh, FXbgcolour, FXfgcolour, FXtitle, nFXbars, FXparams);



  	// FX4 Reverb (Spring/Room/Plate/Hall)
	switch(reverbtype)
	{
		case SPRING:
			FXtitle = "Spr";	// Set FX unit title
			if(unit[REVERB]) // if FX4 activated
			{
				rgbcolour = strip.gamma32(strip.Color(255,0,0));	//Select LED colour
				FXbgcolour = TFT_THRRED;
				FXfgcolour = TFT_THRDIMRED;
			}
			else // if FX4 deactivated
			{
				rgbcolour = strip.gamma32(strip.Color(78,0,0));	//Select LED colour
				FXbgcolour = TFT_THRDIMRED;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = reverb_setting[SPRING][SP_REVERB];
			FXparams[1] = reverb_setting[SPRING][SP_TONE];
			FXparams[2] = reverb_setting[SPRING][SP_MIX];
			FXparams[3] = 0;
			FXparams[4] = 0;
			nFXbars = 3;  
		break;

		case ROOM:
			FXtitle = "Room";	// Set FX unit title
			if(unit[REVERB]) // if FX4 activated
			{
				rgbcolour = strip.gamma32(strip.Color(255,0,127));	//Select LED colour
				FXbgcolour = TFT_THRMAGENTA;
				FXfgcolour = TFT_THRDIMMAGENTA;
			}
			else // if FX4 deactivated
			{
				rgbcolour = strip.gamma32(strip.Color(69,0,44));	//Select LED colour
				FXbgcolour = TFT_THRDIMMAGENTA;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = reverb_setting[ROOM][RO_DECAY];
			FXparams[1] = reverb_setting[ROOM][RO_PREDELAY];
			FXparams[2] = reverb_setting[ROOM][RO_TONE];
			FXparams[3] = reverb_setting[ROOM][RO_MIX];
			FXparams[4] = 0;
			nFXbars = 4;  
		break;

		case PLATE:
			FXtitle = "Plate";	// Set FX unit title
			if(unit[REVERB]) // if FX4 activated
			{
				rgbcolour = strip.gamma32(strip.Color(223,0,255));	//Select LED colour
				FXbgcolour = TFT_THRPURPLE;
				FXfgcolour = TFT_THRDIMPURPLE;
			}
			else // if FX4 deactivated
			{
				rgbcolour = strip.gamma32(strip.Color(78,0,89));	//Select LED colour
				FXbgcolour = TFT_THRDIMPURPLE;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = reverb_setting[PLATE][PL_DECAY];
			FXparams[1] = reverb_setting[PLATE][PL_PREDELAY];
			FXparams[2] = reverb_setting[PLATE][PL_TONE];
			FXparams[3] = reverb_setting[PLATE][PL_MIX];
			FXparams[4] = 0;
			nFXbars = 4;  
		break;

		case HALL:
			FXtitle = "Hall";	// Set FX unit title
			if(unit[REVERB]) // if FX4 activated
			{
				rgbcolour = strip.gamma32(strip.Color(159,0,255));	//Select LED colour
				FXbgcolour = TFT_THRVIOLET;
				FXfgcolour = TFT_THRDIMVIOLET;
			}
			else // if FX4 deactivated
			{
				rgbcolour = strip.gamma32(strip.Color(59,0,89));	//Select LED colour
				FXbgcolour = TFT_THRDIMVIOLET;
				FXfgcolour = TFT_THRVDARKGREY;
			}
			FXparams[0] = reverb_setting[HALL][HA_DECAY];
			FXparams[1] = reverb_setting[HALL][HA_PREDELAY];
			FXparams[2] = reverb_setting[HALL][HA_TONE];
			FXparams[3] = reverb_setting[HALL][HA_MIX];
			FXparams[4] = 0;
			nFXbars = 4;  
		break;
	}	//of switch(reverbtype)
		
	strip.setPixelColor(9, rgbcolour);	//Set pixel's color (in RAM)
	strip.show();	//Update strip to match
	
	FXx = 240;	// set FX unit position
	FXy = 140;
	drawFXUnit(FXx, FXy, FXw, FXh, FXbgcolour, FXfgcolour, FXtitle, nFXbars, FXparams);



  	// Exp/vol pedal positions
  	drawPPChart(300, 80, 20, 160, TFT_THRBROWN, TFT_THRCREAM, "P", pedal_1_val, pedal_2_val);

	

	//Compressor
	// tft.fillRect(x+xp[COMPRESSOR],y+89,43,20,ST7789_BLACK); //erase effect type box (Compressor)
	// tft.fillRect(x+xp[COMPRESSOR],y+1,43,85,ST7789_BLACK);  //erase effect bars box (Compressor)
	 
	// if(unit[COMPRESSOR]) //if unit activated, draw values as vertical lines
	// {
		//{0, 100}, //sustain			
		// measure=(uint8_t)(compressor_setting[CO_SUSTAIN]*val_scale2) ;
		// tft.fillRect(x+xp[COMPRESSOR]+0,y+86-measure,21,measure,ST7789_BEIGE);
		//{0, 100}, //level
		// measure=(uint8_t)(compressor_setting[CO_LEVEL]*val_scale2) ;
		// tft.fillRect(x+xp[COMPRESSOR]+21,y+86-measure,21,measure,ST7789_GREEN);
		//{0, 100}, //mix
		// measure=(uint8_t)(compressor_setting[CO_MIX]*val_scale2) ;
		// tft.fillRect(x+xp[COMPRESSOR]+29,y+86-measure,13,measure,ST7789_RED);
	// }//of Compressor activated

	
	  
	// tft.setTextPadding(0);
	// tft.unloadFont();

	//Gate
	// tft.fillRect(x+xp[GATE],y+1,42,85,ST7789_BLACK); //erase effect bars box (Gate)
	// if(unit[GATE]) //if unit activated, draw delay values as vertical lines
	// {
		//{0, 100}, //threshold
		// measure=gate_setting[GA_THRESHOLD]*val_scale2;
		// tft.fillRect(x+xp[GATE]+0,y+86-measure,21,measure,ST7789_LIGHTBLUE);
		//{0, 100}, //gate decay
		// measure=gate_setting[GA_DECAY]*val_scale2;
		// tft.fillRect(x+xp[GATE]+21,y+86-measure,21,measure,ST7789_RED);
	// }//of Gate activated

	// Dynamics LED
	switch(dyn_mode)
	{
		case Comp:
			if(unit[COMPRESSOR])
			{
				rgbcolour = strip.gamma32(strip.Color(223,223,223));	//Select colour (white)
				strip.setPixelColor(6, rgbcolour);	//Set pixel's color (in RAM)
				strip.show();	//Update strip to match
			}
			else
			{
				rgbcolour = strip.gamma32(strip.Color(69,69,69));	//Select colour (dim white)
				strip.setPixelColor(6, rgbcolour);	//Set pixel's color (in RAM)
				strip.show();	//Update strip to match
			}
		break;

		case Boost:
			if(boost_activated)
			{
				rgbcolour = strip.gamma32(strip.Color(223,95,0));	//Select colour (orange)
				strip.setPixelColor(6, rgbcolour);	//Set pixel's color (in RAM)
				strip.show();	//Update strip to match
			}
			else
			{
				rgbcolour = strip.gamma32(strip.Color(89,44,0));	//Select colour (dim orange)
				strip.setPixelColor(6, rgbcolour);	//Set pixel's color (in RAM)
				strip.show();	//Update strip to match
			}
		break;

		case Gate:
			if(unit[GATE])
			{
				rgbcolour = strip.gamma32(strip.Color(255,255,0));	//Select colour (yellow)
				strip.setPixelColor(6, rgbcolour);	//Set pixel's color (in RAM)
				strip.show();	//Update strip to match
			}
			else
			{
				rgbcolour = strip.gamma32(strip.Color(67,67,0));	//Select colour (dim yellow)
				strip.setPixelColor(6, rgbcolour);	//Set pixel's color (in RAM)
				strip.show();	//Update strip to match
			}
		break;
		
	}


	  
	String s2,s3;
	//int w;
	
	switch(_uistate)
	{
	    case UI_home_amp: //!patchActive
			//if an unchanged User Memory setting is active:	
			if(THR_Values.getActiveUserSetting()!=-1 && !THR_Values.getUserSettingsHaveChanged())
			{
				//update GUI status line
				s2=THR_Values.getPatchName();
				drawPatchName(TFT_SKYBLUE, s2);
			}
			else if(THR_Values.getUserSettingsHaveChanged())
			{
				s2=THR_Values.getPatchName()+"(*)";
				drawPatchName(ST7789_ORANGERED, s2);
			}
			else
			{
				s2="THR Panel";
				drawPatchName(TFT_SKYBLUE, s2);
			}
	 	break;

		case UI_home_patch:	//patch active
			//if an unchanged User Memory setting is active:
			if(!THR_Values.getUserSettingsHaveChanged())
			{
				if(presel_patch_id != active_patch_id)
				{
					s2 = libraryPatchNames[presel_patch_id-1];	//libraryPatchNames is 0-indexed
					drawPatchName(ST7789_ORANGE, s2);
				}
				else
				{
					s2 = libraryPatchNames[active_patch_id-1];
					drawPatchName(TFT_THRCREAM, s2);
				}
			}
			else
			{
				s2 = libraryPatchNames[active_patch_id-1]+"(*)";	//libraryPatchNames is 0-indexed
				drawPatchName(ST7789_ORANGERED, s2);
			}
		break;

		default:

		break;
	}
	 
	maskUpdate=false;  //tell local GUI, that mask is updated
}

void WorkingTimer_Tick()
{
	//Care about queued incoming messages

	if (inqueue.item_count() > 0)
	{
		SysExMessage msg (inqueue.dequeue());
		Serial.println(THR_Values.ParseSysEx(msg.getData(),msg.getSize()));
		//THR_Values.ParseSysEx(msg.getData(),msg.getSize());

		if(!maskActive)
		{
			maskActive=true;  //tell GUI to show Settings mask
			// drawStatusMask(0,85);
		}
		maskUpdate=true;  //tell GUI to update mask one time because of changed settings       
	}		

	//Care about next queued outgoing SysEx, if at least one is pending
    	
	if (outqueue.item_count() > 0)   
	{	
		Outmessage *msg = outqueue.getHeadPtr();  //& means: directly work message in queue, don't copy it
		
		if (!msg->_sent_out)  
		{  
			midi1.sendSysEx(msg->_msg.getSize(),(uint8_t *)(msg->_msg.getData()),true);
			
			Serial.println("sent out");
			msg->_sent_out = true;	
							
		}  //of "not sent out"
		else if (!msg->_needs_ack && !msg->_needs_answer)   //needs no ACK and no Answer
		{
			outqueue.dequeue();
			Serial.println("dequ no ack, no answ"); 
		}
		else  //sent out, and needs  ack   or   answer
		{
			if (msg->_needs_ack)              
			{
				if (msg->_acknowledged)
				{
					if (!msg->_needs_answer)   //ack OK, no answer needed
					{
						outqueue.dequeue();  // => ready
						Serial.println("dequ ack, no answ");						
					}
					else if (msg->_answered)   //ack OK, Answer received
					{
						outqueue.dequeue();  //=> ready
					    Serial.println("dequ ack and answ");						
					}
				}
			}
			else  if(msg->_needs_answer)   //only need Answer, no Ack
			{
				if (msg->_answered)   //Answer received
				{
					outqueue.dequeue();  //=>ready
					Serial.println("dequ no ack but answ");
				}
			}
		}
	}
}

SysExMessage::SysExMessage():Data(nullptr),Size(0) //standard constructor
{
};  

SysExMessage::~SysExMessage() //destructor
{
	delete[] Data;
	Size=0;
};  

SysExMessage::SysExMessage(const byte * data ,size_t size):Size(size) //Constructor
{
	Data=new byte[size];
	memcpy(Data,data,size);
};  
SysExMessage::SysExMessage(const SysExMessage &other ):Size(other.Size)  //Copy Constructor
{
	Data=new byte[other.Size];
	memcpy(Data,other.Data,other.Size);
}
SysExMessage::SysExMessage( SysExMessage &&other ) noexcept : Data(other.Data), Size(other.Size) //Move Constructor
{
	other.Data=nullptr;
	other.Size=0;
}
SysExMessage & SysExMessage::operator=( const SysExMessage & other ) //Copy-assignment
{
	if(&other==this) return *this;
	delete[] Data;
	Data=new byte[other.Size];
	memcpy(Data,other.Data,Size=other.Size);	   
	return *this;
} 

SysExMessage & SysExMessage::operator=( SysExMessage && other ) noexcept //Move-assignment
{
	if(&other==this) return *this;
	delete[] Data;
	Size=other.Size;
	Data=other.Data;
	other.Size=0;
	return *this;	   
} 

const byte * SysExMessage::getData()  //getter for the const byte-array
{
	return (const byte*) Data;
}

size_t SysExMessage::getSize()  //getter for the const byte-array
{
	return Size;
}

uint16_t UnitOnMap(uint16_t u)
{
	std::map<uint16_t,uint16_t>::iterator p = unitOnMap.find(u);
	if ( p !=unitOnMap.end() )//if map contains key u
	{
		return p->second; //return value for this key
	}
	else
	{
		return 0xFFFF;
	}
}

uint16_t ReverbMap(uint16_t u)
{
	std::map<uint16_t,uint16_t>::iterator p = reverbMap.find(u);
	if ( p !=reverbMap.end() )//if map contains key u
	{
		return p->second;//return value for this key
	}
	else
	{
		return 0xFFFF;
	}
}

uint16_t EchoMap(uint16_t u)
{
	std::map<uint16_t,uint16_t>::iterator p = echoMap.find(u);

	if ( p !=echoMap.end() )  //if map contains key u
	{
		return p->second;  //return value for this key
	}
	else
	{
		return 0xFFFF;
	}
}

uint16_t EffectMap(uint16_t u)
{
	std::map<uint16_t,uint16_t>::iterator p = effectMap.find(u);

	if ( p !=effectMap.end() )  //if map contains key u
	{
		return p->second;  //return value for this key
	}
	else
	{
		return 0xFFFF;
	}
}

uint16_t CompressorMap(uint16_t u)
{
	std::map<uint16_t,uint16_t>::iterator p = compressorMap.find(u);

	if ( p !=compressorMap.end() )  //if map contains key u
	{
		return p->second;  //return value for this key
	}
	else
	{
		return 0xFFFF;
	}
}

void OnSysEx(const uint8_t *data, uint16_t length, bool complete)
{
	//Serial.println("SysEx Received "+String(length));
	
	//complete?Serial.println("- completed "):Serial.println("- continued ");
	memcpy(currentSysEx+cur,data, length);
	cur+=length;

	if(complete)
	{
		cur_len=cur+1;
		cur=0;
		::complete=true;
	}
	else
	{
		
	}
}
