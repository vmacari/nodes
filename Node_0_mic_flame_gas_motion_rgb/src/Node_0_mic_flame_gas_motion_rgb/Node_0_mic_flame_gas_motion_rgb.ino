//------------------------------------------------------------------------------
//  BOARD DESCRIPTION:
// 	Sensor node N0 with mic, flame, gas and motion sensors. With RGB led and with
// 	radio module.
//
// Node pinout (sensors):
// 
//  A0 - microphone
//  A1 - flame diode
//  A2 - gas sensor MQ-2
//  A3 - motion sensor
//  D3 (b), D4 (G), D5 (R) - RGB led
//  RF-242 module
//
 //------------------------------------------------------------------------------

#define MY_LEDS_BLINKING_FEATURE
#define MY_DEBUG 
#define MY_RADIO_NRF24
#define MY_NODE_ID        3

#include <Wire.h>
#include <SPI.h>
#include <MySensors.h>
#include <Time.h>
#include <MQ2.h>


//------------------------------------------------------------------------------
// Constants (pin mapping)
#define LED_R_PIN 		    3
#define LED_G_PIN 		    4
#define LED_B_PIN  		    5

#define MIC_PIN 	        A0
#define FLAME_PIN 	      A1
#define GAS_PIN 	        A2      // MQ2
#define MOTION_PIN 	      A3

//------------------------------------------------------------------------------
// Constants (app conf)
#define PRESENTATION_DELAY          50
#define MAX_MOTION_UPDATE_COUNTER   10
#define MAX_GAS_UPDATE_COUNTER      10
#define MAX_FLAME_UPDATE_COUNTER    10
#define MAX_MIC_UPDATE_COUNTER      10

//------------------------------------------------------------------------------
// sensor hardware interface objects
MQ2     gas(GAS_PIN);

//------------------------------------------------------------------------------
// sensors Id's

#define CHILD_ID_MOTION     1
#define CHILD_ID_GAS        2
#define CHILD_ID_FLAME      3
#define CHILD_ID_MICROPHONE 4
#define CHILD_ID_LED_R      5
#define CHILD_ID_LED_G      6
#define CHILD_ID_LED_B      7

//------------------------------------------------------------------------------
// Messages
MyMessage msgMotion       (CHILD_ID_MOTION,     V_TRIPPED);
MyMessage msgGas          (CHILD_ID_GAS,        V_LEVEL);
MyMessage msgFlame        (CHILD_ID_FLAME,      V_LIGHT_LEVEL);
MyMessage msgMicrophone   (CHILD_ID_MICROPHONE, V_LEVEL);

MyMessage msgLedR         (CHILD_ID_LED_R,      V_LIGHT_LEVEL);
MyMessage msgLedG         (CHILD_ID_LED_G,      V_LIGHT_LEVEL);
MyMessage msgLedB         (CHILD_ID_LED_B,      V_LIGHT_LEVEL);

//------------------------------------------------------------------------------
// Globals
boolean lastMotionValue;
float   lastGasValue;
int     lastFlameValue;
int     lastMicValue;

int motionUpdateCounter;
int gasUpdateCounter;
int flameUpdateCounter;
int micUpdateCounter;

//------------------------------------------------------------------------------
void presentation () {
  
  sendSketchInfo("MQ2,Motion,Flame,Mic,rgb", "1.0");            delay(PRESENTATION_DELAY);
  
  present(CHILD_ID_MOTION,   S_MOTION, "Motion sensor");          delay(PRESENTATION_DELAY);
  present(CHILD_ID_GAS,      S_GAS, "MQ-2 GAs sensor");        delay(PRESENTATION_DELAY);
  present(CHILD_ID_FLAME,    S_LIGHT_LEVEL, "Flame Sensor");        delay(PRESENTATION_DELAY);
  present(CHILD_ID_MICROPHONE,    S_SOUND, "Sound Sensor");        delay(PRESENTATION_DELAY);

  present(CHILD_ID_LED_R,    S_DIMMER, "Actuator Red led");       delay(PRESENTATION_DELAY);
  present(CHILD_ID_LED_G,    S_DIMMER, "Actuator Green led");     delay(PRESENTATION_DELAY);
  present(CHILD_ID_LED_B,    S_DIMMER, "Actuator Blue led");      delay(PRESENTATION_DELAY);
}


//------------------------------------------------------------------------------
void setup()
{
   pinMode(MOTION_PIN,                INPUT);      
   pinMode(GAS_PIN,                   INPUT);      
   pinMode(FLAME_PIN,                 INPUT);      
   pinMode(MIC_PIN,                   INPUT);      
   
   pinMode(LED_R_PIN,                 OUTPUT);      
   pinMode(LED_G_PIN,                 OUTPUT);      
   pinMode(LED_B_PIN,                 OUTPUT);      

   analogWrite(LED_R_PIN, loadState(CHILD_ID_LED_R));
   analogWrite(LED_G_PIN, loadState(CHILD_ID_LED_G));
   analogWrite(LED_R_PIN, loadState(CHILD_ID_LED_B));
   gas.begin();   

   lastMotionValue      = false;
   lastGasValue         = 0.0f;
   lastFlameValue       = 0;
   lastMicValue         = 0;

   motionUpdateCounter  = 0;
   gasUpdateCounter     = 0;
   flameUpdateCounter   = 0;
   micUpdateCounter     = 0;
}

//------------------------------------------------------------------------------
void loop()
{
  unsigned long currentTime   = millis ();
  
  lastMotionValue = readMotionSensor(currentTime, lastMotionValue);
  lastGasValue    = readGasSensor   (currentTime, lastGasValue);
  lastFlameValue  = readFlameSensor (currentTime, lastFlameValue);
  lastMicValue    = readMicSensor   (currentTime, lastMicValue);
}

//------------------------------------------------------------------------------
boolean readMotionSensor (unsigned long currentTime, boolean oldValue) {
   boolean value = (digitalRead(MOTION_PIN) == HIGH);
   if (!isnan(value) && value != oldValue) {
    send(msgMotion.set(value));
    motionUpdateCounter = 0;
    return value;
  }
  
  if (motionUpdateCounter ++ > MAX_MOTION_UPDATE_COUNTER) {
    send(msgMotion.set(oldValue));
    motionUpdateCounter = 0;
  }
  
  return oldValue;
}  

//------------------------------------------------------------------------------
float readGasSensor (unsigned long currentTime, float oldValue) {

  gas.read(true);
  float value = gas.readLPG();
  
  if (!isnan(value) && value != oldValue) {
    send(msgGas.set(value, 2));
    gasUpdateCounter = 0;
    return value;
  }
  
  if (gasUpdateCounter ++ > MAX_GAS_UPDATE_COUNTER) {
    send(msgGas.set(oldValue, 2));
    gasUpdateCounter = 0;
  }
  
  return oldValue;
}  

//------------------------------------------------------------------------------
int readFlameSensor (unsigned long currentTime, int oldValue) {
 
  int value = analogRead(FLAME_PIN);
  
  if (!isnan(value) && value != oldValue) {
    send(msgFlame.set(-value));
    flameUpdateCounter = 0;
    return value;
  }
  
  if (flameUpdateCounter ++ > MAX_FLAME_UPDATE_COUNTER) {
    send(msgFlame.set(-oldValue));
    flameUpdateCounter = 0;
  }
  
  return oldValue;
}  

//------------------------------------------------------------------------------
#define MIC_DELTA 5
int readMicSensor (unsigned long currentTime, int oldValue) {
 
  int value = analogRead(MIC_PIN);
  
  if (!isnan(value) && value != oldValue) {
    send(msgMicrophone.set(value));
    micUpdateCounter = 0;
    return value;
  }
  
  if (micUpdateCounter ++ > MAX_MIC_UPDATE_COUNTER) {
    send(msgMicrophone.set(oldValue));
    micUpdateCounter = 0;
  }
  
  return oldValue;
}  


//------------------------------------------------------------------------------
#define IR_NODE           103
#define DISPLAY_NODE      102


void receive(const MyMessage &message) {
/*
if(mGetAck(message)){
  return;
}
  
if (message.sender== IR_NODE) {
      if (message.type == S_GAS) {
        msgGas.destination = DISPLAY_NODE;
        msgGas.type = S_GAS;
        send(msgGas.set(lastGasValue, 2));

      } else if (message.type == S_LIGHT_LEVEL) {
        msgFlame.destination = DISPLAY_NODE;
        msgFlame.type = S_LIGHT_LEVEL;
        send(msgFlame.set(lastFlameValue));
      } else if (message.type == S_SOUND) {
        
        msgMicrophone.destination = DISPLAY_NODE;
        msgMicrophone.type = S_SOUND;
        send(msgMicrophone.set(lastMicValue));
      }
        
    }
  else*/ if (message.sensor == CHILD_ID_LED_R && message.type == V_LIGHT) {
     analogWrite(LED_R_PIN, message.getByte());
     saveState(message.sensor, message.getByte());
    }
    else if (message.sensor == CHILD_ID_LED_G && message.type == V_LIGHT) {
      analogWrite(LED_G_PIN, message.getByte());
      saveState(message.sensor, message.getByte());
    }
    else if (message.sensor == CHILD_ID_LED_B && message.type == V_LIGHT) {
     analogWrite(LED_B_PIN, message.getByte());
     saveState(message.sensor, message.getByte());
    }


    
}

