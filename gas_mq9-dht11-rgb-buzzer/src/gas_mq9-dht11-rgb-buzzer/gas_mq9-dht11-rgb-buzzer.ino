
//------------------------------------------------------------------------------
//	BOARD DESCRIPTION:
// 		Kithcen board mainly designed to monitor Gas, Temperature and Humidity values
//  It will triger alarms if the gas level is High and no motion around.
// When gas level is Hi and there is some motion, it will blink a led
// All the data are continuously sent to a monitoring linux box
//
//
//  BOARD WIRING:
//	D7 				- (?) Motion sensor
//	A0 				- Gas sensor (MQ9)
//  D4, D5, D6 		- RGB Led
//  D3 				- Buzzer
//  D2 				- DHT Sensort
//  + RF Module
//

//------------------------------------------------------------------------------
//  MQ-9
//
// Sensitive for Carbon Monoxide, flammable gasses.
// The heater uses an alternating voltage of 5V and 1.5V. It depends on the gases how to use that alternating voltage. If only Carbon Monoxide is tested, the heater can be set at 1.5V.
// The MQ309A (also on this page) is like this sensor, but uses a lower heater voltage.
// Search for datasheet: http://duckduckgo.com/?q=%22mq-9%22+gas+sensor+filetype%3Apdf
//
//------------------------------------------------------------------------------

#define MY_LEDS_BLINKING_FEATURE
#define MY_DEBUG 
#define MY_RADIO_NRF24
#define MY_NODE_ID        2

#include <Wire.h>
#include <SPI.h>
#include <MySensors.h>
#include <Time.h>
#include <MQ2.h>
#include <DHT.h>
#include <Tone.h>

//------------------------------------------------------------------------------
// Constants
#define MAX_GAS_UPDATE_COUNTER      10
#define MAX_HUM_UPDATE_COUNTER      10
#define MAX_TEMP_UPDATE_COUNTER     10
#define MAX_MOTION_UPDATE_COUNTER   10

//------------------------------------------------------------------------------
// sensor pins
#define HUMIDITY_SENSOR_DIGITAL_PIN 2
#define MOTION_SENSOR_DIGITAL_PIN   7
#define GAS_SENSOR_ANALOG_PIN       A0
#define LED_B_DIGITAL_PIN			      4
#define LED_G_DIGITAL_PIN			      5
#define LED_R_DIGITAL_PIN		      	6
#define BUZZER_DIGITAL_PIN   		    3

//------------------------------------------------------------------------------
// sensor hardware interface objects
DHT 		humiditySensor(HUMIDITY_SENSOR_DIGITAL_PIN, DHT11);
MQ2     gas(GAS_SENSOR_ANALOG_PIN);
Tone    buzzer;

//------------------------------------------------------------------------------
// global variables
float 			    lastHumidityValue			          = 0;
float  			    lastTemperatureValue 		        = 0;
unsigned long 	lastHumiditySensorReadTime 	    = 0;
uint8_t 		    lastGasValue  				          = 0;

int             humidityUpdateCouner            = 0;
int             temperatureUpdateCounter        = 0;
int             gasUpdateCounter                = 0;
int             motionUpdateCounter             = 0;

//------------------------------------------------------------------------------
// sensors Id's
#define CHILD_ID_HUM_DHT 	  1
#define CHILD_ID_TEMP_DHT 	2


#define CHILD_ID_BUZZER 	  3

#define CHILD_ID_MOTION		  4
#define CHILD_ID_GAS		    5

#define CHILD_ID_LED_R		  6
#define CHILD_ID_LED_G		  7
#define CHILD_ID_LED_B		  8

//------------------------------------------------------------------------------
// Messages
MyMessage msgHum		(CHILD_ID_HUM_DHT, 	  V_HUM);
MyMessage msgTemp		(CHILD_ID_TEMP_DHT,   V_TEMP);

MyMessage msgGas		(CHILD_ID_GAS, 		    V_LEVEL);
MyMessage msgMotion		(CHILD_ID_MOTION,	  V_TRIPPED);

MyMessage msgBuzzer		(CHILD_ID_BUZZER, 	V_PERCENTAGE);

MyMessage msgLedR		(CHILD_ID_LED_R, 	    V_PERCENTAGE);
MyMessage msgLedG		(CHILD_ID_LED_G, 	    V_PERCENTAGE);
MyMessage msgLedB		(CHILD_ID_LED_B, 	    V_PERCENTAGE);

//------------------------------------------------------------------------------
#define PRESENTATION_DELAY 50
void presentation () {
  
  sendSketchInfo("Gas,Motion,DHT,buz,rgb", "1.0");        delay(PRESENTATION_DELAY);
  
  present(CHILD_ID_HUM_DHT,  S_HUM,  "DHT humidity");            delay(PRESENTATION_DELAY);
  present(CHILD_ID_TEMP_DHT, S_TEMP, "DHT temperature");         delay(PRESENTATION_DELAY);
  present(CHILD_ID_BUZZER,   S_DIMMER, "Buzzer");                 delay(PRESENTATION_DELAY);

  present(CHILD_ID_MOTION,   S_MOTION, "Motion sensor");          delay(PRESENTATION_DELAY);
  present(CHILD_ID_GAS,      S_GAS, "MQ-9 GAs sensor");        delay(PRESENTATION_DELAY);

  present(CHILD_ID_LED_R,    S_DIMMER, "Actuator Red led");       delay(PRESENTATION_DELAY);
  present(CHILD_ID_LED_G,    S_DIMMER, "Actuator Green led");     delay(PRESENTATION_DELAY);
  present(CHILD_ID_LED_B,    S_DIMMER, "Actuator Blue led");      delay(PRESENTATION_DELAY);
}
//------------------------------------------------------------------------------
void setup()
{
   pinMode(MOTION_SENSOR_DIGITAL_PIN, INPUT);      // sets the motion sensor digital pin as input
   gas.begin();

   pinMode(LED_B_DIGITAL_PIN, OUTPUT);
   pinMode(LED_G_DIGITAL_PIN, OUTPUT);
   pinMode(LED_R_DIGITAL_PIN, OUTPUT);

   analogWrite(LED_R_DIGITAL_PIN, loadState(CHILD_ID_LED_R));
   analogWrite(LED_G_DIGITAL_PIN, loadState(CHILD_ID_LED_G));
   analogWrite(LED_B_DIGITAL_PIN, loadState(CHILD_ID_LED_B));

   buzzer.begin(BUZZER_DIGITAL_PIN);
} 

//------------------------------------------------------------------------------
void loop()
{
 unsigned long currentTime 	= millis ();

 updateHumiditySensorValues 	(currentTime);
 updateMotionSensorValues	(currentTime);
 updateGasSensorValues		(currentTime);
}

bool lastMotionState = false;

//------------------------------------------------------------------------------
void updateMotionSensorValues (unsigned long currentTime) {
 bool motionState = digitalRead(MOTION_SENSOR_DIGITAL_PIN) == HIGH; 
 
 if (motionState != lastMotionState || motionUpdateCounter++ > MAX_MOTION_UPDATE_COUNTER)	{
   lastMotionState = motionState;
   motionUpdateCounter = 0;
   send(msgMotion.set(motionState));
 }
}

//------------------------------------------------------------------------------
#define GAS_DELTA 20
void updateGasSensorValues (unsigned long currentTime) {

  gas.read(true);
  float lpgValue =  gas.readLPG();
  if (!isnan(lpgValue) && abs(lpgValue - lastGasValue) > GAS_DELTA) {
     lastGasValue = lpgValue;
     Serial.print("--------- Sending GAS data: ");
     Serial.println(lastGasValue);
     send(msgGas.set(lpgValue, 2));
  }
  else if (gasUpdateCounter ++ > MAX_GAS_UPDATE_COUNTER) {
    gasUpdateCounter = 0;
    send(msgGas.set(lastGasValue, 2));
  }

//        if (gasValue > 120) {
//          ledR.on();
////          buzzer.on();
//          buzzer.buzz(1175, 200);
//        } else {
//          ledR.off();
////          buzzer.off();
//        }
}

//------------------------------------------------------------------------------
void updateHumiditySensorValues (unsigned long currentTime) {

 if (currentTime > (lastHumiditySensorReadTime  + 500)) { // humiditySensorDelay)) {
   
   lastHumiditySensorReadTime = currentTime;
   float temperature = humiditySensor.readTemperature();

   if (!isnan(temperature) &&  floatValuesChanged(temperature, lastTemperatureValue)) {
     lastTemperatureValue = temperature;
     send(msgTemp.set(temperature, 1));
   }

   float humidity = humiditySensor.readHumidity();
   if (!isnan(humidity) &&  floatValuesChanged(humidity, lastHumidityValue)) {
     lastHumidityValue = humidity;
     send(msgHum.set(humidity, 1));
   }
 }

}

//------------------------------------------------------------------------------
//	TODO: Cut of decimals, to not react to minor temperature changes
bool floatValuesChanged(float valueOne, float valueTwo) {
 return (int)valueOne != (int)valueTwo;
}

//------------------------------------------------------------------------------
//	TODO: add a range
bool byteValuesChanged(uint8_t valueOne, uint8_t valueTwo) {
   return (abs(valueOne - valueTwo) > 10);
}

//------------------------------------------------------------------------------

#define IR_NODE           103
#define DISPLAY_NODE      102

void receive(const MyMessage &message) {


  if (message.sender== IR_NODE) {
      if (message.type == S_TEMP) {

        msgTemp.destination = DISPLAY_NODE;
        msgTemp.type = S_TEMP;
        send(msgTemp.set(lastTemperatureValue, 2));

      } else if (message.type == S_HUM) {
        msgHum.destination = DISPLAY_NODE;
        msgHum.type = S_HUM;
        send(msgTemp.set(lastHumidityValue, 2));
      }
        
    }
    else 
    // 1. handle buzzer values
    // 2. handle r, g, b leg values
    if (message.sensor == CHILD_ID_BUZZER) {
         buzzer.play(message.getInt(), 200);
    }
    else if (message.sensor == CHILD_ID_LED_R && message.type == V_PERCENTAGE) {
     analogWrite(LED_R_DIGITAL_PIN, message.getByte());
     saveState(message.sensor, message.getByte());
    }
    else if (message.sensor == CHILD_ID_LED_G && message.type == V_PERCENTAGE) {
      analogWrite(LED_G_DIGITAL_PIN, message.getByte());
      saveState(message.sensor, message.getByte());
    }
    else if (message.sensor == CHILD_ID_LED_B && message.type == V_PERCENTAGE) {
     analogWrite(LED_B_DIGITAL_PIN, message.getByte());
     saveState(message.sensor, message.getByte());
    }
}
