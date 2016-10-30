
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
//	A0 				- Gas sensor
//  D4, D5, D6 		- RGB Led
//  D3 				- Buzzer
//  D2 				- DHT Sensort
//  + RF Module
//
//------------------------------------------------------------------------------

#include <Wire.h>
#include <SPI.h>
#include <MySensor.h>
#include <Time.h>
#include <Buzzer.h>
#include <Motion.h>
#include <Gas.h>

#include <DHT.h>
#include <Led.h>

//------------------------------------------------------------------------------
// Constants

//------------------------------------------------------------------------------
// sensor pins
#define HUMIDITY_SENSOR_DIGITAL_PIN 2
#define MOTION_SENSOR_DIGITAL_PIN   7
#define GAS_SENSOR_ANALOG_PIN       A0
#define LED_B_DIGITAL_PIN			4
#define LED_G_DIGITAL_PIN			5
#define LED_R_DIGITAL_PIN			6
#define BUZZER_DIGITAL_PIN   		3

//------------------------------------------------------------------------------
// sensor objects
MySensor 	gw;
DHT 		humiditySensor;

Gas 		gas;
Buzzer 		buzzer;
Motion   	motion;
Led 		ledR;
Led 		ledG;
Led 		ledB;

//------------------------------------------------------------------------------
// global variables

float 			lastHumidityValue			= 0;
float  			lastTemperatureValue 		= 0;
unsigned long 	lastHumiditySensorReadTime 	= 0;

uint8_t 		lastGasValue  				= 0;

//------------------------------------------------------------------------------
// sensors Id's
#define CHILD_ID_HUM_DHT 	1
#define CHILD_ID_TEMP_DHT 	2

#define CHILD_ID_BUZZER 	3

#define CHILD_ID_MOTION		4
#define CHILD_ID_GAS		5

#define CHILD_ID_LED_R		6
#define CHILD_ID_LED_G		7
#define CHILD_ID_LED_B		8

//------------------------------------------------------------------------------
// Messages
MyMessage msgHum		(CHILD_ID_HUM_DHT, 	V_HUM);
MyMessage msgTemp		(CHILD_ID_TEMP_DHT,     V_TEMP);

MyMessage msgGas		(CHILD_ID_GAS, 		V_VAR1);
MyMessage msgMotion		(CHILD_ID_MOTION,	V_ARMED);

MyMessage msgBuzzer		(CHILD_ID_BUZZER, 	V_VAR1);

MyMessage msgLedR		(CHILD_ID_LED_R, 	V_LIGHT_LEVEL);
MyMessage msgLedG		(CHILD_ID_LED_G, 	V_LIGHT_LEVEL);
MyMessage msgLedB		(CHILD_ID_LED_B, 	V_LIGHT_LEVEL);

//------------------------------------------------------------------------------
void setupSensors () {

 // setup dht (humiditySensor)
 humiditySensor.setup(HUMIDITY_SENSOR_DIGITAL_PIN);

   motion.begin(MOTION_SENSOR_DIGITAL_PIN);
   gas.begin(GAS_SENSOR_ANALOG_PIN);

   ledR.begin(LED_R_DIGITAL_PIN);

   ledR.set(gw.loadState(CHILD_ID_LED_R));

   ledG.begin(LED_G_DIGITAL_PIN);
   ledG.set(gw.loadState(CHILD_ID_LED_G));

   ledB.begin(LED_B_DIGITAL_PIN);
   ledB.set(gw.loadState(CHILD_ID_LED_B));

   buzzer.begin(BUZZER_DIGITAL_PIN);
}

//------------------------------------------------------------------------------
void setup()
{

 Serial.begin(9600);
 delay(1000);
 Wire.begin();

 gw.begin(incomingMessage, AUTO, true);
 gw.sendSketchInfo("Multi-sensor node", "1.0");

 setupSensors();

 gw.present(CHILD_ID_HUM_DHT,  S_HUM);
 gw.present(CHILD_ID_TEMP_DHT, S_TEMP);

 gw.present(CHILD_ID_BUZZER,   S_LIGHT);

 gw.present(CHILD_ID_MOTION,   S_LIGHT);
 gw.present(CHILD_ID_GAS,      S_LIGHT);

 gw.present(CHILD_ID_LED_R,    S_LIGHT);
 gw.present(CHILD_ID_LED_G,    S_LIGHT);
 gw.present(CHILD_ID_LED_B,    S_LIGHT);

}


//------------------------------------------------------------------------------
void loop()
{
 unsigned long currentTime 	= millis ();

 updateHumiditySensorValues 	(currentTime);
 updateMotionSensorValues	(currentTime);
 updateGasSensorValues		(currentTime);

 gw.process();
 //buzzer.update();
}

bool lastMotionState = false;

//------------------------------------------------------------------------------
void updateMotionSensorValues (unsigned long currentTime) {

 bool motionState = motion.get();

 if (motionState != lastMotionState)	{
   lastMotionState = motionState;
   gw.send(msgMotion.set(motionState));
 }
}

//------------------------------------------------------------------------------
void updateGasSensorValues (unsigned long currentTime) {
 uint8_t gasValue = gas.get();

 if (!isnan(gasValue) &&  byteValuesChanged(gasValue, lastGasValue)) {
   lastGasValue = gasValue;
   gw.send(msgGas.set(gasValue, 1));
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

 int humiditySensorDelay = humiditySensor.getMinimumSamplingPeriod();


 if (currentTime > (lastHumiditySensorReadTime  + humiditySensorDelay)) {
   lastHumiditySensorReadTime = currentTime;

   float temperature = humiditySensor.getTemperature();

   if (!isnan(temperature) &&  floatValuesChanged(temperature, lastTemperatureValue)) {
     lastTemperatureValue = temperature;
     gw.send(msgTemp.set(temperature, 1));
   }

   float humidity = humiditySensor.getHumidity();
   if (!isnan(humidity) &&  floatValuesChanged(humidity, lastHumidityValue)) {
     lastHumidityValue = humidity;
     gw.send(msgHum.set(humidity, 1));
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
 //return (uint8_t)(valueOne/100) != (uint8_t)(valueTwo/100);
}


//------------------------------------------------------------------------------
void incomingMessage(const MyMessage &message) {

 // 1. handle buzzer values
 // 2. handle r, g, b leg values

 if (message.sensor == CHILD_ID_BUZZER) {
       buzzer.buzz(message.getInt(), 200);
 }
 else if (message.sensor == CHILD_ID_LED_R && message.type == V_LIGHT) {
   ledR.set(message.getByte());
   gw.saveState(message.sensor, message.getByte());
 }
 else if (message.sensor == CHILD_ID_LED_G && message.type == V_LIGHT) {
   ledG.set(message.getByte());
   gw.saveState(message.sensor, message.getByte());
 }
 else if (message.sensor == CHILD_ID_LED_B && message.type == V_LIGHT) {
   ledB.set(message.getByte());
   gw.saveState(message.sensor, message.getByte());

 }

}
