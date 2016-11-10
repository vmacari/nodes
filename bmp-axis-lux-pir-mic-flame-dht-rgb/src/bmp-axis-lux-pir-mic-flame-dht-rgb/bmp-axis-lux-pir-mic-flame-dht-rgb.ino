
//-----------------------------------------------------------------------------
// Hardware description:
//  - RF
//  - BH1750, LUX, XYZ - I2C
//  - Infrared sensor  - A6
//  - Microphone - A7
//  - PIR - D2
//  - DHT - D7
//  - RGB - D3, D5, D6
//-----------------------------------------------------------------------------
// My sensors definitions

#define NODE_ID_XYZ_HUM    4
#define MY_LEDS_BLINKING_FEATURE
#define MY_DEBUG 

#define MY_RADIO_NRF24
#define MY_NODE_ID        NODE_ID_XYZ_HUM
//#define MY_RF24_CHANNEL   11

#define DHTTYPE DHT11

//-----------------------------------------------------------------------------
#include <SPI.h>
#include <MySensors.h>
#include <BH1750.h> // light level
#include <Wire.h>
#include <DHT.h>
#include <DHT_U.h> // temp, hum
#include <Adafruit_BMP085.h> // presure, temp
#include <Adafruit_HMC5883_U.h> // xyz axis

//-----------------------------------------------------------------------------
// generic constants
const unsigned long SLEEP_TIME = 3000; // Sleep time between reads (in milliseconds)

//-----------------------------------------------------------------------------
// forecast constants
const float         ALTITUDE   = 656; // ( 200 m) for Moldova
const float         xDelta = 10.0f, yDelta = 10.0f, zDelta = 10.0f;

// this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm
// get kPa/h be dividing hPa by 10
#define CONVERSION_FACTOR (1.0/10.0)

//-----------------------------------------------------------------------------
// Sensors ID's
#define CHILD_ID_PRESSURE    1 // *
#define CHILD_ID_TEMP1       2 // *
#define CHILD_ID_LIGHT       3 // *
#define CHILD_ID_COMPASS     4
#define CHILD_ID_INFRARED    5 // *
#define CHILD_ID_MICROPHONE  6 // *
#define CHILD_ID_PIR         7 // *
#define CHILD_ID_HUM         8 // *
#define CHILD_ID_TEMP2       9 // *
#define CHILD_ID_LED_R       10// *
#define CHILD_ID_LED_G       11// *
#define CHILD_ID_LED_B       12// *

//-----------------------------------------------------------------------------
// Messages
MyMessage   lightLevelMsg (CHILD_ID_LIGHT,     V_LIGHT_LEVEL);
MyMessage   pressureMsg   (CHILD_ID_PRESSURE,  V_VAR1);
MyMessage   msgHumidity   (CHILD_ID_HUM,       V_HUM);
MyMessage   msgTemp2      (CHILD_ID_TEMP2,     V_TEMP);
MyMessage   msgPirMotion  (CHILD_ID_PIR,       V_TRIPPED);

MyMessage   msgTemp1      (CHILD_ID_TEMP1,     V_TEMP);
MyMessage   msgPressure   (CHILD_ID_PRESSURE,  V_PRESSURE);

MyMessage   msgStatusRed  (CHILD_ID_LED_R,     V_DIMMER);
MyMessage   msgStatusGreen(CHILD_ID_LED_G,     V_DIMMER);
MyMessage   msgStatusBlue (CHILD_ID_LED_B,     V_DIMMER);

MyMessage   msgIrValue    (CHILD_ID_INFRARED,  V_LEVEL);
MyMessage   msgMicValue   (CHILD_ID_MICROPHONE,V_LEVEL);

MyMessage   msgCompass   (CHILD_ID_COMPASS, V_VAR1);

//-----------------------------------------------------------------------------
// hardware configuration
#define HUMIDITY_SENSOR_DIGITAL_PIN 7
#define MOTION_SENSOR_DIGITAL_PIN   2

#define RGB_R_LED_DIGITAL_PIN       3
#define RGB_G_LED_DIGITAL_PIN       5
#define RGB_B_LED_DIGITAL_PIN       6

#define SENSOR_IR_ANALOG_PIN         A6
#define SENSOR_MICROPHONE_ANALOG_PIN A7

//  - Infrared sensor  - A6
//  - Microphone - A7

//-----------------------------------------------------------------------------
// Peripherials
BH1750             lightSensor;
DHT_Unified        dht(HUMIDITY_SENSOR_DIGITAL_PIN, DHTTYPE);
Adafruit_BMP085    bmp = Adafruit_BMP085();      // Digital Pressure Sensor
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

//-----------------------------------------------------------------------------
// globals
uint16_t lightSensorLastValue = 0;

float   lastTemp2;
float   lastHum;
int     dhtSamplingPeriode = 0;
long    lastDhtSamplingTime= 0;
boolean lastMotionValue = false;
int     lastMicrophoneLevel = 0;
int     lastIrLevel = 0;

//-----------------------------------------------------------------------------
// forecast globals
float     lastPressure = -1;
float     lastTemp1 = -1;
//int       lastForecast = -1;
const int LAST_SAMPLES_COUNT = 5;
float     lastPressureSamples[LAST_SAMPLES_COUNT];
int       minuteCount = 0;
bool      firstRound = true;

// average value is used in forecast algorithm.
float     pressureAvg;

// average after 2 hours is used as reference value for the next iteration.
float     pressureAvg2;

float     dP_dt;

int lastRedLedStatus     = 0;
int lastGreenLedStatus   = 0;
int lastBlueLedStatus    = 0;

sensors_event_t lastCompasEvent; 
sensors_event_t event;
sensor_t sensor;

//-----------------------------------------------------------------------------
long getCurrentTimeInMS () {
  return millis();
}

//-----------------------------------------------------------------------------
void setup()
{
    // setup IO
    pinMode(MOTION_SENSOR_DIGITAL_PIN, INPUT);      // sets the motion sensor digital pin as input
    pinMode(RGB_R_LED_DIGITAL_PIN,     OUTPUT);
    pinMode(RGB_G_LED_DIGITAL_PIN,     OUTPUT);
    pinMode(RGB_B_LED_DIGITAL_PIN,     OUTPUT);

    pinMode(SENSOR_IR_ANALOG_PIN,     INPUT);
    pinMode(SENSOR_MICROPHONE_ANALOG_PIN,     INPUT);

    // setup peripherials
    lightSensor.begin();
    dht.begin();
      
    dht.temperature().getSensor(&sensor);
//    Serial.println("------------------------------------");
//    Serial.println("Temperature");
//    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
//    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
//    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
//    Serial.println("------------------------------------");
//    // Print humidity sensor details.
//    dht.humidity().getSensor(&sensor);
//    Serial.println("------------------------------------");
//    Serial.println("Humidity");
//    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
//    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
//    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
//    Serial.println("------------------------------------");      
    dhtSamplingPeriode = sensor.min_delay / 1000; //dht.getMinimumSamplingPeriod();

    if (!bmp.begin())
    {
		  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    }

    //----------------------------------------------------------------------------------
    // Configure compass to read magnetic fields
    compass.begin();
    
    // Get value from eeprom and write to output
    analogWrite(RGB_R_LED_DIGITAL_PIN, 255 * loadState(RGB_R_LED_DIGITAL_PIN) / 100);
    analogWrite(RGB_G_LED_DIGITAL_PIN, 255 * loadState(RGB_G_LED_DIGITAL_PIN) / 100);
    analogWrite(RGB_B_LED_DIGITAL_PIN, 255 * loadState(RGB_B_LED_DIGITAL_PIN) / 100);

    // update GW status
    send( msgStatusRed.set(loadState(RGB_R_LED_DIGITAL_PIN)), false );
    send( msgStatusGreen.set(loadState(RGB_G_LED_DIGITAL_PIN)), false );
    send( msgStatusBlue.set(loadState(RGB_B_LED_DIGITAL_PIN)), false );
}

//-----------------------------------------------------------------------------
#define PRESENTATION_DELAY 50
void presentation () {
    sendSketchInfo("Node 3 (lux, bmp, xyz ...)", "1.0");

    present(CHILD_ID_LIGHT, S_LIGHT_LEVEL, "Light level");
    delay(PRESENTATION_DELAY);
    present(CHILD_ID_HUM,   S_HUM,         "Humidity sensor" );
    delay(PRESENTATION_DELAY);
    present(CHILD_ID_TEMP2, S_TEMP,        "Temperature (From humidity)" );
    delay(PRESENTATION_DELAY);
    present(CHILD_ID_PIR,   S_MOTION,      "PIR motion" );
    delay(PRESENTATION_DELAY);

    present(CHILD_ID_PRESSURE,   S_BARO, "Pressure sensor");
    delay(PRESENTATION_DELAY);
    present(CHILD_ID_TEMP1, S_TEMP, "Temperature from pressure");
    delay(PRESENTATION_DELAY);

    present(CHILD_ID_LED_R, S_DIMMER, "RED led",   false);
    delay(PRESENTATION_DELAY);
    present(CHILD_ID_LED_G, S_DIMMER, "GREEN led", false);
    delay(PRESENTATION_DELAY);
    present(CHILD_ID_LED_B, S_DIMMER, "BLUE led",  false);
    delay(PRESENTATION_DELAY);

    present(CHILD_ID_INFRARED, S_DIMMER, "Flame sensor",   false);
    delay(PRESENTATION_DELAY);
    present(CHILD_ID_MICROPHONE, S_DIMMER, "Microphone sensor",   false);
    delay(PRESENTATION_DELAY);
    present(CHILD_ID_COMPASS, S_CUSTOM, "Magnetic compas",   false); 
    delay(PRESENTATION_DELAY);
}


//-----------------------------------------------------------------------------
float dhtData = 0;
int levelSensor =0;
void loop()
{
  //---------------------------------------------------------------------------
  // Read ligth level from BH sensor
  uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  if (lightSensorLastValue != lux) {
     lightSensorLastValue = lux;

      Serial.print("Sending light level from BH: ");
      Serial.println(lux);
      send(lightLevelMsg.set(lux));
  }

  //---------------------------------------------------------------------------
  // read temperature and humidity from DHT sensor
  if (getCurrentTimeInMS () > (lastDhtSamplingTime + dhtSamplingPeriode)) {
    lastDhtSamplingTime = getCurrentTimeInMS ();
    

    dht.temperature().getEvent(&event);
    dhtData = event.temperature; 
    
    if (!isnan(dhtData) && (lastTemp2 != dhtData)) {
      lastTemp2 = dhtData;
      //temperature = dht.toFahrenheit(temperature);
      //Serial.print("Sending temperature from DHT : ");
      //Serial.println(temperature);
      send(msgTemp2.set(dhtData, 1));
    } 
//    else if (isnan(dhtData)) {
//      Serial.println("failed to read temperature from DHT");
//    }

    dht.humidity().getEvent(&event);
    //float humidity 
    dhtData = event.relative_humidity; 
    if (!isnan(dhtData) && (lastHum != dhtData)) {
      lastHum = dhtData;
//      Serial.print("Sending humidity from DHT : ");
//      Serial.println(humidity);
      send(msgHumidity.set(dhtData, 1));
    } 
//    else if (isnan(dhtData)) {
//      Serial.println("failed to read humidity from DHT");
//    }
  }

  //---------------------------------------------------------------------------
  // Read digital motion value
  boolean tripped = digitalRead(MOTION_SENSOR_DIGITAL_PIN) == HIGH;
  if (lastMotionValue != tripped ) {
    lastMotionValue = tripped;

//    Serial.print("Motion value changed : ");
//    Serial.println(tripped);
    send(msgPirMotion.set(tripped ? "1": "0"));  // Send tripped value to gw
  }

  //---------------------------------------------------------------------------
  // read pressure and temperature for forecast
  //float temperature 
  dhtData = bmp.readTemperature();
  if (dhtData != lastTemp1) {
    lastTemp1 = dhtData;
    send(msgTemp1.set(dhtData, 1));
  }

  dhtData = bmp.readSealevelPressure(ALTITUDE) / 100.0;
  if (dhtData != lastPressure) {
    lastPressure = dhtData;
    send(msgPressure.set(dhtData, 0));
  }

  //---------------------------------------------------------------------------
  //  if led status was sucesfully changed, report the status to GW (?)

  //---------------------------------------------------------------------------
  //  read IR an Microphone levels
  levelSensor = analogRead(SENSOR_MICROPHONE_ANALOG_PIN);
  

  if (lastMicrophoneLevel != levelSensor) {
    lastMicrophoneLevel = levelSensor;
    send(msgMicValue.set(levelSensor));
  }

  levelSensor = analogRead(SENSOR_IR_ANALOG_PIN);
  if (lastIrLevel != levelSensor) {
    lastIrLevel = levelSensor;
    send(msgIrValue.set(levelSensor));
  }

  //---------------------------------------------------------------------------
  // read compas values
  compass.getEvent(&event);
  if (compareCompasValues(event, lastCompasEvent)) {
    send(msgCompass.set((void*)&event, sizeof(event)));
    lastCompasEvent = event;
  }

  //---------------------------------------------------------------------------
  // to be dinamically adjusted if needed, or removed !
  sleep(SLEEP_TIME);
}

//-----------------------------------------------------------------------------
boolean compareCompasValues (sensors_event_t &v1, sensors_event_t &v2) {
  return (
      (abs(v1.magnetic.x - v2.magnetic.x) > xDelta) ||
      (abs(v1.magnetic.y - v2.magnetic.y) > yDelta) ||
      (abs(v1.magnetic.z - v2.magnetic.z) > zDelta)
  );
}

//-----------------------------------------------------------------------------
// Process gateway incomming messages
void incomingMessage(const MyMessage &message)
{
  if (message.type == V_LIGHT) {

  } else if (message.type == V_DIMMER) {
      	uint8_t incomingDimmerStatus = message.getByte();
      	incomingDimmerStatus = constrain((int8_t)incomingDimmerStatus, 0, 100);
        if (message.sensor == CHILD_ID_LED_R) {
          analogWrite(RGB_R_LED_DIGITAL_PIN, 255 * incomingDimmerStatus / 100);
          if (lastRedLedStatus != incomingDimmerStatus) {
            lastRedLedStatus = incomingDimmerStatus;
            send(msgStatusRed.set(incomingDimmerStatus),false);
            saveState(RGB_R_LED_DIGITAL_PIN, incomingDimmerStatus);
          }
        }
        else if (message.sensor == CHILD_ID_LED_G) {
          analogWrite(RGB_G_LED_DIGITAL_PIN, 255 * incomingDimmerStatus / 100);
          if (lastGreenLedStatus != incomingDimmerStatus) {
              lastGreenLedStatus = incomingDimmerStatus;
              send(msgStatusGreen.set(incomingDimmerStatus),false);
              saveState(RGB_G_LED_DIGITAL_PIN, incomingDimmerStatus);
          }
        }
        else if (message.sensor == CHILD_ID_LED_B) {
          analogWrite(RGB_B_LED_DIGITAL_PIN, 255 * incomingDimmerStatus / 100);
          if (lastBlueLedStatus != incomingDimmerStatus) {
              lastBlueLedStatus = incomingDimmerStatus;
              send(msgStatusBlue.set(incomingDimmerStatus),false);
              saveState(RGB_B_LED_DIGITAL_PIN, incomingDimmerStatus);
          }

        }
  }
}
//-----------------------------------------------------------------------------
