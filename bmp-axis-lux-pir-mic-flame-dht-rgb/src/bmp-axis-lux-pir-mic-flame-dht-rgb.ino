
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
#include <SPI.h>
#include <MySensor.h>
#include <BH1750.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <HMC5883L.h>

//-----------------------------------------------------------------------------
// generic constants
const unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)

//-----------------------------------------------------------------------------
// forecast constants
const float         ALTITUDE   = 656; // ( 200 m) for Moldova
const char          *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
const float   xDelta = 10.0f, yDelta = 10.0f, zDelta = 10.0f;

enum FORECAST
{
	STABLE = 0,			// "Stable Weather Pattern"
	SUNNY = 1,			// "Slowly rising Good Weather", "Clear/Sunny "
	CLOUDY = 2,			// "Slowly falling L-Pressure ", "Cloudy/Rain "
	UNSTABLE = 3,		        // "Quickly rising H-Press",     "Not Stable"
	THUNDERSTORM = 4,	        // "Quickly falling L-Press",    "Thunderstorm"
	UNKNOWN = 5		        // "Unknown (More Time needed)
};

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
MyMessage   msgForecast   (CHILD_ID_PRESSURE,  V_FORECAST);

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


#define NODE_ID_XYZ_HUM    100
//  - Infrared sensor  - A6
//  - Microphone - A7

//-----------------------------------------------------------------------------
// Peripherials
MySensor         gw;
BH1750           lightSensor;
DHT              dht;
Adafruit_BMP085  bmp = Adafruit_BMP085();      // Digital Pressure Sensor
HMC5883L         compass;
s


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
float   lastPressure = -1;
float   lastTemp1 = -1;
int     lastForecast = -1;
const int LAST_SAMPLES_COUNT = 5;
float   lastPressureSamples[LAST_SAMPLES_COUNT];
int     minuteCount = 0;
bool   firstRound = true;
// average value is used in forecast algorithm.
float   pressureAvg;
// average after 2 hours is used as reference value for the next iteration.
float   pressureAvg2;

float   dP_dt;


int lastRedLedStatus     = 0;
int lastGreenLedStatus   = 0;
int lastBlueLedStatus    = 0;

Vector lastCompasNorm;

//-----------------------------------------------------------------------------
long getCurrentTimeInMS () {
  return millis();
}

//-----------------------------------------------------------------------------
float getLastPressureSamplesAverage()
{
	float lastPressureSamplesAverage = 0;
	for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
	{
		lastPressureSamplesAverage += lastPressureSamples[i];
	}
	lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

	return lastPressureSamplesAverage;
}


//-----------------------------------------------------------------------------
// Algorithm found here
// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
// Pressure in hPa -->  forecast done by calculating kPa/h
int sample(float pressure)
{
	// Calculate the average of the last n minutes.
	int index = minuteCount % LAST_SAMPLES_COUNT;
	lastPressureSamples[index] = pressure;

	minuteCount++;
	if (minuteCount > 185)
	{
		minuteCount = 6;
	}

	if (minuteCount == 5)
	{
		pressureAvg = getLastPressureSamplesAverage();
	}
	else if (minuteCount == 35)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change * 2; // note this is for t = 0.5hour
		}
		else
		{
			dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
		}
	}
	else if (minuteCount == 65)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) //first time initial 3 hour
		{
			dP_dt = change; //note this is for t = 1 hour
		}
		else
		{
			dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 95)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 1.5; // note this is for t = 1.5 hour
		}
		else
		{
			dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 125)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		pressureAvg2 = lastPressureAvg; // store for later use.
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 2; // note this is for t = 2 hour
		}
		else
		{
			dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 155)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 2.5; // note this is for t = 2.5 hour
		}
		else
		{
			dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
		}
	}
	else if (minuteCount == 185)
	{
		float lastPressureAvg = getLastPressureSamplesAverage();
		float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
		if (firstRound) // first time initial 3 hour
		{
			dP_dt = change / 3; // note this is for t = 3 hour
		}
		else
		{
			dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
		}
		pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
		firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
	}

	int forecast = UNKNOWN;
	if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
	{
		forecast = UNKNOWN;
	}
	else if (dP_dt < (-0.25))
	{
		forecast = THUNDERSTORM;
	}
	else if (dP_dt > 0.25)
	{
		forecast = UNSTABLE;
	}
	else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
	{
		forecast = CLOUDY;
	}
	else if ((dP_dt > 0.05) && (dP_dt < 0.25))
	{
		forecast = SUNNY;
	}
	else if ((dP_dt >(-0.05)) && (dP_dt < 0.05))
	{
		forecast = STABLE;
	}
	else
	{
		forecast = UNKNOWN;
	}

	// uncomment when debugging
	Serial.print(F("Forecast at minute "));
	Serial.print(minuteCount);
	Serial.print(F(" dP/dt = "));
	Serial.print(dP_dt);
	Serial.print(F("kPa/h --> "));
	Serial.println(weather[forecast]);

	return forecast;
}

//-----------------------------------------------------------------------------
void setup()
{

    // setup gateway
    gw.begin(incomingMessage, NODE_ID_XYZ_HUM, true);
    gw.sendSketchInfo("Node 3 (lux, bmp, xyz ...)", "1.0");

    gw.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL, "Light level");
    gw.present(CHILD_ID_HUM,   S_HUM,         "Humidity sensor" );
    gw.present(CHILD_ID_TEMP2, S_TEMP,        "Temperature (From humidity)" );
    gw.present(CHILD_ID_PIR,   S_MOTION,      "PIR motion" );

    gw.present(CHILD_ID_PRESSURE,   S_BARO, "Pressure sensor");
    gw.present(CHILD_ID_TEMP1, S_TEMP, "Temperature from pressure");

    gw.present(CHILD_ID_LED_R, S_DIMMER, "RED led",   false);
    gw.present(CHILD_ID_LED_G, S_DIMMER, "GREEN led", false);
    gw.present(CHILD_ID_LED_B, S_DIMMER, "BLUE led",  false);

    gw.present(CHILD_ID_INFRARED, S_DIMMER, "Flame sensor",   false);
    gw.present(CHILD_ID_MICROPHONE, S_DIMMER, "Microphone sensor",   false);
    gw.present(CHILD_ID_COMPASS, S_CUSTOM, "Magnetic compas",   false);

    // setup IO
    pinMode(MOTION_SENSOR_DIGITAL_PIN, INPUT);      // sets the motion sensor digital pin as input
    pinMode(RGB_R_LED_DIGITAL_PIN,     OUTPUT);
    pinMode(RGB_G_LED_DIGITAL_PIN,     OUTPUT);
    pinMode(RGB_B_LED_DIGITAL_PIN,     OUTPUT);

    pinMode(SENSOR_IR_ANALOG_PIN,     INPUT);
    pinMode(SENSOR_MICROPHONE_ANALOG_PIN,     INPUT);

    // setup peripherials
    lightSensor.begin();
    dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);

    dhtSamplingPeriode = dht.getMinimumSamplingPeriod();

    if (!bmp.begin())
    {
		Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    }

    //----------------------------------------------------------------------------------
    // Configure compass to read magnetic fields
    compass.setRange(HMC5883L_RANGE_1_3GA);    // Set measurement range
    compass.setMeasurementMode(HMC5883L_CONTINOUS);  // Set measurement mode
    compass.setDataRate(HMC5883L_DATARATE_30HZ);  // Set data rate
    compass.setSamples(HMC5883L_SAMPLES_8);  // Set number of samples averaged
    compass.setOffset(0, 0);      // Set calibration offset. See HMC5883L_calibration.ino

    if(!compass.begin()) {
      Serial.println("Failed to initialize compas");
    }
    // Get value from eeprom and write to output
    analogWrite(RGB_R_LED_DIGITAL_PIN, 255 * gw.loadState(RGB_R_LED_DIGITAL_PIN) / 100);
    analogWrite(RGB_G_LED_DIGITAL_PIN, 255 * gw.loadState(RGB_G_LED_DIGITAL_PIN) / 100);
    analogWrite(RGB_B_LED_DIGITAL_PIN, 255 * gw.loadState(RGB_B_LED_DIGITAL_PIN) / 100);

    // update GW status
    gw.send( msgStatusRed.set(gw.loadState(RGB_R_LED_DIGITAL_PIN)), false );
    gw.send( msgStatusGreen.set(gw.loadState(RGB_G_LED_DIGITAL_PIN)), false );
    gw.send( msgStatusBlue.set(gw.loadState(RGB_B_LED_DIGITAL_PIN)), false );
}

//-----------------------------------------------------------------------------
void loop()
{

   gw.process();

  //---------------------------------------------------------------------------
  // Read ligth level from BH sensor
  uint16_t lux = lightSensor.readLightLevel();// Get Lux value
  if (lightSensorLastValue != lux) {
     lightSensorLastValue = lux;

      Serial.print("Sending light level from BH: ");
      Serial.println(lux);

      gw.send(lightLevelMsg.set(lux));
  }

  //---------------------------------------------------------------------------
  // read temperature and humidity from DHT sensor
  if (getCurrentTimeInMS () > (lastDhtSamplingTime + dhtSamplingPeriode)) {
    lastDhtSamplingTime = getCurrentTimeInMS ();

    float temperature = dht.getTemperature();
    if (!isnan(temperature) && (lastTemp2 != temperature)) {
      lastTemp2 = temperature;
      //temperature = dht.toFahrenheit(temperature);
      Serial.print("Sending temperature from DHT : ");
      Serial.println(temperature);
      gw.send(msgTemp2.set(temperature, 1));
    } else if (isnan(temperature)) {
      Serial.println("failed to read temperature from DHT");
    }

    float humidity = dht.getHumidity();
    if (!isnan(humidity) && (lastHum != humidity)) {
      lastHum = humidity;
      Serial.print("Sending humidity from DHT : ");
      Serial.println(humidity);
      gw.send(msgHumidity.set(humidity, 1));
    } else if (isnan(humidity)) {
      Serial.println("failed to read humidity from DHT");
    }
  }

  //---------------------------------------------------------------------------
  // Read digital motion value
  boolean tripped = digitalRead(MOTION_SENSOR_DIGITAL_PIN) == HIGH;
  if (lastMotionValue != tripped ) {
    lastMotionValue = tripped;

    Serial.print("Motion value changed : ");
    Serial.println(tripped);
    gw.send(msgPirMotion.set(tripped ? "1": "0"));  // Send tripped value to gw
  }

  //---------------------------------------------------------------------------
  // read pressure and temperature for forecast
  float temperature = bmp.readTemperature();
  if (temperature != lastTemp1) {
    lastTemp1 = temperature;
    gw.send(msgTemp1.set(temperature, 1));

  }

  float pressure = bmp.readSealevelPressure(ALTITUDE) / 100.0;
  if (pressure != lastPressure) {
    lastPressure = pressure;
    gw.send(msgPressure.set(pressure, 0));

    int forecast = sample(pressure);
    if (lastForecast != forecast) {
      lastForecast = forecast;
      gw.send(msgForecast.set(weather[forecast]));
    }

  }

  //---------------------------------------------------------------------------
  //  if led status was sucesfully changed, report the status to GW (?)


  //---------------------------------------------------------------------------
  //  read IR an Microphone levels
  int microphoneLevel = analogRead(SENSOR_MICROPHONE_ANALOG_PIN);
  int irLevel         = analogRead(SENSOR_IR_ANALOG_PIN);

  if (lastMicrophoneLevel != microphoneLevel) {
    lastMicrophoneLevel = microphoneLevel;
    gw.send(msgMicValue.set(microphoneLevel));
  }

  if (lastIrLevel != irLevel) {
    lastIrLevel = irLevel;
    gw.send(msgIrValue.set(irLevel));
  }

  //---------------------------------------------------------------------------
  // read compas values
//  Vector compasRaw    = compass.readRaw();
  Vector compasNorm   = compass.readNormalize();
  if (compareVectors(compasNorm, lastCompasNorm)) {
      lastCompasNorm = compasNorm;
      gw.send(msgCompass.set((void*)&compasNorm, sizeof(compasNorm)));
  }

  //---------------------------------------------------------------------------
  // to be dinamically adjusted if needed, or removed !
  gw.sleep(SLEEP_TIME);
}

//-----------------------------------------------------------------------------
bool compareVectors (Vector &v1, Vector v2) {
  return (
      (abs(v1.XAxis - v2.XAxis) > xDelta) ||
      (abs(v1.YAxis - v2.YAxis) > yDelta) ||
      (abs(v1.ZAxis - v2.ZAxis) > zDelta)
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
            gw.send(msgStatusRed.set(incomingDimmerStatus),false);
            gw.saveState(RGB_R_LED_DIGITAL_PIN, incomingDimmerStatus);
          }
        }
        else if (message.sensor == CHILD_ID_LED_G) {
          analogWrite(RGB_G_LED_DIGITAL_PIN, 255 * incomingDimmerStatus / 100);
          if (lastGreenLedStatus != incomingDimmerStatus) {
              lastGreenLedStatus = incomingDimmerStatus;
              gw.send(msgStatusGreen.set(incomingDimmerStatus),false);
              gw.saveState(RGB_G_LED_DIGITAL_PIN, incomingDimmerStatus);
          }
        }
        else if (message.sensor == CHILD_ID_LED_B) {
          analogWrite(RGB_B_LED_DIGITAL_PIN, 255 * incomingDimmerStatus / 100);
          if (lastBlueLedStatus != incomingDimmerStatus) {
              lastBlueLedStatus = incomingDimmerStatus;
              gw.send(msgStatusBlue.set(incomingDimmerStatus),false);
              gw.saveState(RGB_B_LED_DIGITAL_PIN, incomingDimmerStatus);
          }

        }
  }
}
//-----------------------------------------------------------------------------
