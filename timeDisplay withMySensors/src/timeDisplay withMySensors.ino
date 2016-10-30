

//------------------------------------------------------------------------------
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <MySensor.h>
#include <Time.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>

//------------------------------------------------------------------------------
// Constants
#define OLED_RESET 4

//#define ALTITUDE 1655.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message

//------------------------------------------------------------------------------
// sensor pins
#define HUMIDITY_SENSOR_DIGITAL_PIN 2

//------------------------------------------------------------------------------
// sensor objects
DHT humidity;
RTC_DS1307 rtc;
//Adafruit_BMP085 pressure = Adafruit_BMP085();      // Digital Pressure Sensor
Adafruit_SSD1306 display(OLED_RESET);
MySensor 	gw;

//------------------------------------------------------------------------------
// global variables
unsigned long SLEEP_TIME = 30000;

float dhtLastTemp;
float dhtLastHum;

boolean metric = true;

long iterationCounter = 0;

//int encoderValue;
//int encoder0PinA = 3;
//int encoder0PinB = 4;
//int encoder0Pos = 0;
//int encoder0PinALast = LOW;
int n = LOW;

//------------------------------------------------------------------------------
// sensors detection flags
//boolean isPresureSensorDetected = false;
boolean isHumiditySensorDetected = false;

//------------------------------------------------------------------------------
// sensors Id's
#define CHILD_ID_HUM_DHT 	0
#define CHILD_ID_TEMP_DHT 	1
#define CHILD_ID_DISP 		2

//#define CHILD_ID_TEMP_BMP 	2
//#define CHILD_ID_PRES_BMP 	3
//#define CHILD_ID_BUZZER 	4
//#define CHILD_ID_VIBRO		2
//#define CHILD_ID_LED_R		3
//#define CHILD_ID_LED_G		4
//#define CHILD_ID_LED_B		5

//------------------------------------------------------------------------------
// Messages
MyMessage dhtMsgHum		(CHILD_ID_HUM_DHT, 	V_HUM);
MyMessage dhtMsgTemp	(CHILD_ID_TEMP_DHT, V_TEMP);
MyMessage lcdMsgDisp	(CHILD_ID_DISP, V_VAR1);

//MyMessage bmpTempMsg	(CHILD_ID_TEMP_BMP, V_TEMP);
//MyMessage bmpPressureMsg(CHILD_ID_PRES_BMP, V_PRESSURE);
//MyMessage bmpForecastMsg(CHILD_ID_PRES_BMP, V_FORECAST);

//MyMessage buzzerMsg		(CHILD_ID_BUZZER, 	V_LIGHT);
//MyMessage vibroMsg		(CHILD_ID_VIBRO, 	V_LIGHT);
//MyMessage ledRMsg		(CHILD_ID_LED_R, 	V_LIGHT);
//MyMessage ledGMsg		(CHILD_ID_LED_G, 	V_LIGHT);
//MyMessage ledBMsg		(CHILD_ID_LED_B, 	V_LIGHT);

//------------------------------------------------------------------------------
void setupDisplay ()
{
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)

	display.clearDisplay();
	display.setTextSize(3);
	display.setTextColor(BLACK, WHITE);
	display.setCursor(10, 100);
	display.print("REBOOT");

}

//------------------------------------------------------------------------------
void setupSensors ()
{

//	//  setup bmp (pressure)
//	isPresureSensorDetected = true;
//	if (!pressure.begin())
//	{
//		Serial.print("Failed to initialize pressure sensor");
//		isPresureSensorDetected = false;
//	}

	// setup dht (humidity)
	isHumiditySensorDetected = true;
	humidity.setup(HUMIDITY_SENSOR_DIGITAL_PIN);
//	if(!humidity.begin()) {
//		Serial.print("Failed to initialize humidity sensor");
//		isHumiditySensorDetected = false;
//	}

	// setup rtc
	rtc.begin();
	if (!rtc.isrunning())
	{
		//rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	}


	// setup encoder

	// setup vibro (as relay)

	// setup buzzer (as relay ? )

	// setp RGB led (as 3 relays with)




}

void setup()
{
	delay(1000);
	Wire.begin();

	gw.begin(incomingMessage, AUTO, true);
	gw.sendSketchInfo("Multi-Display sensor", "1.0");

	setupDisplay();
	setupSensors();

	gw.present(CHILD_ID_HUM_DHT, S_HUM);
	gw.present(CHILD_ID_TEMP_DHT, S_TEMP);

//	gw.present(CHILD_ID_PRES_BMP, S_BARO);
//	gw.present(CHILD_ID_TEMP_BMP, S_TEMP);

//	gw.present(CHILD_ID_BUZER, S_LIGHT);
//	gw.present(CHILD_ID_VIBRO, S_LIGHT);
//	gw.present(CHILD_ID_LED_R, S_LIGHT);
//	gw.present(CHILD_ID_LED_G, S_LIGHT);
//	gw.present(CHILD_ID_LED_B, S_LIGHT);

	display.display();

}

int lasMin = 0;
int lastHr = 0;
int lastSec = 0;

int lasDay = 0;
int lastYear = 0;
int lastMon = 0;

void drawClock (int xDate, int yDate, int x, int y)
{

	int hours = 0;
	int minutes = 0;
	int seconds = 0;

	int rtcDay = 0;
	int rtcYear = 0;
	int rtcMon = 0;

	if (!rtc.isrunning())
	{
		display.fillRect(x, y, 61, 9, BLACK);
		display.setCursor(x + 1, y + 1);
		display.setTextSize(1);
		display.setTextColor(WHITE);
		display.print("--:--:--");

		display.fillRect(xDate, yDate, 61, 9, BLACK);
		display.setCursor(xDate + 1, yDate + 1);
		display.print("????-??-??");
		return;
	}


	DateTime now = rtc.now();
	hours = now.hour();
	minutes = now.minute();
	seconds = now.second();

	rtcDay = now.day();
	rtcYear = now.year();
	rtcMon = now.month();

	if (lasDay != rtcDay || lastYear != rtcYear || lastMon != rtcMon)
	{
		lasDay = rtcDay;
		lastYear = rtcYear;
		lastMon = rtcMon;

		display.fillRect(xDate, yDate, 61, 9, WHITE);
		display.setCursor(xDate + 1, yDate + 1);
		display.setTextSize(1);
		display.setTextColor(BLACK);

		display.print(rtcYear);
		display.print("-");

		if (rtcMon < 10)
		{
			display.print("0");
		}
		display.print(rtcMon);
		display.print("-");

		if (rtcDay < 10)
		{
			display.print("0");
		}
		display.print(rtcDay);
	}


	// update hour
	if (lastHr != hours || lasMin != minutes || lastSec != seconds)
	{
		display.fillRect(x, y, 61, 9, BLACK);
		display.setCursor(x + 1, y + 1);

		display.setTextSize(1);
		display.setTextColor(WHITE);

		if (hours < 10)
		{
			display.print("0");
		}

		display.print(hours);
		display.print(":");


		if (minutes < 10)
		{
			display.print("0");
		}
		display.print(minutes);

		display.print(":");


		if (seconds < 10)
		{
			display.print("0");
		}
		display.print(seconds);
	}
}

// ---------------------------------------------------------------
// 128X64
void displayTemperature (double temperature)
{
	display.fillRect(0, 16, 55, 9, BLACK);
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(1, 17);
	display.print(temperature, 2);
	display.print("'C");
}

//void displayAbsolutePresure (double presure)
//{
//	display.fillRect(57, 16, 70, 9, BLACK);
//	display.setTextSize(1);
//	display.setTextColor(WHITE);
//	display.setCursor(60, 17);
//	display.print(presure, 2);
//	display.print("mb");
//
//}


//void readBMPSensorData ()
//{
//	/*
//		double T, P, p0, a;
//			status = pressure.startTemperature();
//		if (status != 0)
//		{
//			// Wait for the measurement to complete:
//			delay(status);
//
//			status = pressure.getTemperature(T);
//			if (status != 0)
//			{
//
//				if (T != oldT)
//				{
//					displayTemperature(T);
//					oldT = T;
//				}
//				status = pressure.startPressure(3);
//				if (status != 0)
//				{
//					// Wait for the measurement to complete:
//					delay(status);
//					status = pressure.getPressure(P, T);
//					if (status != 0)
//					{
//						if ( P != oldP)
//						{
//							displayAbsolutePresure(P);
//							oldP = P;
//						}
//					}
//				}
//			}
//		}
//	*/
//}

double oldP = 0;
double oldT = 0;
void loop()
{
	char status;

	drawClock(0, 0, 75, 0);
	display.display();


	iterationCounter ++;

	// force update of the whole display
	if (iterationCounter > 1000)
	{
		iterationCounter = 0;
		lasMin = 0;
		lastHr = 0;
		lastSec = 0;

		lasDay = 0;
		lastYear = 0;
		lastMon = 0;
		oldP = 0;
		oldT = 0;
		display.clearDisplay();

		delay(1000);
		display.display();
	}
}

void incomingMessage(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
//	if (message.type == V_LIGHT)
//	{
//	}

}
