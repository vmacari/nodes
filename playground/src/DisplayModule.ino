
//#include <Wire.h>
//#include <Adafruit_GFX.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

//#include "Adafruit_ssd1306syp.h"

//#include <Time.h>
//#include <Adafruit_SSD1306.h>

#include <Wire.h>
#include "RTClib.h"
#include <SPI.h>
#include <MySensor.h>
#include <MyGateway.h>
#include <stdarg.h>


RTC_DS1307 rtc;


#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message


//#define SDA_PIN A4
//#define SCL_PIN A5
//Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);
#define OLED_RESET 7
Adafruit_SSD1306 display(OLED_RESET);

//boolean bmpInitialized = false;
long iterationCounter = 0;

//int encoderValue;
//int encoder0PinA = 3;
//int encoder0PinB = 4;
//int encoder0Pos = 0;
//int encoder0PinALast = LOW;
//int n = LOW;

int RTC_DS_PIN	   = A3;
int touchSensorPin = 8;
int vibroPin 	   = 6;
int vibroValue = 0;
#define CHILD_ID_DISP 		0

#define RGB_R_PIN	4
#define RGB_G_PIN	5
#define RGB_B_PIN	6

MySensor 	gw;
MyMessage lcdMsgDisp	(CHILD_ID_DISP, V_VAR1);


void setup()
{

	pinMode (RGB_B_PIN, OUTPUT);
	digitalWrite(RGB_B_PIN, 0);
	delay(1000);
	digitalWrite(RGB_B_PIN, 1);
	Wire.begin();

	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
	//display.initialize();
	digitalWrite(RGB_B_PIN, 0);
	display.clearDisplay();
	display.setTextSize(3);
	display.setTextColor(BLACK, WHITE);
	display.setCursor(10, 100);
	display.print("REBOOT");

	gw.begin(incomingMessage, AUTO, true);
	gw.sendSketchInfo("Display sensor", "1.0");

	gw.present(CHILD_ID_DISP, S_CUSTOM);

	Serial.begin(9600);
	Serial.println("Starting rtc ... ");

	rtc.begin();
	if (!rtc.isrunning())
	{
		rtc.adjust(DateTime((const char*)F(__DATE__),(const char*) F(__TIME__)));
	}

	Serial.println("Display data ..  ");
	display.display();

	Serial.println("Configure pins ... ");
	pinMode (touchSensorPin, INPUT);
	pinMode (vibroPin, OUTPUT);



	analogWrite(vibroPin, 0);

digitalWrite(RGB_B_PIN, 1);
	Serial.println("Done intialization");
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

//// ---------------------------------------------------------------
//// 128X64
//void displayTemperature (double temperature)
//{
//	display.fillRect(0, 16, 55, 9, BLACK);
//	display.setTextSize(1);
//	display.setTextColor(WHITE);
//	display.setCursor(1, 17);
//	display.print(temperature, 2);
//	display.print("'C");
//}
//
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

//double oldP = 0;
//double oldT = 0;
void loop()
{
//	char status;
//	double T, P, p0, a;
//
//	drawClock(0, 0, 75, 0);
//
////	status = pressure.startTemperature();
////	if (status != 0)
////	{
////		// Wait for the measurement to complete:
////		delay(status);
////
////		status = pressure.getTemperature(T);
////		if (status != 0)
////		{
////
////			if (T != oldT)
////			{
////				displayTemperature(T);
////				oldT = T;
////			}
////			status = pressure.startPressure(3);
////			if (status != 0)
////			{
////				// Wait for the measurement to complete:
////				delay(status);
////				status = pressure.getPressure(P, T);
////				if (status != 0)
////				{
////					if ( P != oldP)
////					{
////						displayAbsolutePresure(P);
////						oldP = P;
////					}
////				}
////			}
////		}
////	}
//
//	//    display.update();
////	display.display();
////
////	iterationCounter ++;
////
////	// force update of the whole display
////	if (iterationCounter > 1000)
////	{
////		iterationCounter = 0;
////		lasMin = 0;
////		lastHr = 0;
////		lastSec = 0;
////
////		lasDay = 0;
////		lastYear = 0;
////		lastMon = 0;
//////		oldP = 0;
//////		oldT = 0;
////		// display.clear();
////		display.clearDisplay();
////
////		delay(1000);
////		//  display.update();
////		display.display();
////	}
//
//
//	if (digitalRead(touchSensorPin)) {
//		vibroValue ++;
//		if (vibroValue > 250) vibroValue = 0;
//
//  		analogWrite(vibroPin, vibroValue);
//
//	} else {
//		vibroValue = 0;
//		analogWrite(vibroPin, 0);
//	}
//
//

}

void incomingMessage(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
//	if (message.type == V_LIGHT)
//	{
//	}

}
