/*
 *	1. display - SDA/SKL
 *  2. vibro
 *  3. rtc
 *  4. rgb
 *  5. touch
 *
 */

#include <SPI.h>
#include <MySensor.h>
#include "RTClib.h"
#include <Wire.h>
#include <stdarg.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 0
Adafruit_SSD1306 display(OLED_RESET);


RTC_DS1307 rtc;

#define TIME_MSG_LEN  11   // time sync to PC is HEADER followed by Unix time_t as ten ASCII digits
#define TIME_HEADER  'T'   // Header tag for serial time sync message
#define TIME_REQUEST  7    // ASCII bell character requests a time sync message

#define SDA_PIN A4
#define SCL_PIN A5

long iterationCounter = 0;


int     RTC_DS_PIN	= A3;
int     touchSensorPin  = 8;
int     vibroPin 	= 6;
int     vibroValue      = 0;
#define CHILD_ID_DISP 	0

#define RGB_B_PIN	3
#define RGB_R_PIN	5
#define RGB_G_PIN	4


MySensor      gw;
boolean       timeReceived = false;
unsigned long lastUpdate=0;
unsigned long lastRequest=0;

MyMessage     lcdMsgDisp  (CHILD_ID_DISP, V_VAR1);
int           writeIndex;
char          lastMessages [10][10]; // 10 messages

void setup()
{
	Serial.begin(115200);

	pinMode (RGB_R_PIN, OUTPUT);
	pinMode (RGB_G_PIN, OUTPUT);
	pinMode (RGB_B_PIN, OUTPUT);

	digitalWrite(RGB_R_PIN, 0);
	digitalWrite(RGB_G_PIN, 0);
	digitalWrite(RGB_B_PIN, 0);

	Serial.println("Starting RF ... ");

	gw.begin(incomingMessage, AUTO, true);
	gw.sendSketchInfo("Display sensor", "1.0");
	gw.present(CHILD_ID_DISP, S_CUSTOM);

	delay(1000);

	Serial.println("Starting display ... ");


//	display.initialize();
        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
        display.clearDisplay();   // clears the screen and buffer


	display.setTextSize(2);
	display.setTextColor(WHITE, BLACK);
	display.setCursor(25, 30);
	display.print("REBOOT");
//        display.update();
        display.display(); // show splashscreen

	Serial.println("Starting rtc ... ");

	rtc.begin();
	if (!rtc.isrunning())
	{
		rtc.adjust(DateTime((const char*)F(__DATE__),(const char*) F(__TIME__)));
	}


	Serial.println("Configure pins ... ");
	pinMode (touchSensorPin, INPUT);
	pinMode (vibroPin, OUTPUT);

	analogWrite(vibroPin, 0);

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

                Serial.println("RTC not running");
		display.fillRect(x, y, 61, 9, BLACK);
		display.setCursor(x + 1, y + 1);
		display.setTextSize(1);
		display.setTextColor(WHITE);
		display.print("--:--:--");

		display.fillRect(xDate, yDate, 61, 9, BLACK);
		display.setCursor(xDate + 1, yDate + 1);
		display.print("????-??-??");
		//display.update();
                display.display(); // show splashscreen
		return;
	}


	boolean updateDisplay = false;
	DateTime now = rtc.now();
	hours = now.hour();
	minutes = now.minute();
	seconds = now.second();

	rtcDay = now.day();
	rtcYear = now.year();
	rtcMon = now.month();

	if (lasDay != rtcDay || lastYear != rtcYear || lastMon != rtcMon)
	{
                 Serial.println("Update date");
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
		updateDisplay = true;
	}


	// update hour
	if (lastHr != hours || lasMin != minutes || lastSec != seconds)
	{

                   Serial.println("Update time");

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
		updateDisplay = true;
	}

	if (updateDisplay) {
//		display.update();
display.display(); // show splashscreen
	}
 }


int touchValue = 0;
int lastTouchValue = 1;
boolean dismissCurrentMessage = false;
boolean newMessageArrived = false;
void loop()
{
	char status;
	double T, P, p0, a;



	drawClock(5, 0, 75, 0);


        touchValue = digitalRead(touchSensorPin);

	if (touchValue) {
		vibroValue ++;

		if (vibroValue > 250) vibroValue =250;

  		analogWrite(vibroPin, vibroValue);
                analogWrite(RGB_R_PIN, vibroValue);





	} else {
		vibroValue = 0;
		analogWrite(vibroPin, 0);
                analogWrite(RGB_R_PIN, 0);
	}



  if (touchValue != lastTouchValue ) {

    	display.setTextSize(1);
	display.setTextColor(WHITE, BLACK);
	display.setCursor(0, 0);
        if (touchValue) {
    	  display.print("*");
          dismissCurrentMessage = true;
        } else {
          display.print(" ");
        }
        //display.update();
        display.display(); // show splashscreen
        lastTouchValue = touchValue;
  }


  //------------------------------------------------------------------------
  if (dismissCurrentMessage) {

    // remove current messate and display next from buffer if any
    dismissCurrentMessage = false;
  }

  //------------------------------------------------------------------------
  if (newMessageArrived) {
    // alert (vibro and flash) on new message

    newMessageArrived = false;
  }

  // kepp alerting (flashing led) if there are any unread messages


}

void incomingMessage(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
	if (message.type == V_LIGHT)
	{
            // [0] = 1;
            //lastMessages[writeIndex]
            display.setTextSize(1);
            display.setTextColor(BLACK,WHITE);
	    display.setCursor(0, 25);
             display.print(message.data);
            display.display();
            newMessageArrived = true;
	}
}
