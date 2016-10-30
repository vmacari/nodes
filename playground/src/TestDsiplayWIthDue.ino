

//#include <Adafruit_ssd1306syp.h>
//#include <Time.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include <Wire.h>
#include <SPI.h>
#include <stdarg.h>


#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);



void setup()
{
	delay(1000);
	Wire.begin();
	delay(1000);

	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)

	display.clearDisplay();
	display.setTextSize(3);
	display.setTextColor(BLACK, WHITE);
	display.setCursor(10, 100);
	display.print("REBOOT");

	display.display();
}


void loop()
{



}
