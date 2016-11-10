/*
* Copyright (C) 2013 Henrik Ekblad <henrik.ekblad@gmail.com>
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* DESCRIPTION
* The ArduinoGateway prints data received from sensors on the serial link.
* The gateway accepts input on seral which will be sent out on radio network.
*
* The GW code is designed for Arduino Nano 328p / 16MHz
*
* Wire connections (OPTIONAL):
* - Inclusion button should be connected between digital pin 3 and GND
* - RX/TX/ERR leds need to be connected between +5V (anode) and digital ping 6/5/4 with resistor 270-330R in a series
*
* LEDs (OPTIONAL):
* - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
* - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
* - ERR (red) - fast blink on error during transmission error or recieve crc error
*/

//#define MY_DEBUG 
#define MY_RADIO_NRF24
#define MY_GATEWAY_SERIAL
//#define MY_RF24_CHANNEL  11

// Define a lower baud rate for Arduino's running on 8 MHz (Arduino Pro Mini 3.3V & SenseBender)
#if F_CPU == 8000000L
#define MY_BAUD_RATE 38400
#endif


#include <SPI.h>
#include <MyConfig.h>
#include <MySensors.h>
//#include <MyGateway.h>
#include <stdarg.h>


//#define INCLUSION_MODE_TIME 1 // Number of minutes inclusion mode is enabled
//#define INCLUSION_MODE_PIN 3 // Digital pin used for inclusion mode button

#define LED1 2
#define LED2 3
#define LED3 4

int counter = 0;
#define SHORT_DELAY  25
#define LONG_DELAY 100

//MyGateway gw(DEFAULT_CE_PIN, DEFAULT_CS_PIN, INCLUSION_MODE_TIME, INCLUSION_MODE_PIN,  6, 5, 4);

char inputString[MY_GATEWAY_MAX_RECEIVE_LENGTH] = "";    // A string to hold incoming commands from serial/ethernet interface
int inputPos = 0;
boolean commandComplete = false;  // whether the string is complete

void setup()
{
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  Serial.begin(115200);
  //gw.begin();
}

void loop()
{

//  //gw.processRadioMessage();
//  if (commandComplete) {
//    // A command wass issued from serial interface
//    // We will now try to send it to the actuator
//    parseAndSend(inputString);
//    commandComplete = false;
//    inputPos = 0;
//  }
//  else {
//    if (counter == 0) {
//        digitalWrite(LED1, HIGH);
//        delay (SHORT_DELAY);
//    }
//    else if (counter >= 1 && counter < 20) {
//        digitalWrite(LED1, LOW);
//        digitalWrite(LED2, LOW);
//        digitalWrite(LED3, LOW);
//        delay (LONG_DELAY);
//    }
//    else if (counter >= 20) {
//      counter = -1;
//    }
//    counter ++;
//  }


    if (counter == 0) {
        digitalWrite(LED1, HIGH);
        delay (SHORT_DELAY);
    }
    else if (counter >= 1 && counter < 20) {
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        delay (LONG_DELAY);
    }
    else if (counter >= 20) {
      counter = -1;
    }
    counter ++;

}

void receive(MyMessage &receivedMessage) {
  Serial.println("Message received from serial!");  
}

//void receiveRf(MyMessage &receivedMessage) {
////  Serial.println("Message received form RF");  
//}

/*
SerialEvent occurs whenever a new data comes in the
hardware serial RX.  This routine is run between each
time loop() runs, so using delay inside loop can delay
response.  Multiple bytes of data may be available.
*/
//void serialEvent() {
//
//while (Serial.available()) {
//  // get the new byte:
//  char inChar = (char)Serial.read();
//  // if the incoming character is a newline, set a flag
//  // so the main loop can do something about it:
//  if (inputPos < MY_GATEWAY_MAX_RECEIVE_LENGTH-1 && !commandComplete) {
//    if (inChar == '\n') {
//      inputString[inputPos] = 0;
//      commandComplete = true;
//    } else {
//      // add it to the inputString:
//      inputString[inputPos] = inChar;
//      inputPos++;
//    }
//  } else {
//     // Incoming message too long. Throw away
//      inputPos = 0;
//  }
//}
//
//}
