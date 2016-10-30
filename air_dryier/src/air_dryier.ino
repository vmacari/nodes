// AIR Dry unit with:
// a refrigerator and ventilator (Relay 1 & Relay 2)
// a water acumulation tank (a switch)
// termistor
// RGB led
// Hum + Temperature sensor
// On/Off regulator

// Hardware mapping
// 1. (A0) On/Off Switch. Pulled up to 3.3 V. Active is Low state. Some regulator ...
// 2. (A1) Thermistor, Pulled up to 3.4V
// 3. (D3) R, (D5) G, (D6) B  - RGB Led. Direct coonected
// 4. (D2) Compressor RElay
// 5. (D4) Ventilator Relay
// 6. (A2) Tank full switch

// 7. (A3) Humidity sensor (DHT)

//-----------------------------------------------------------------------------
#include "TimerOne.h"
#include <SPI.h>
#include <MySensor.h>
#include <Wire.h>
#include "DHT.h"

//-----------------------------------------------------------------------------
#define INPUT_ON_OFF_SWITCH       A0
#define INPUT_THERMISTOR          A1
#define OUTPUT_LED_R              3
#define OUTPUT_LED_G              5
#define OUTPUT_LED_B              6

#define OUTPUT_COMPRESSOR_RELAY   2
#define OUTPUT_VENTILATION_RELAY  4

#define INPUT_TANK_FULL_SWITCH    A2
#define INPUT_DHT_DIGITAL_PIN     A3

//-----------------------------------------------------------------------------
// Fading means the app is stoped in some way. Blikning means the app is doing something

#define STATUS_NONE         0   // none
#define STATUS_IDLE         1   // green led fading
#define STATUS_TANK_FULL    2   // red led fading
#define STATUS_WORKING      3   // blue led blinking
#define STATUS_STOPED       4   // green led fading
#define STATUS_DEFROST      5   // blues led fading

//-----------------------------------------------------------------------------
// Node ID
#define NODE_ID_AIR_DRYIER   102


//-----------------------------------------------------------------------------
// Application working modes:
// 1. The app starts the compressor for few minutes, until the freezing element becomes
//    under 0 degrees.
// 2. After frozing elelemt is minus -1, start ventilator
// 3. Keep compressor and ventilator running for few minutes, untill wather covers the frizint element
// 4. Stop compressor
// 5. After a very short period, stop ventilator
// 6. Wait few minutes until the wather will go to thank

// In the loop verify following measurementes:
//   humidity, temperature, tank level indicator, ON/Off switch
//   If humidity is outside of allowed delta, start working for a limited periode of time, Then ventilation and refrigerator needs to have a rest
//   If on switch is off, do not work
//
// With radio module:
// - receive humidity level
// - receive temperature level (parameter used to start/stop ventilator)
// - receive time to work and defrost
//
// - send temperature and humidity
// - send tank status
// - send vetilator and refrigerator statuses
// - send on/off switch position
// - send operation status

//-----------------------------------------------------------------------------
int applicationStatus       = STATUS_DEFROST;

//-----------------------------------------------------------------------------
// Modules
MySensor         gw;
DHT dht;

//-----------------------------------------------------------------------------
// Messages
#define CHILD_ID_ON_OFF      1   //   report on/off status
#define CHILD_ID_TANK_FULL   2   //   report tank full status
#define CHILD_ID_THERMISTOR  3   //   report thermistor value
#define CHILD_ID_COMPRESSOR  4   //   report compressor status
#define CHILD_ID_VENTIATOR   5   //   report ventilator status
#define CHILD_ID_HUMIDITY    6   //   report actual humidity, read humidity level to keep
#define CHILD_ID_TEMP        7   //   report actual temperature, read operational temperature
#define CHILD_ID_STATUS      8   //   report node status to GW
#define CHILD_ID_TIME        9   //   time to set to node
#define CHILD_ID_DELAY       10  //   working time delay (how long compressor is running)


MyMessage   msgOnOffSwitch      (CHILD_ID_ON_OFF,    V_TRIPPED);
MyMessage   msgTankFullSwitch   (CHILD_ID_TANK_FULL,    V_TRIPPED);

MyMessage   msgHumidity         (CHILD_ID_CHILD_ID_HUMIDITYHUM,       V_HUM);
MyMessage   msgTemperature      (CHILD_ID_TEMP,     V_TEMP);


MyMessage   msgNodeStatus  (CHILD_ID_STATUS,     V_LEVEL);
MyMessage   msgStatusGreen(CHILD_ID_LED_G,     V_DIMMER);


//-----------------------------------------------------------------------------
bool isTankFull () {
  return (analogRead(INPUT_TANK_FULL_SWITCH) > 10);
}

//-----------------------------------------------------------------------------
bool isSystemTurnedOn () {
 return (analogRead(INPUT_ON_OFF_SWITCH) > 10);
}

//-----------------------------------------------------------------------------
int lastDhtTemperature;
int lastDhtHumidity;
int currentDhtTemperature;
int currentDhtHumidity;

bool lastTankFullSwitchStatus;
bool currentTankFullSwitchStatus;
bool lastOnOffSwitchStatus;
bool currentOnOffSwitchStatus;

int lastThermistorTemperature;
int currentThermistorTemperature;

//-----------------------------------------------------------------------------
// A calback with dividers to handle multiple timer resolution
void callbackTimer()
{

  static long timerDivider = 0;

  // 10 ms timer
  if (!(timerDivider % 10)) {
  }

  // 100 ms timer
  if (!(timerDivider % 100)) {
  }

  // 0,5 sec timer
  if (!(timerDivider % 500)) {

  }

  // 1 sec timer
  if (!(timerDivider % 1000)) {

    currentDhtHumidity    = dht.getHumidity();
    currentDhtTemperature = dht.getTemperature();
    Serial.print("DHT Read status : ");
    Serial.print(dht.getStatusString());
    Serial.println();
  }

  updateLedsStatus(timerDivider);
  timerDivider ++;
}

//-----------------------------------------------------------------------------
int getFadeValue (long timerDivider) {

  static int ledFadeValue = 0;
  static int ledBlinkValue = -1;

     if (!(timerDivider % 5)) {
      if (ledBlinkValue) {
        ledFadeValue ++;
        if (ledFadeValue >= 250) {
          ledBlinkValue = false;
        }
      } else {
        ledFadeValue --;
        if (ledFadeValue <= -100) {
          ledBlinkValue = true;
        }
      }
     }

      return ledFadeValue > 0 ? ledFadeValue : 0;
}

//-----------------------------------------------------------------------------
void updateLedsStatus (long timerDivider) {

  static int ledFadeValue = 0;
  static int ledBlinkValue = -1;
  const bool ledIdleStatusPattern [] = {1, 0, 0, 1, 0, 0, 0, 0, 0};

  if (applicationStatus == STATUS_IDLE) {
      digitalWrite(OUTPUT_LED_R, 0);
      analogWrite(OUTPUT_LED_G, getFadeValue(timerDivider));
      digitalWrite(OUTPUT_LED_B, 0);

  } else if (applicationStatus == STATUS_TANK_FULL) {
      analogWrite(OUTPUT_LED_R, getFadeValue(timerDivider));
      digitalWrite(OUTPUT_LED_G, 0);
      digitalWrite(OUTPUT_LED_B, 0);

  }
  else if (applicationStatus == STATUS_WORKING) {
    if (!(timerDivider % 100)) {
      ledBlinkValue += 1;
      if (ledBlinkValue > sizeof(ledIdleStatusPattern) / sizeof(bool)) {
        ledBlinkValue = 0;
      }
      digitalWrite(OUTPUT_LED_R, 0);
      digitalWrite(OUTPUT_LED_G, 0);
      digitalWrite(OUTPUT_LED_B, ledIdleStatusPattern[ledBlinkValue]);
      ledFadeValue = 0;
    }
  }
  else if (applicationStatus == STATUS_STOPED) {
      digitalWrite(OUTPUT_LED_R,  0);
      analogWrite(OUTPUT_LED_G, getFadeValue(timerDivider));
      digitalWrite(OUTPUT_LED_B, 0);
  } else if (applicationStatus == STATUS_DEFROST) {
      digitalWrite(OUTPUT_LED_R,  0);
      digitalWrite(OUTPUT_LED_G, 0);
      analogWrite(OUTPUT_LED_B, getFadeValue(timerDivider));

  } else {
    digitalWrite(OUTPUT_LED_R, 0);
    digitalWrite(OUTPUT_LED_G, 0);
    digitalWrite(OUTPUT_LED_B, 0);
  }
}

//-----------------------------------------------------------------------------
void setup()
{
  // configure hardware
  pinMode(INPUT_ON_OFF_SWITCH, INPUT);
  pinMode(INPUT_THERMISTOR, INPUT);
  pinMode(INPUT_TANK_FULL_SWITCH, INPUT);

  pinMode(OUTPUT_LED_R, OUTPUT);
  pinMode(OUTPUT_LED_G, OUTPUT);
  pinMode(OUTPUT_LED_B, OUTPUT);

  pinMode(OUTPUT_COMPRESSOR_RELAY, OUTPUT);
  pinMode(OUTPUT_VENTILATION_RELAY, OUTPUT);

  dht.setup(INPUT_DHT_DIGITAL_PIN); // data pin 2

  Timer1.initialize(1000);                 // 1 ms
  Timer1.attachInterrupt(callbackTimer);  // attaches callback() as a timer overflow interrupt

  Serial.begin(9600);
}


int ledNumber = 1;
void loop()
{

  //--------------------------------------------------------------------------
  // handle temperature change
  if (lastDhtTemperature != currentDhtTemperature) {
    int delta = abs(lastDhtTemperature - currentDhtTemperature);
    lastDhtTemperature = currentDhtTemperature;

    gw.send(msgTemp2.set(temperature, 1));
  }


  //--------------------------------------------------------------------------
  // handle humidity change
  if (lastDhtHumidity != currentDhtHumidity) {

    int delta = abs(lastDhtHumidity - currentDhtHumidity);

    lastDhtHumidity = currentDhtHumidity;
  }

int ;
int currentDhtTemperature;
int ;


}
