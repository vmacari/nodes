/**
 	Sensor node N0 with mic, flame, gas and motion sensors. With RGB led and with
 	radio module.
 *
 * Node pinout (sensors):
 *  A0 - microphone
 *  A1 - flame diode
 *  A2 - gas sensor
 *  A3 - motion sensor
 *  D3 (b), D4 (G), D5 (R) - RGB led
 *  RF-242 module
 *
 */

#define LED_R 		3
#define LED_G 		4
#define LED_B 		5

#define MIC_PIN 	A0
#define FLAME_PIN 	A1
#define GAS_PIN 	A2
#define MOTION_PIN 	A3

void setup()
{
	pinMode(LED_R, OUTPUT);
	pinMode(LED_G, OUTPUT);
	pinMode(LED_B, OUTPUT);

	pinMode(MIC_PIN, INPUT);
	pinMode(FLAME_PIN, INPUT);
	pinMode(GAS_PIN, INPUT);
	pinMode(MOTION_PIN, INPUT);

	Serial.begin(9600);
}

void loop()
{

	analogWrite(LED_R, 0);
	analogWrite(LED_G, 0);
	analogWrite(LED_B, 0);
	for (int i = 0 ; i < 255; i ++) {
		analogWrite(LED_R, i);
		delay(10);
	}

	//delay(500);
	analogWrite(LED_R, 0);

	for (int i = 0 ; i < 255; i ++) {
		analogWrite(LED_G, i);
		delay(10);
	}
	//delay(500);
	analogWrite(LED_G, 0);

		for (int i = 0 ; i < 255; i ++) {
		analogWrite(LED_B, i);
		delay(10);
	}
	//delay(500);

	int mic = analogRead(MIC_PIN);
	int flame = analogRead(FLAME_PIN);

	int gas = analogRead(GAS_PIN);
	int motion = analogRead(MOTION_PIN);

	Serial.print(" mic: ");Serial.println(mic);
	Serial.print(" flame: ");Serial.println(flame);
	Serial.print(" gas: ");Serial.println(gas);
	Serial.print(" motion: ");Serial.println(motion);
	Serial.println("--------------------------------");

}
