// Temperature_Controlled_Relay.ino
// Temperature controller air conditioning sensor for my old AC.
// Saves power by turning the AC completely off.
// Saves me having to wake up and turn on the AC on super hot days.
// AC is set to normally closed relay contact just in case the relay fails; the AC will be on.
// Uses a one-wire DS18B20 temp sensor.
// Single button for  Automatic / Manual ON / Manual OFF mode selection.
// LED for mode indication. Green = Auto, Blinking Red/Green = Manual ON, Red = Manual OFF.
// 10K potentiometer for set point control. Range between TEMP_MIN and TEMP_MAX

// Wiring

// Temp Sensor - To make connection easier I am powering sensor via output pins.
// POS_SENSOR_PIN -> Senor Power
// GND_SENSOR_PIN -> Sensor GND
// TEMP_SENSOR_PIN -> Sensor Data Pin

// 5v Relay Board
// 5v -> Relay Board Power
// GND -> Relay Board GND
// RELAY_PIN -> Relay Trigger

// Potentiometer
// 5v -> POT Pin 1
// POT_PIN -> Pin 2 (wiper)
// GND -> POT Pin 3

// Button
// GND -> Button -> SELECT_BUTTON_PIN

// Indicator LEDS
// GND -> Resistor -> RED LED -> LED_PIN -> GRN LED -> Resistor -> 5v
// When LED_PIN is LOW, power flows from 5v to GND via GRN LED, When LED_PIN is HIGH, power flows from LED_PIN to GND via RED LED.


#include <OneWire.h>
#include <DallasTemperature.h>


#define POS_SENSOR_PIN 6
#define GND_SENSOR_PIN 7
#define TEMP_SENSOR_PIN 8

#define RELAY_PIN 4 // HIGH = RELAY ON aka AC OFF, LOW = RELAY OFF aka AC ON.

#define POT_PIN A1

#define SELECT_BUTTON_PIN 5

#define LED_PIN 2 // HIGH = Red, LOW = Green.
#define  POS_LED_PIN 3

// Temp Range Settings
#define TEMP_MIN 65
#define TEMP_MAX 80
#define DROOP 1 // Amount temperature has to drop to turn off AC
#define MIN_CYCLE_US 300000UL // Prevent AC from short-cycling, always run for at least; (5 minutes).


static uint8_t AC_mode; // 0 = Auto, 1 = Manual ON, 2 = Manual OFF.
float setTemp = 75;

OneWire tempSensor(TEMP_SENSOR_PIN);
DallasTemperature sensors(&tempSensor);
DeviceAddress tempDeviceAddress;


void setup() {
	// Serial.begin(115200);

	AC_mode = 0; // Default to Auto.

	// Default pin states.
	digitalWrite(POS_LED_PIN, HIGH);
	digitalWrite(LED_PIN, HIGH);
	pinMode(POS_LED_PIN, OUTPUT);
	pinMode(LED_PIN, OUTPUT);

	digitalWrite(RELAY_PIN, LOW);
	pinMode(RELAY_PIN, OUTPUT);

	pinMode(SELECT_BUTTON_PIN, INPUT_PULLUP);

	digitalWrite(POS_SENSOR_PIN, HIGH);
	digitalWrite(GND_SENSOR_PIN, LOW);
	pinMode(POS_SENSOR_PIN, OUTPUT);
	pinMode(GND_SENSOR_PIN, OUTPUT);

  //Initialize Sensor
  sensors.begin();
	sensors.getAddress(tempDeviceAddress, 0); // Retrieve sensor address.
	sensors.setResolution(tempDeviceAddress, 12); // Set temp resolution to 12 bits.
}


void loop() {
	sampleButton();
  samplePot();
	updateStatus(tempDeviceAddress);

}

float sampleTemp(DeviceAddress deviceAddress) {
	// Requests temp update from sensor.
	// Returns temp in F.
	// Returns 255 on error.

	sensors.requestTemperatures();

	float tempF = sensors.getTempF(deviceAddress);
	if(tempF == DEVICE_DISCONNECTED_F)
		return 255;
	else
		return tempF;
}

void sampleButton() { // Check button state and cycle through modes.
	if (!digitalRead(SELECT_BUTTON_PIN)) { // Active when Low.
		++AC_mode%=3; // Increment AC_mode, then modulo by 3 to cycle between 0, 1, and 2.
		delay(200); // Debounce and prevents double presses.
	}

}

void updateStatus(DeviceAddress deviceAddress) {
	switch (AC_mode) {

		case 0: // Auto Mode, Solid Green Light.
			digitalWrite(LED_PIN, LOW);
			autoMode(deviceAddress);
		break;

		case 1: // Manual ON Mode, Blinking Red/Green Light.
			digitalWrite(LED_PIN, (millis() & 0b10000000000)? HIGH : LOW); // A trick to blink the led at a consistent rate. every 1024ms.
			while(digitalRead(RELAY_PIN))
				digitalWrite(RELAY_PIN, LOW);
		break;

		case 2: // Manual OFF Mode, Solid Red Light.
			digitalWrite(LED_PIN, HIGH);
			while(!digitalRead(RELAY_PIN))
				digitalWrite(RELAY_PIN, HIGH);
		break;
	}
}

void autoMode(DeviceAddress deviceAddress) {
	static unsigned long startTime;

	if (sampleTemp(deviceAddress)>=setTemp) // If temp is higher than setTemp.
		while(digitalRead(RELAY_PIN)) {
			startTime = millis();  // Save startTime.
			digitalWrite(RELAY_PIN, LOW); // Turn on AC.
		}
	else if (sampleTemp(deviceAddress)<=setTemp-DROOP && millis() - startTime >= MIN_CYCLE_US) // Else if lower AND min cycle time has ellapsed.
		while(!digitalRead(RELAY_PIN))
			digitalWrite(RELAY_PIN, HIGH); // Turn off AC.
}

void samplePot() {
  // read potentiometer value and
  // map pot range (0 - 1023) to temp range.
  // swap 0 and 1023 if you want the pot to be reversed.
  setTemp = map(analogRead(POT_PIN), 0, 1023, TEMP_MAX*10, TEMP_MIN*10); // Multiply by 10 for accuracy.
  setTemp/=10;
  
//  Serial.print(setTemp);
//  Serial.print(' ');
//  Serial.println(sampleTemp(tempDeviceAddress));
}
