# Intro 
I have an old AC that does not have temperature control. I also have chinchillas that cannot tolerate high temperatures. Introducing the Temperature Controlled Relay!  
It will turn on/off a relay at a certain temperature. Has automatic, manual on, manual off, and set point potentiometer.  

Temperature controller air conditioning sensor for my old AC.
Saves power by turning the AC completely off.
Saves me having to wake up and turn on the AC on super hot days.
AC is set to normally closed relay contact just in case the relay fails; the AC will be on.
Uses a one-wire DS18B20 temp sensor.
Single button for  Automatic / Manual ON / Manual OFF mode selection.
LED for mode indication. Green = Auto, Blinking Red/Green = Manual ON, Red = Manual OFF.
10K potentiometer for set point control. Range between TEMP_MIN and TEMP_MAX

# Wiring

Temp Sensor - To make connection easier I am powering sensor via output pins.
POS_SENSOR_PIN -> Senor Power
GND_SENSOR_PIN -> Sensor GND
TEMP_SENSOR_PIN -> Sensor Data Pin

5v Relay Board
5v -> Relay Board Power
GND -> Relay Board GND
RELAY_PIN -> Relay Trigger

Potentiometer
5v -> POT Pin 1
POT_PIN -> Pin 2 (wiper)
GND -> POT Pin 3

Button
GND -> Button -> SELECT_BUTTON_PIN

Indicator LEDS
GND -> Resistor -> RED LED -> LED_PIN -> GRN LED -> Resistor -> 5v
When LED_PIN is LOW, power flows from 5v to GND via GRN LED, When LED_PIN is HIGH, power flows from LED_PIN to GND via RED LED.

