## Window Evacuation Button-Activated Release System (Window E-BARS)

**Arduino Nano ESP32 | MLE04641 | ABX00083

This project implements a fire-alert window guard safety device using temperature sensing, servo-driven window release, RGB indicators, buzzer alarms, event logging, and WiFi time synchronization.

## Materials Needed

Core Hardware
	•	Arduino Nano ESP32 (ABX00083)
	•	DHT22 Temperature and Humidity Sensor
	•	SG90 or compatible Servo Motor
	•	Push Button (momentary)
	•	Passive or Active Buzzer
	•	Common-Cathode RGB LED
	•	7-Segment Display (single digit)

Optional / External Components
	•	3-32v Boost Converter
	•	Breadboard or PCB
	•	Jumper/Solid wires
	•	5V regulated power supply
	•	USB Type-C cable


## Pin Configuration

The following pins are defined inside the firmware.

**Sensor and Actuators
	•	DHT22 → D11
	•	Servo → A3
	•	Button → D2
	•	Buzzer → D13

**RGB LED
	•	Red → A0
	•	Green → A1
	•	Blue → A2

**Seven Segment Display
	•	Segment A → D9
	•	Segment B → D10
	•	Segment C → D4
	•	Segment D → D5
	•	Segment E → D6
	•	Segment F → D8
	•	Segment G → D7
	•	Decimal Point → D3


## Libraries to Install

Install the following libraries in the Arduino IDE Library Manager before compiling.
	•	WiFi.h (included with ESP32 core)
	•	time.h (included with ESP32 core)
	•	DHT sensor library
	•	ESP32Servo
	•	EEPROM

Install process:
	1.	Open Arduino IDE
	2.	Go to Sketch → Include Library → Manage Libraries
	3.	Search and install:

	•	DHT sensor library by Adafruit
	•	ESP32Servo

The ESP 32 Board package already includes the other libraries

## Board Setup

Before uploading:
	1.	Install ESP32 Board Support Package in Arduino IDE.
	2.	Select board:
        Arduino Nano ESP32
	3.	Set the correct COM port.


## Code Changes Before Upload

1. WiFi Credentials

Modify the WiFi list inside the code.

Using (⌘ + F) / (ctrl + F), Locate:

WifiConfig wifiList[WIFI_COUNT] = {
  { "wifissid1", "password1" },
  { "wifissid2", "password2" },
  { "wifissid3", "password3" },
  { "wifissid4", "password4" },
  { "wifissid5", "password5" }
};

Replace the values with your real network credentials.
The device will automatically try each network until it connects.

2. Servo Positions

Adjust servo positions depending on your mechanical design.

Using (⌘ + F) / (ctrl + F), Locate:

static int cfg_SERVO_POS_CLOSED   = 52;
static int cfg_SERVO_POS_RELEASED = 152;

Ensure:
	•	CLOSED = window guard locked
	•	RELEASED = window guard deployed

3. Fire Detection Thresholds

Modify temperature triggers if needed.

Using (⌘ + F) / (ctrl + F), Locate:

static float cfg_FIRESTART = 35.0f;
static float cfg_FIRECLR   = 35.0f;

Meaning:
	•	FIRESTART → temperature that triggers the fire state
	•	FIRECLR → temperature that clears fire state


4. Simulation Limits

Maximum simulation set temperature:

static float cfg_SIMMAX = 1674.1f;

## Procedure of Usage
	1.	Connect the Arduino Nano ESP32 to your computer.
	2.	Open the .ino file in Arduino IDE.
	3.	Select:
          Board: Arduino Nano ESP32
	4.	Click Upload.
  5. On the top right, turn on Serial Monitor by clicking the magnifying glass icon
  6. Set the Serial Monitor baud rate to: 115200


## Basic Usage

** Button Controls

Single click
-  Toggle Manual Override

Double click
-  Toggle Buzzer Mute

Triple click
-  Print Stats and Log

Quad click
-  Clear EEPROM Stats and Logs

Long press
-  Cancel override and mute buzzer

** Serial Commands

Open the Serial Monitor and type commands.

Examples:
HELP
STATS
LOG
SIM 60
RGB TEST PULSE
SERVO TEST
DISPLAY COUNTUP

** EEPROM Logging

The system stores:
	•	Event history
	•	Maximum temperature reached
	•	Total fire time
	•	Button interaction statistics

EEPROM capacity:  4096 bytes
