//DCC EX Modified
// Goal is Loco reciving <T 14 10 1> commands and fuction commands and then processing them.
// MQTT serial messaged are currently NOT being parsed and processed.
//EXAMPLE: Loco 14 given speed forward command, command read from rocrail/command by Node-Red, data is formatted in node red and outputted to topic "test"
//Contents of test is subscribed to, and output is sent to serial.write.
//Commands sent to ESP 32 WROOM from Arduino serial AND from DCC EX web throttle are proccessed correctly. Loco fully controlable.


////////////////////////////////////////////////////////////////////////////////////
//  DCC-EX CommandStation-EX   Please see https://DCC-EX.com
//
// This file is the main sketch for the Command Station.
//
// CONFIGURATION:
// Configuration is normally performed by editing a file called config.h.
// This file is NOT shipped with the code so that if you pull a later version
// of the code, your configuration will not be overwritten.
//
// If you used the automatic installer program, config.h will have been created automatically.
//
// To obtain a starting copy of config.h please copy the file config.example.h which is
// shipped with the code and may be updated as new features are added.
//
// If config.h is not found, config.example.h will be used with all defaults.
////////////////////////////////////////////////////////////////////////////////////

#if __has_include ( "config.h")
  #include "config.h"
  #ifndef MOTOR_SHIELD_TYPE
  #error Your config.h must include a MOTOR_SHIELD_TYPE definition. If you see this warning in spite not having a config.h, you have a buggy preprocessor and must copy config.example.h to config.h
  #endif
#else
  #warning config.h not found. Using defaults from config.example.h
  #include "config.example.h"
#endif

 #include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = "*****"; // Enter your Wi-Fi name
const char *password = "*******";  // Enter Wi-Fi password// MQTT Broker
const char *mqtt_broker = "192.168.***.***";
const char *topic = "test";
const char *mqtt_username = "****";
const char *mqtt_password = "****";
const int mqtt_port = 1883;

//Set tanks and PINS

int tank; //Front tank status
int rtank; //Rear reserve and filling access tank
const int motor = 21; //21 on device
int waterlow = 16; // 16 on device
int waterhigh = 17; // D17 on device
int rwaterlow = 18; //D18 on device
int rwaterhigh = 19; //D19 on device
int wl=0;
int wh=0;
int rwl=0;
int rwh=0;
bool cycle=false;





WiFiClient espClient;
PubSubClient client(espClient);











/*
 *  © 2021 Neil McKechnie
 *  © 2020-2021 Chris Harlow, Harald Barth, David Cutting,
 *  Fred Decker, Gregor Baues, Anthony W - Dayton
 *  © 2023 Nathan Kellenicki
 *  All rights reserved.
 *
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "DCCEX.h"
#include "Display_Implementation.h"

#ifdef CPU_TYPE_ERROR
#error CANNOT COMPILE - DCC++ EX ONLY WORKS WITH THE ARCHITECTURES LISTED IN defines.h
#endif

#ifdef WIFI_WARNING
#warning You have defined that you want WiFi but your hardware has not enough memory to do that, so WiFi DISABLED
#endif
#ifdef ETHERNET_WARNING
#warning You have defined that you want Ethernet but your hardware has not enough memory to do that, so Ethernet DISABLED
#endif
#ifdef EXRAIL_WARNING
#warning You have myAutomation.h but your hardware has not enough memory to do that, so EX-RAIL DISABLED
#endif

void setup()
{
  // The main sketch has responsibilities during setup()

  // Responsibility 1: Start the usb connection for diagnostics
  // This is normally Serial but uses SerialUSB on a SAMD processor
  SerialManager::init();

  DIAG(F("License GPLv3 fsf.org (c) dcc-ex.com"));

// Initialise HAL layer before reading EEprom or setting up MotorDrivers 
  IODevice::begin();

  // As the setup of a motor shield may require a read of the current sense input from the ADC,
  // let's make sure to initialise the ADCee class!
  ADCee::begin();
  // Set up MotorDrivers early to initialize all pins
  TrackManager::Setup(MOTOR_SHIELD_TYPE);

  DISPLAY_START (
    // This block is still executed for DIAGS if display not in use
    LCD(0,F("DCC-EX v%S"),F(VERSION));
    LCD(1,F("Lic GPLv3"));
  );

  // Responsibility 2: Start all the communications before the DCC engine
  // Start the WiFi interface on a MEGA, Uno cannot currently handle WiFi
  // Start Ethernet if it exists
#ifndef ARDUINO_ARCH_ESP32
#if WIFI_ON
  WifiInterface::setup(WIFI_SERIAL_LINK_SPEED, F(WIFI_SSID), F(WIFI_PASSWORD), F(WIFI_HOSTNAME), IP_PORT, WIFI_CHANNEL, WIFI_FORCE_AP);
#endif // WIFI_ON
#else
  // ESP32 needs wifi on always
  WifiESP::setup(WIFI_SSID, WIFI_PASSWORD, WIFI_HOSTNAME, IP_PORT, WIFI_CHANNEL, WIFI_FORCE_AP);
#endif // ARDUINO_ARCH_ESP32

#if ETHERNET_ON
  EthernetInterface::setup();
#endif // ETHERNET_ON
  
  // Responsibility 3: Start the DCC engine.
  DCC::begin();

  // Start RMFT aka EX-RAIL (ignored if no automnation)
  RMFT::begin();


  // Invoke any DCC++EX commands in the form "SETUP("xxxx");"" found in optional file mySetup.h.
  //  This can be used to create turnouts, outputs, sensors etc. through the normal text commands.
  #if __has_include ( "mySetup.h")
    #define SETUP(cmd) DCCEXParser::parse(F(cmd))
    #include "mySetup.h"
    #undef SETUP
  #endif

  #if defined(LCN_SERIAL)
  LCN_SERIAL.begin(115200);
  LCN::init(LCN_SERIAL);
  #endif
  LCD(3, F("Ready"));
  CommandDistributor::broadcastPower();

  
//pinMode (waterlow, FUNCTION_0);

//pinMode (waterhigh, FUNCTION_0);

//pinMode (rwaterlow, FUNCTION_0);

//pinMode (rwaterhigh, FUNCTION_0);

//pinMode (motor, FUNCTION_3);

pinMode(motor, OUTPUT); //3.7v Micro coreless motor, running pump

pinMode(waterlow, INPUT_PULLUP);

pinMode(waterhigh, INPUT_PULLUP);

pinMode(rwaterlow, INPUT_PULLUP);

pinMode(rwaterhigh, INPUT_PULLUP);



    // Set software serial baud to 115200;
    Serial.begin(115200);
    // Connecting to a WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");
    //connecting to a mqtt broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public EMQX MQTT broker connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    // Publish and subscribe
    client.publish(topic, "WD-1");
    client.subscribe(topic);
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
// Forward the message to the Serial input buffer as if it was typed
    for (unsigned int i = 0; i < length; i++) {
        Serial.write(payload[i]);
    }

}

void loop()
{
  // The main sketch has responsibilities during loop()

  // Responsibility 1: Handle DCC background processes
  //                   (loco reminders and power checks)
  DCC::loop();

  // Responsibility 2: handle any incoming commands on USB connection
  SerialManager::loop();

  // Responsibility 3: Optionally handle any incoming WiFi traffic
#ifndef ARDUINO_ARCH_ESP32
#if WIFI_ON
  WifiInterface::loop();
#endif //WIFI_ON
#else  //ARDUINO_ARCH_ESP32
#ifndef WIFI_TASK_ON_CORE0
  WifiESP::loop();
#endif
#endif //ARDUINO_ARCH_ESP32
#if ETHERNET_ON
  EthernetInterface::loop();
#endif

  RMFT::loop();  // ignored if no automation

  #if defined(LCN_SERIAL)
  LCN::loop();
  #endif

  // Display refresh
  DisplayInterface::loop();

  // Handle/update IO devices.
  IODevice::loop();

  Sensor::checkAll(); // Update and print changes

  // Report any decrease in memory (will automatically trigger on first call)
  static int ramLowWatermark = __INT_MAX__; // replaced on first loop

  int freeNow = DCCTimer::getMinimumFreeMemory();
  if (freeNow < ramLowWatermark) {
    ramLowWatermark = freeNow;
    LCD(3,F("Free RAM=%5db"), ramLowWatermark);
  }
      client.loop();

 
//Tank sensor reads
  // Everything behind "//" below, is omitted in order to clean serial output for testing
    
wl = digitalRead(waterlow); 
wh = digitalRead(waterhigh);
rwl = digitalRead(rwaterlow);
rwh = digitalRead(rwaterhigh);

if (wh == LOW) {tank = LOW;}

if (wl == LOW) {tank = HIGH;}

if (rwh == LOW) {rtank = LOW;}

if (rwl == LOW) {rtank = HIGH;}

if (tank == HIGH && rtank == LOW) {digitalWrite (motor, HIGH);} //Front tank is empty, rear tank is full turn on pump

if (tank == LOW) {digitalWrite (motor, LOW);} //Front tank is now full pump motor turn off

//if (rtank == HIGH) {digitalWrite (motor, LOW);} //Front tank is empty AND rear tank is empty, stop motor to avoid damage to motor

//Send loco for refueling
  
//if (rwl == HIGH && cycle == false) {client.publish("rocrail/service/client","<lc id=\"WD1\" cmd=\"gotoblock\" blockid=\"bk4\"/>");}

//if (rwl == HIGH && cycle == false) {client.publish("rocrail/service/client","<lc id=\"WD1\" cmd=\"go\"/>");}

//delay (2000);

//if (rwl == HIGH && cycle == false) {cycle = true;}

//if (tank == LOW && rtank == LOW) {cycle = false;}

//Serial.print("Publish message: ");
//Serial.println(msg);

//Serial.print("tank:");
//Serial.println(tank);
//Serial.print("wl:");
//Serial.println(wl);
//Serial.print("wh:");
//Serial.println(wh);

//Serial.print("rtank:");
//Serial.println(rtank);
//Serial.print("rwl:");
//Serial.println(rwl);
//Serial.print("rwh:");
//Serial.println(rwh);
//Serial.print("cycle: ");
//Serial.println(cycle);
}

