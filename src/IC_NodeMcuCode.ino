/***************************************************
  Irrigation Controller
  Richard Huish 2017
  ESP8266 based with local home-assistant.io GUI,
    relay output for dual irrigation control via MQTT
    MQTT command message with 'on' payload command one of the outputs on.
    MQTT command message with 'off' payload command one of the outputs off.
  ----------
  Key Libraries:
  ESP8266WiFi.h    https://github.com/esp8266/Arduino
  ----------
  GUI: Locally hosted home assistant
  MQTT: Locally hosted broker https://mosquitto.org/
  ----------
  The circuit:
  NodeMCU Amica (ESP8266)
  Inputs:
    N/A
  Outputs:
    Relay one output - pin 14 (NodeMCU Pin xx)
    Relay two output - pin 12 (NodeMCU Pin xx)
    LED_NODEMCU - pin 16 (NodeMCU Pin D0)
    LED_ESP - GPIO pin 2 (NodeMCU Pin D4) (Shared with 433Mhz TX)
    ----------
  Notes:
    NodeMCU lED lights to show MQTT conenction.
    ESP lED lights to show WIFI conenction.

****************************************************/

// Note: Libaries are inluced in "Project Dependencies" file platformio.ini
#include <ESP8266WiFi.h>  // ESP8266 core for Arduino https://github.com/esp8266/Arduino
#include <PubSubClient.h>  // Arduino Client for MQTT https://github.com/knolleary/pubsubclient
#include <private.h> // Passwords etc not for github


// WiFi parameters
const char* wifi_ssid = secret_wifi_ssid; // Wifi access point SSID
const char* wifi_password = secret_wifi_password; // Wifi access point password

// MQTT Settings
const char* mqtt_server = secret_mqtt_server; // E.G. 192.168.1.xx
const char* clientName = secret_clientName; // Client to report to MQTT
const char* mqtt_username = secret_mqtt_username; // MQTT Username
const char* mqtt_password = secret_mqtt_password; // MQTT Password
boolean willRetain = true; // MQTT Last Will and Testament
const char* willMessage = "offline"; // MQTT Last Will and Testament Message

// Subscribe
// The MQTT topic commands are received on to change the switch state.
const char* commandTopic1 = secret_commandTopic1; // E.G. Home/Irrigation/Command1
const char* commandTopic2 = secret_commandTopic2; // E.G. Home/Irrigation/Command2

// Publish
// The MQTT topic to publish state updates.
const char* publishStateTopic1 = secret_stateTopic1; // E.G. Home/Irrigation/OutputState1
const char* publishStateTopic2 = secret_stateTopic2; // E.G. Home/Irrigation/OutputState2

const char* publishLastWillTopic = secret_publishLastWillTopic; // E.G. Home/LightingGateway/status"
const char* publishClientName = secret_publishClientName; // E.G. Home/LightingGateway/clientName"
const char* publishIpAddress = secret_publishIpAddress; // E.G. Home/LightingGateway/IpAddress"
const char* publishSignalStrength = secret_publishSignalStrength; // E.G. Home/LightingGateway/SignalStrength"
const char* publishHostName = secret_publishHostName; // E.G. Home/LightingGateway/HostName"
const char* publishSSID = secret_publishSSID; // E.G. Home/LightingGateway/SSID"

// Define state machine states
typedef enum {
  s_idle1 = 0,          // state idle
  s_Output1Start = 1,  // state start
  s_Output1On = 2,     // state on
  s_Output1Stop = 3,   // state stop
} e_state1;
int stateMachine1 = 0;

typedef enum {
  s_idle2 = 0,          // state idle
  s_Output2Start = 1,  // state start
  s_Output2On = 2,     // state on
  s_Output2Stop = 3,   // state stop
} e_state2;
int stateMachine2 = 0;

typedef enum {
  outputOne = 0,
  outputTwo = 1,
} irrigationOutputs;

// MQTT instance
WiFiClient espClient;
PubSubClient mqttClient(espClient);
char message_buff[100];
long lastReconnectAttempt = 0; // Reconnecting MQTT - non-blocking https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino

// MQTT publish frequency
unsigned long previousMillis = 0;
const long publishInterval = 60000; // Publish requency in milliseconds 60000 = 1 min

// LED output parameters
const int DIGITAL_PIN_LED_ESP = 2; // Define LED on ESP8266 sub-modual
const int DIGITAL_PIN_LED_NODEMCU = 16; // Define LED on NodeMCU board - Lights on pin LOW
const int DIGITAL_PIN_RELAY_ONE = 14; // Define relay output one
const int DIGITAL_PIN_RELAY_TWO = 12; // Definerelay output two

// Maximum number of minutes to keep outputs on (Set in code, but modified by web commands, local setpoint in case of internet connection break)
float outputOneDuration = 5;
float outputTwoDuration = 5;

// Output powered status
bool outputOnePoweredStatus = false;
bool outputTwoPoweredStatus = false;


// Setp the connection to WIFI and the MQTT Broker. Normally called only once from setup
void setup_wifi() {
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);

  // Connect to the WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.print("");
  Serial.println("WiFi connected");

  Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("Hostname: %s\n", WiFi.hostname().c_str());

  digitalWrite(DIGITAL_PIN_LED_NODEMCU, LOW); // Lights on LOW. Light the NodeMCU LED to show wifi connection.
}



// MQTT payload to transmit via out gateway
void mqttcallback(char* topic, byte* payload, unsigned int length) {
  //If you want to publish a message from within the message callback function, it is necessary to make a copy of the topic and payload values as the client uses the same internal buffer for inbound and outbound messages:
  //http://www.hivemq.com/blog/mqtt-client-library-encyclopedia-arduino-pubsubclient/
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();


  // create character buffer with ending null terminator (string)
  int i = 0;
  for (i = 0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  // Check the value of the message
  String msgString = String(message_buff);
  Serial.println(msgString);

  // Check the message topic
  String srtTopic = topic;

  if (srtTopic.equals(commandTopic1)) {
    if (msgString == "1"){
      stateMachine1 = s_Output1Start; // Set output one to be on
    } else if (msgString == "0"){
      stateMachine1 = s_Output1Stop; // Set output one to be off
    }
  }

  else if (srtTopic.equals(commandTopic2)) {
    if (msgString == "1"){
      stateMachine2 = s_Output2Start; // Set output one to be on
    } else if (msgString == "0"){
      stateMachine2 = s_Output2Stop; // Set output one to be off
    }
  }
}

/*
  Non-Blocking mqtt reconnect.
  Called from checkMqttConnection.
  Based on example from 5ace47b Sep 7, 2015 https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
*/
boolean mqttReconnect() {
  // Call on the background functions to allow them to do their thing
  yield();
  // Attempt to connect
  if (mqttClient.connect(clientName, mqtt_username, mqtt_password, publishLastWillTopic, 0, willRetain, willMessage)) {

    Serial.print("Attempting MQTT connection...");

    // Once connected, update status to online, retained = true - last will Message will drop in if we go offline ...
    mqttClient.publish(publishLastWillTopic, "online", true);

    // Publish device name, retained = true
    mqttClient.publish(publishClientName, clientName, true);

    // Publish device IP Address, retained = true
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
    mqttClient.publish(publishIpAddress, buf, true);

    // Publish the Wi-Fi signal quality (RSSI), retained = true
    String tempVar = String(WiFi.RSSI());
    mqttClient.publish(publishSignalStrength, tempVar.c_str(), true);

    // Publish the device DHCP hostname, retained = true
    mqttClient.publish(publishHostName, WiFi.hostname().c_str(), true);

    // Publish the WiFi SSID, retained = true
    mqttClient.publish(publishSSID, WiFi.SSID().c_str(), true);

    // Resubscribe to feeds
    mqttClient.subscribe(secret_commandTopic1);
    mqttClient.subscribe(secret_commandTopic2);


    Serial.println("connected");

  } else {
    Serial.print("Failed MQTT connection, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try in 1.5 seconds");
  }
  return mqttClient.connected();
}

/*
  Checks if connection to the MQTT server is ok. Client connected
  using a non-blocking reconnect function. If the client loses
  its connection, it attempts to reconnect every 5 seconds
  without blocking the main loop.
  Called from main loop.
*/
void checkMqttConnection() {
  if (!mqttClient.connected()) {
    // We are not connected. Turn off the wifi LED
    digitalWrite(DIGITAL_PIN_LED_ESP, HIGH); // Lights on LOW
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (mqttReconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // We are connected.
    digitalWrite(DIGITAL_PIN_LED_ESP, LOW); // Lights on LOW
    //call on the background functions to allow them to do their thing.
    yield();
    // Client connected: MQTT client loop processing
    mqttClient.loop();
  }
}

// MQTT Publish with normal or immediate option.
void mtqqPublish(bool ignorePublishInterval) {

  // Only run when publishInterval in milliseonds expires or ignorePublishInterval == true
  unsigned long currentMillis = millis();
  // CODE TO MOVE TO functions
  if (currentMillis - previousMillis >= publishInterval || ignorePublishInterval == true) {
    // save the last time this ran
    previousMillis = currentMillis;

    if (mqttClient.connected()) {
      // Publish the Wi-Fi signal quality (RSSI), retained = true
      String tempVar = String(WiFi.RSSI());
      mqttClient.publish(publishSignalStrength, tempVar.c_str(), true);

      // Publish with retained messages
      String strOutputOne = String(outputOnePoweredStatus);
      if (!mqttClient.publish(publishStateTopic1, strOutputOne.c_str(), true))
        Serial.print(F("Failed to output state one to [")), Serial.print(strOutputOne), Serial.print("] ");
      else
        Serial.print(F("Output one state published to [")), Serial.print(strOutputOne), Serial.println("] ");

      String strOutputTwo = String(outputTwoPoweredStatus);
      if (!mqttClient.publish(publishStateTopic2, strOutputTwo.c_str(), true))
        Serial.print(F("Failed to output state two to [")), Serial.print(strOutputTwo), Serial.print("] ");
      else
        Serial.print(F("Output two state published to [")), Serial.print(strOutputTwo), Serial.println("] ");
    }
  }
}

void controlOutputOne(bool state){
  if (state == true) {
    // Command the output on.
    Serial.println("controlOutputOne state true");
    digitalWrite(DIGITAL_PIN_RELAY_ONE, LOW);
    outputOnePoweredStatus = true;
  } else {
    // Command the output off.
    Serial.println("controlOutputOne state false");
    digitalWrite(DIGITAL_PIN_RELAY_ONE, HIGH);
    outputOnePoweredStatus = false;
  }
}

void controlOutputTwo(bool state){
  if (state == true) {
    // Command the output on.
    Serial.println("controlOutputTwo state true");
    digitalWrite(DIGITAL_PIN_RELAY_TWO, LOW);
    outputTwoPoweredStatus = true;
  } else {
    // Command the output off.
    Serial.println("controlOutputTwo state false");
    digitalWrite(DIGITAL_PIN_RELAY_TWO, HIGH);
    outputTwoPoweredStatus = false;
  }
}


// State machines for controller
void checkState1() {
  switch (stateMachine1) {

    case s_idle1:
      // State is currently: idle
      break;


    case s_Output1Start:
      // State is currently: starting
      Serial.println("State is currently: starting output one");
      // Command the output on.
      controlOutputOne(true);
      mtqqPublish(true); // Immediate publish cycle
      stateMachine1 = s_Output1On;
      break;


    case s_Output1On:
      // State is currently: On
      // Check if we need to stop, by checking for watchdog.

      // if (!checkHeatRequired(dht.readTemperature(), targetHeaterTemperature, targetHeaterTemperatureHyst))
      // {
      //   // Heat no longer required, stop.
      //   stateMachine = s_HeaterStop;
      // }
      break;



    case s_Output1Stop:
      // State is currently: stopping
      Serial.println("State is currently: stopping output one");
      // Command the output off.
      controlOutputOne(false);
      mtqqPublish(true); // Immediate publish cycle
      // Set state mahcine to idle on the next loop
      stateMachine1 = s_idle1;
      break;


  }
}

void checkState2() {
  switch (stateMachine2) {

    case s_idle2:
      // State is currently: idle
      break;


    case s_Output2Start:
      // State is currently: starting
      Serial.println("State is currently: starting output two");
      // Command the output on.
      controlOutputTwo(true);
      mtqqPublish(true); // Immediate publish cycle
      stateMachine2 = s_Output2On;
      break;


    case s_Output2On:
      // State is currently: On
      // Check if we need to stop, by checking for watchdog.

      // if (!checkHeatRequired(dht.readTemperature(), targetHeaterTemperature, targetHeaterTemperatureHyst))
      // {
      //   // Heat no longer required, stop.
      //   stateMachine = s_HeaterStop;
      // }
      break;



    case s_Output2Stop:
      // State is currently: stopping
      Serial.println("State is currently: stopping output two");
      // Command the output off.
      controlOutputTwo(false);
      mtqqPublish(true); // Immediate publish cycle
      // Set state mahcine to idle on the next loop
      stateMachine1 = s_idle2;
      break;
  }
}

void setup() {
  // Initialize pins
  pinMode(DIGITAL_PIN_LED_NODEMCU, OUTPUT);
  pinMode(DIGITAL_PIN_LED_ESP, OUTPUT);
  pinMode(DIGITAL_PIN_RELAY_ONE, OUTPUT);
  pinMode(DIGITAL_PIN_RELAY_TWO, OUTPUT);

  // Initialize pin start values
  digitalWrite(DIGITAL_PIN_LED_NODEMCU, LOW); // Lights on HIGH
  digitalWrite(DIGITAL_PIN_LED_ESP, HIGH); // Lights on LOW
  digitalWrite(DIGITAL_PIN_RELAY_ONE, HIGH);
  digitalWrite(DIGITAL_PIN_RELAY_TWO, HIGH);

  // set serial speed
  Serial.begin(115200);
  Serial.println("Setup Starting");


  // Call on the background functions to allow them to do their thing
  yield();
  // Setup wifi
  setup_wifi();
  // Call on the background functions to allow them to do their thing
  yield();
  // Set MQTT settings
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttcallback);
  // Call on the background functions to allow them to do their thing
  yield();
  Serial.println("Setup Complete");
}




/// Main working loop
void loop() {
  yield(); //call on the background functions to allow them to do their thing.
  // First check if we are connected to the MQTT broker
  checkMqttConnection();
  //call on the background functions to allow them to do their thing.
  yield();
  // Check the status and do actions
  checkState1();
  checkState2();

  mtqqPublish(false); // Normal publish cycle
}
