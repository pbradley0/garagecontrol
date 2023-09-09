/* Garage Door Controller
 * Paul Bradley Aug 2023
 * v1.0
 * 
 * Circuit:
 * Door relay connected to pin 26 (G26)
 * Closed reed switch connected to pin 27 (G27)
 * Magnetic encoder connected to pins 21 (SDA, G21), 22 (SCL G22) DIR to G25
 * 
 */

#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SimpleTimer.h>
#include <AS5600.h>

String garageName = "FirstGarage";  // Name of the garage door
const int garageTravel = 20;  // Number of revolutions for full travel of door

// Network and server config
const char* ssid = "yourssid";
const char* passwd = "yourpasswd";
const char* hostname = "First Garage Controller";

// MQTT config
const char* mqttServer = "192.168.1.128";  // MQTT server on artemis
const int mqttPort = 1883;  // MQTT port
const char* clientName = "garage";
const char* userName = "garagecontrol"; // MQTT username
const char* password = "yourmqttpasswd";  // MQTT pasword

//ESP32 pins
const int garageDoorRelayPin = 26;
const int closedDoorPin = 27;
const int clockPin = 22;
const int dataPin = 21;

#define open 0
#define closed 1
#define opening 3
#define closing 4

// Global variables
bool boot = true;
int garagePercent = 0;
int garagePercentOld = 0;
int garageStatus = digitalRead(closedDoorPin);
int garageStatusOld = garageStatus;

void WiFireconnect(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info);  // Reconnects WiFi if connection is lost
void mqttReceive(char* topic, byte* payload, unsigned int length);
void mqttReconnect();  //  Connects or reconnects to the MQTT server
void resetGarageTrigger(); // Resets the button push.  Seperate function so waiting 2 seconds to reset doesn't pause everything else
void reportGarageStatus(); // Reports what the door is doing
void checkIn();  //  Checks in with the MQTT server

//WiFi and MQTT objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);
SimpleTimer timer;

AS5600 as5600; // Magnetic encoder

void setup() 
{
  Serial.begin(115200);

  // Set up mag encoder
  as5600.begin(dataPin, clockPin, AS5600_SW_DIRECTION_PIN);  //  set magnetic encoder pins.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("AS5600 Connection: ");
  Serial.println(b);

  // Set pins
  pinMode(garageDoorRelayPin, OUTPUT);
  pinMode(closedDoorPin, INPUT_PULLUP);

  // Set up WiFi
  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, passwd);
  Serial.print("\nConnecting to WiFi");
  while(WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to Wifi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  WiFi.onEvent(WiFireconnect, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // Set up MQTT server
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttReceive);

  // Set up periodic functions
  timer.setInterval(300000, checkIn);  // Checks in with server every 5 minutes (300000)
  timer.setInterval(250, reportGarageStatus);  // Sees and reports if garage status has changed every 250 ms
}

void loop() 
{
  if (!mqttClient.connected())
  {
    mqttReconnect();
  }
  
  mqttClient.loop();
  timer.run();
}

void WiFireconnect(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info)
{
  int retries = 0; //  Timeout for reconnect
  
  WiFi.begin(ssid, password);
  Serial.print("\nReconnecting to WiFi..");
  while(WiFi.status() != WL_CONNECTED)
  {
    if ( retries < 30 )
    {
      Serial.print(".");
      delay(500);
      retries++;
    }
    else
      esp_restart();  // Reboot if not connected in 15 seconds
  }
  Serial.println("\nConnected to Wifi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect()
{
  String garageStatusTopic = "garage/" + garageName + "/status";  // Create the status string
  String garageTopic = "garage/" + garageName + "/commands";  // Create the topic string
  int retries = 0;
  
  while (!mqttClient.connected())
  {
    if (retries < 15)
    {
        if (mqttClient.connect(clientName, userName, password))
          {
            Serial.println("Connection to MQTT server has been estabilished");
            if(boot)
            {
              mqttClient.publish(garageStatusTopic.c_str(), "Rebooted");
              boot = false;
            }
            if(!boot)
            {
              mqttClient.publish(garageStatusTopic.c_str(), "Reconnected");
            }
            mqttClient.subscribe(garageTopic.c_str());
          }
        else
          {
            retries++;
            Serial.print("Connection to MQTT server failed. Retrying. Attempt: ");\
            Serial.print(retries);
            Serial.println();
            delay(5000);
          }
    }
    if ( retries >= 15 )
    {
      Serial.println("Connection to MQTT server failed. Rebooting.");
      esp_restart(); // Reboot the ESP32
    }
  }
}

void mqttReceive(char* topic, byte* payload, unsigned int length)
{
  // Print the incoming message
  Serial.print("Message received [");
  String newTopic = topic;
  Serial.print(topic);
  Serial.print("]: ");
  payload[length] = '\0';
  String newPayload = String((char*)payload);
  Serial.print(newPayload);
  Serial.println();
  String garageTopic = "garage/" + garageName + "/commands";  // Create the string we should be looking for

  //Do what the message requests
  if ( newTopic == garageTopic )
  {
    if ( newPayload == "toggle")
    {
      // Push the button
      digitalWrite(garageDoorRelayPin, HIGH);
      timer.setTimeout(2000, resetGarageTrigger);  //  Resets the relay in 2 seconds
    }
    
    if ( newPayload == "reboot" )
    {
      esp_restart(); // Reboot the ESP32
    }
  }
}

void resetGarageTrigger()
{
  digitalWrite(garageDoorRelayPin, LOW);
}

void reportGarageStatus()
{
  String garageStatusTopic = "garage/" + garageName + "/status";  // Create the status string
  String garagePercentTopic = "garage/" + garageName + "/percent";  //  Create the percent string
  float garageSpeed = as5600.getAngularSpeed();
  int closedSwitchStatus = digitalRead(closedDoorPin);   
  
  if ( garageSpeed == 0 )
  {
    if ( closedSwitchStatus == false)  // Reed switch shows door is closed
      garageStatus = closed;
    else
      garageStatus = open;
      
    if ( garageStatus != garageStatusOld )  // If the door open or closed has changed so we don't spam the MQTT server
    {
      if ( garageStatus == closed )  // Door is closed
      {
        mqttClient.publish(garageStatusTopic.c_str(), "closed", true);
        Serial.println("Garage is closed");
        as5600.resetCumulativePosition();  // Resets the revolution counter just in case its off
        garagePercent = 0;  // Reset percent open
      }
      else
      {
        mqttClient.publish(garageStatusTopic.c_str(), "open", true);
        Serial.print("Garage is open ");
        Serial.print(garagePercent);
        Serial.println("%");
        garageStatus = open;
      }
      garageStatusOld = garageStatus;
    }
  }
  else // Door is moving.
  {
    garagePercent = ( (float)as5600.getRevolutions() / garageTravel ) * 100;

    if (garageSpeed > 0 )  // Garage is opening
    {
      garageStatus = opening;
      if ( garageStatus != garageStatusOld )  // Only update if something changed so we don't spam the MQTT server
      {
        mqttClient.publish(garageStatusTopic.c_str(), "opening", true);
        Serial.println("Garage is opening ");
        garageStatusOld = garageStatus;
      }          
    }

    if (garageSpeed < 0 )  // Garage is closing
    {
      garageStatus = closing;
      if ( garageStatus != garageStatusOld )  // Only update if something changed so we don't spam the MQTT server
      {
        mqttClient.publish(garageStatusTopic.c_str(), "closing", true);
        Serial.println("Garage is closing ");
        garageStatusOld = garageStatus;
      }
    }    

    if ( (garagePercent % 5 == 0 ) && (garagePercent != garagePercentOld) )  // If the percentage is a multiple of 5 and is new
    {
      Serial.print(garagePercent);
      Serial.print("% ");
      char percent [2];
      itoa(garagePercent, percent, 10);
      mqttClient.publish(garagePercentTopic.c_str(), percent, true);
      garagePercentOld = garagePercent;
    }
  }
}

void checkIn()
{
  String checkInTopic = "checkIn/garage/" + garageName;
  mqttClient.publish(checkInTopic.c_str(), "OK");
}
