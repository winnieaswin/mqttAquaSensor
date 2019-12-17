#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <NTPClient.h>
#include <ESPmDNS.h>
#include <RemoteDebug.h>
#include <PubSubClient.h>
#include <string.h>
#include <stdio.h>


//Deepsleep configuration#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  450
RTC_DATA_ATTR int bootCount = 0;

const char* mqtt_server = "192.168.1.160";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

char t_char [8];
char h_char [8];
char dallas_t_char [8];

char WIFI_SSID[]="ASWIN_2.4G_";
char WIFI_PASSWORD[]="Pi@Riya*1";
char CLIENT_ID[]= "b001";
int status = WL_IDLE_STATUS;
RemoteDebug Debug;
const char* host = "b001";
int day,hours,minutes,seconds,year,month,date,minuteSave;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP,"0.pool.ntp.org", 25200, 0);
#define DHTPIN 23     // Digital pin connected to the DHT sensor
#define DHTTYPE    DHT22     // DHT 11
DHT dht(DHTPIN, DHTTYPE);
// ledPin refers to ESP32 GPIO 23

const int ledPin = 2;
float ph,rawadc = 0.00;
float h,t,dallas_t;
String formattedTime;
String IP_adr;
int tick=0,msgCount=0,msgReceived = 0,publishMsg=0, sensorfail =0;

int publicfail = 5;
int publicfailCount = 5;
int samplect = 200;
int i = 0;


#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();
//config ph meter

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 15
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }

}

void initWifi()
{
  while (status != WL_CONNECTED)
  {
      //Debug.print("Attempting to connect to SSID: ");
      //Debug.println(WIFI_SSID);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      // wait 5 seconds for connection:
      delay(5000);
      Serial.println("Connected to wifi");
  }

}

void initRemoteDebug()
{
	String hostNameWifi = CLIENT_ID;
	hostNameWifi.concat(".local");
	if (MDNS.begin(CLIENT_ID))
  {
			//Serial.print("* MDNS responder started. Hostname -> ");
		  //Serial.println(CLIENT_ID);
	}
	MDNS.addService("telnet", "tcp", 23);// Telnet server RemoteDebug
	Debug.setSerialEnabled(true);
	Debug.begin(CLIENT_ID);
	Debug.setResetCmdEnabled(true);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "homeB001";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void timerDeepSleep(){
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +   " Seconds");
}


void setup() {


  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  initWifi();
  timerDeepSleep();
  initRemoteDebug();
  analogSetAttenuation(ADC_11db);
  adcAttachPin(34);
  analogSetCycles(255);
  analogSetClockDiv(255); // 1338mS
  IP_adr = WiFi.localIP().toString();
  sensors.begin();     //dallas start
  dht.begin();
  timeClient.begin();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Debug.println("Setup Ok");
  delay(500);
  // put your setup code here, to run once:
}

void loop() {

  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  timeClient.update();
  minutes = timeClient.getMinutes();
  formattedTime = timeClient.getTimeStampString();
  h = dht.readHumidity();
  dtostrf(h,2, 2,h_char); // convertion float to string
  t = dht.readTemperature();
  dtostrf(t,2, 1,t_char); // convertion float to string
  sensors.requestTemperatures();
  dallas_t = sensors.getTempCByIndex(0);
  dtostrf(dallas_t,2, 1,dallas_t_char); // convertion float to string

  if (isnan(h) || isnan(t)) {
      Debug.println("Failed to read from DHT sensor!");
      delay(2000);
      publishMsg =0;
      sensorfail =1;
  }
  if (dallas_t = -127)
  {
    delay(100);
    sensors.requestTemperatures();
    dallas_t = sensors.getTempCByIndex(0);
  }

  for(i =0; i <samplect; i++)
  {
    rawadc = rawadc + analogRead(34);
  }
  // ph = (rawadc/samplect)*0.00233489;
    ph = ((rawadc/samplect)-4925)/-275;
  //https://www.mathportal.org/calculators/analytic-geometry/two-point-form-calculator.php
  // BNC to GND = 3000 (2,5V) & 7ph, Amonnique  1900 (1,65V) & 11ph
  rawadc =0;
  if ((minutes == 0)||(minutes == 10)||(minutes == 20)||(minutes == 30)||(minutes == 40)||(minutes == 50)||(sensorfail==1))
  {

      Debug.println(formattedTime);
      Debug.println("IP_adr: ");
      Debug.println(IP_adr.c_str());
      Debug.println("Mac_adr: ");
      Debug.println(WiFi.macAddress().c_str());
      Debug.print("ph: ");
      Debug.println(ph);
      Debug.print("ph_adc: ");
      Debug.println(analogRead(34));
      Debug.print("ESP32_t: ");
      Debug.println((temprature_sens_read() - 32) / 1.8);

      client.publish("esp32/temperature", t_char);
      Debug.print("dht22_t: ");
      Debug.println(t_char);
      client.publish("esp32/humidity", h_char);
      Debug.print("dht22_h: ");
      Debug.println(h_char);
      client.publish("esp32/dallas_t",dallas_t_char);
      Debug.print("dallas_t: ");
      Debug.println(dallas_t_char);
      Serial.println("Going to sleep now");
      delay(1000);
      Serial.flush();
      esp_deep_sleep_start();

  }
  digitalWrite(ledPin,  HIGH);    // turn the LED off by making the voltage LOW
  delay(200);
  sensorfail = 0;
}
