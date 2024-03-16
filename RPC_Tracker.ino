/****************************************************************/
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1015.h>
#include <SPI.h>
/****************************************************************/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "SoftwareSerial.h"
#include <pgmspace.h>
/****************************************************************/
#include <Servo.h> // include Servo library 
/****************************************************************/
#define GPIO0 5
#define GPIO1 6
#define GPIO2 7

#define GPIO0_PIN D3
#define GPIO1_PIN D4
#define GPIO2_PIN D8
/****************************************************************/
#define OLED_RESET 0
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_INA219 ina219;
Adafruit_ADS1115 ads1115;

unsigned long previousMillis = 0;
unsigned long interval = 250;
const int chipSelect = 10;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float energy = 0;
float batt = 0;

const int SCLpin = D1;
const int SDApin = D2;
/****************************************************************/
#define WIFI_AP "CPH1609"
#define WIFI_PASSWORD "12345678"

#define TOKEN "lF6tQt3iRMfwtYizqCFY"

char thingsboardServer[] = "demo.thingsboard.io";

// Initialize the Ethernet client object
WiFiClient espClient;
PubSubClient client(espClient);
SoftwareSerial soft(2, 3); // RX, TX

int status = WL_IDLE_STATUS;
unsigned long lastSend;
/****************************************************************/

// 180 horizontal MAX
Servo horizontal; // horizontal servo
int servoh = 77;   // 90;     // stand horizontal servo

int servohLimitHigh = 155;
int servohLimitLow = 25;

// 65 degrees MAX
Servo vertical;   // vertical servo 
int servov = 45;    //   90;     // stand vertical servo

int servovLimitHigh = 160;
int servovLimitLow = 20;


int ldrbl = 3; //Bottom Left
int ldrbr = 2; //Bottom Right
int ldrtl = 1.; //Top Left
int ldrtr = 0; //Top Right

// We assume that all GPIOs are LOW
boolean gpioState[] = {false, false, false};

void setup() {
  /****************************************************************/
  Wire.begin(SDApin, SCLpin);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  ina219.begin();
  ads1115.begin();
  ads1115.setGain(GAIN_TWO);  
  /****************************************************************/
  // initialize serial for debugging
  Serial.begin(9600);
  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  client.setServer( thingsboardServer, 1883 );
  client.setCallback(on_message);
  lastSend = 0;
  /****************************************************************/
  // servo connections
  // name.attacht(pin);
  horizontal.attach(D6);
  vertical.attach(D7);
  delay(500);
  /****************************************************************/
  // Set output mode for all GPIO pins
  pinMode(GPIO0_PIN, OUTPUT);
  pinMode(GPIO1_PIN, OUTPUT);
  pinMode(GPIO2_PIN, OUTPUT);
  digitalWrite(GPIO0_PIN, LOW);
  digitalWrite(GPIO1_PIN, LOW);
  digitalWrite(GPIO2_PIN, LOW);
}

void loop() {
  track();
  
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(WIFI_AP);
      // Connect to WPA/WPA2 network
      status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  }

  if ( client.connected() == false ) {
    reconnect();
  }

  if ( millis() - lastSend > 1000 ) { // Update and send only after 1 seconds
    ina219values();
    battvoltage();
    sendpayload();
    displaydata();
    lastSend = millis();
  }

  client.loop();
}

void ina219values() {
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  energy = energy + loadvoltage * current_mA / 3600;
}

void battvoltage() {
  float offset = 0.3;
  unsigned int raw = analogRead(A0);
  float volt = raw / 1023.0;
  batt = volt * 4.2;
  batt = batt + offset;
}

void sendpayload() {
  // Prepare a JSON payload string
  String payload = "{";
  payload += "\"loadvoltage\":"; payload += loadvoltage; payload += ",";
  payload += "\"current\":"; payload += current_mA; payload += ",";
  payload += "\"power\":"; payload += loadvoltage * current_mA; payload += ",";
  payload += "\"battvoltage\":"; payload += batt;
  payload += "}";

  // Send payload
  char telemetrys[100];
  payload.toCharArray( telemetrys, 100 );
  client.publish( "v1/devices/me/telemetry", telemetrys );
  Serial.println( telemetrys );
}


void displaydata() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(loadvoltage);
  display.setCursor(35, 0);
  display.println("V");
  display.setCursor(50, 0);
  display.println(current_mA);
  display.setCursor(95, 0);
  display.println("mA");
  display.setCursor(0, 10);
  display.println(loadvoltage * current_mA);
  display.setCursor(65, 10);
  display.println("mW");
  display.setCursor(0, 20);
  display.println("BATT");
  display.setCursor(30, 20);
  display.println(batt);
  display.setCursor(65, 20);
  display.println("V");
  display.display();
}

// The callback for when a PUBLISH message is received from the server.
void on_message(const char* topic, byte* payload, unsigned int length) {

  Serial.println("On message");

  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(json);

  // Decode JSON request
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.parseObject((char*)json);

  if (!data.success()) {
    Serial.println("parseObject() failed");
    return;
  }

  // Check request method
  String methodName = String((const char*)data["method"]);

  if (methodName.equals("getGpioStatus")) {
    // Reply with GPIO status
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
  } else if (methodName.equals("setGpioStatus")) {
    // Update GPIO status and reply
    set_gpio_status(data["params"]["pin"], data["params"]["enabled"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
  } else if (methodName.equals("setLedValueHijau")) {
    // Update GPIO status and reply
    set_gpio_status(GPIO0, data["params"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
  } else if (methodName.equals("setLedValueKuning")) {
    // Update GPIO status and reply
    set_gpio_status(GPIO1, data["params"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
  } else if (methodName.equals("setLedValueMerah")) {
    // Update GPIO status and reply
    set_gpio_status(GPIO2, data["params"]);
    String responseTopic = String(topic);
    responseTopic.replace("request", "response");
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
  }
}

String get_gpio_status() {
  // Prepare gpios JSON payload string
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();
  data[String(GPIO0)] = gpioState[0] ? true : false;
  data[String(GPIO1)] = gpioState[1] ? true : false;
  data[String(GPIO2)] = gpioState[2] ? true : false;
  char payload[256];
  data.printTo(payload, sizeof(payload));
  String strPayload = String(payload);
  Serial.print("Get gpio status: ");
  Serial.println(strPayload);
  return strPayload;
}

void set_gpio_status(int pin, boolean enabled) {
  if (pin == GPIO0) {
    // Output GPIOs state
    digitalWrite(GPIO0_PIN, enabled ? HIGH : LOW);
    // Update GPIOs state
    gpioState[0] = enabled;
  } else if (pin == GPIO1) {
    // Output GPIOs state
    digitalWrite(GPIO1_PIN, enabled ? HIGH : LOW);
    // Update GPIOs state
    gpioState[1] = enabled;
  } else if (pin == GPIO2) {
    // Output GPIOs state
    digitalWrite(GPIO2_PIN, enabled ? HIGH : LOW);
    // Update GPIOs state
    gpioState[2] = enabled;
  }
}

void reconnect() {
    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("D1 Mini Device", TOKEN, NULL) ) {
      Serial.println( "[DONE]" );
      // Subscribing to receive RPC requests
      client.subscribe("v1/devices/me/rpc/request/+");
      // Sending current GPIO status
      Serial.println("Sending current GPIO status ...");
      client.publish("v1/devices/me/attributes", get_gpio_status().c_str());
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 3 seconds]" );
      // Wait 3 seconds before retrying
      delay( 3000 );
    }
}

void track(){
  int16_t  bl = map(ads1115.readADC_SingleEnded(ldrbl),0,32767,0,8192); // Bottom left
  int16_t  br = map(ads1115.readADC_SingleEnded(ldrbr),0,32767,0,8192); // Bottom Right
  int16_t  tl = map(ads1115.readADC_SingleEnded(ldrtl),0,32767,0,8192); // Top Left
  int16_t  tr = map(ads1115.readADC_SingleEnded(ldrtr),0,32767,0,8192); // Top Right

  Serial.println("");
  Serial.print(bl);
  Serial.println(" Bottom left ");
  Serial.print(br);
  Serial.println(" Bottom Right ");
  Serial.print(tl);
  Serial.println(" Top Left ");
  Serial.print(tr);
  Serial.println(" Top Right");
  
  int percentb, percentt, percentl, percentr;
  int dtime = 50; int tol = 10;
  
  int avb = (bl + br) / 2; // average value bottom
  int avt = (tl + tr) / 2; // average value top
  int avl = (bl + tl) / 2; // average value left
  int avr = (br + tr) / 2; // average value right
  
  percentb = map(avb,0,8192,0,100); 
  percentt = map(avt,0,8192,0,100); 
  percentl = map(avl,0,8192,0,100); 
  percentr = map(avr,0,8192,0,100);
  
  int dvert = avt - avb; // check the diffirence of top and bottom
  int dhoriz = avl - avr;// check the diffirence of left and rigt

  Serial.print(avt);
  Serial.print(" ");
  Serial.print(avb);
  Serial.print(" ");
  Serial.print(avl);
  Serial.print(" ");
  Serial.print(avr);
  Serial.print("   ");
  Serial.print(dtime);
  Serial.print("   ");
  Serial.print(tol);
  Serial.println(" ");
  Serial.print(dvert);
  Serial.println(" ");
  Serial.print(dhoriz);
  Serial.println(" ");
  Serial.print(percentt);
  Serial.print(" ");
  Serial.print(percentb);
  Serial.print(" ");
  Serial.print(percentl);
  Serial.print(" ");
  Serial.print(percentr);

  if (-1*tol > dvert || dvert > tol){ // check if the diffirence is in the tolerance else change vertical angle
  if (avt > avb){
    servov = ++servov;
    if (servov > servovLimitHigh){ 
      servov = servovLimitHigh;
    }
  } else if (avt < avb){
    servov= --servov;
    if (servov < servovLimitLow){
      servov = servovLimitLow;
    }
  } else if (avt = avb){
    // nothing
  }
  vertical.write(servov);
  }
  if (-1 * tol > dhoriz || dhoriz > tol){ // check if the diffirence is in the tolerance else change horizontal angle
  if (avl > avr){
     servoh = ++servoh;
     if (servoh > servohLimitHigh){
     servoh = servohLimitHigh;
     }
  } else if (avl < avr){
    servoh = --servoh;
    if (servoh < servohLimitLow){
    servoh = servohLimitLow;
    }
  } else if (avl = avr){
    // nothing
  }
  horizontal.write(servoh);
  }
   delay(100);
}
