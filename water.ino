#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"

HMC5883L mag;

WiFiClient espClient;
PubSubClient client(espClient);

#define mqtt_id "WaterMeter"
#define mqtt_server ""
#define mqtt_port 1883
#define mqtt_user ""
#define mqtt_password ""

int switch_value = 70000;

int16_t mx, my, mz;

boolean state = false;
unsigned long lastSend = 0;
volatile unsigned long liters = 0;
volatile unsigned int pulses = 0;
volatile unsigned int pulsesPrSec = 0;

const float pulsesPrLiter = 120;


void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    char message[length];
    for (int i=0;i<length;i++) {
      message[i] = (char)payload[i];
    }
  	Serial.println(message);
		if(strcmp(topic, "water/liters") == 0) {
			Serial.println("Updating value for: liters");
  	 	liters = atoi(message);
		}
}

void mqttReconnect() {
	while (!client.connected()) {
    Serial.print("failed, rc=");
    Serial.println(client.state());
		if (client.connect(mqtt_id, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      
      client.subscribe("water/liters");
      
     } else {
			Serial.print("failed, rc=");
			Serial.print(client.state());
			Serial.println(" try again in 5 seconds");
			delay(5000);
		}
	}
}

void setup() {
	Serial.begin(115200);
		
	WiFiManager wifiManager;
	wifiManager.setTimeout(180);
	if(!wifiManager.autoConnect("AutoConnectAP")) {
		Serial.println("failed to connect ot wifi and hit timeout");
		delay(3000);
		//reset and try again, or maybe put it to deep sleep
		ESP.reset();
		delay(5000);
	}

	pinMode(BUILTIN_LED, OUTPUT);

	client.setServer(mqtt_server, mqtt_port);

	Serial.println("local ip");
	Serial.println(WiFi.localIP());

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize device
    Serial.println("Initializing I2C devices...");
    mag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
}

void send() {
	if(pulses >= pulsesPrSec) {
		liters++;
		pulses = 0;
	}
	if(lastSend + 5000 < millis()) {
		Serial.print("Sending value (pulses): ");
		Serial.println(pulses);
		client.publish("water/pulses", String(pulses).c_str(), true);

		Serial.print("Sending value (liters): ");
		Serial.println(liters);
		client.publish("water/liters", String(liters).c_str(), true);

		Serial.print("Sending value (pulsesPrSec): ");
		Serial.println(pulsesPrSec);
		client.publish("water/pulsesPrSec", String(pulsesPrSec).c_str(), true);
		pulsesPrSec = 0;
		
		lastSend = millis();
	}
}

void loop() {
  mqttReconnect();
	client.loop();

	// read raw heading measurements from device
	mag.getHeading(&mx, &my, &mz);
		
	int v = mx*mx+my*my;
	if((v < (switch_value - 5000) && state) || (v > (switch_value + 5000) && !state)) {
		state = !state;
		pulses++;
		pulsesPrSec++;
		digitalWrite(BUILTIN_LED, state ? HIGH : LOW);
	}
	send();
}
