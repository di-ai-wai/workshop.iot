/**
 * IOT Workshop for Softwaren, 25.04.2018
 * 
 * (c) 2018 Stefan Peter, Andre Wölfing
 * 
 * V1.0 Blinking LED
 * V1.1 Helligkeit mit LDR
 * V1.2 Zusätzlich Temperatur und Feuchtigkeit messen
 * V1.3 Temperatur und Luftdruck mit BME280 auslesen
 * V1.4 WiFi-enabling
 * V1.5 Ab in die IOT-Welt: MQTT
 * V1.6 Umbau LED für den Internet-Button
 **/

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <BME280.h>
#include <WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#define LED_PIN         5       // LED Pin auf LOLIN32
#define LDR_SENSOR      A0      // LDR auf analogem Eingang mit Spannungsteiler
#define DHT_PIN         A13     // Input Pin für DHT-Sensor
#define DHT_BLAU        DHT11
#define DHT_WEISS       DHT22

// globale Variablen
int sensorValue;                // Speichern des Helligkeitswertes
float dht_humidity;             // Feuchtigkeit
float dht_temp;                 // Temperaturvariable
float bme_temp;                 // Vergleichstemperatur
float bme_press;                // Luftdruck

// WiFi Infos
const char* ssid     = "iot";
const char* password = "10T-W0rksh0P";

//MQTT broker settings
#define HOST        "192.168.0.10"
#define PORT        1883
#define USERNAME    "iot"
#define PASSWORD    "w0rksh0p"

#define CLIENTID          "iot01/"
#define PRE_WS            "weatherStation/"
#define PRE_SR            "sleepingRoom/"
#define T_TEMPERATURE     "temperature"
#define T_PRESSURE        "pressure"
#define T_HUMIDITY        "humidity"
#define T_LIGHT           "light"
#define T_LAMP            "lamp"
#define T_CLIENTSTATUS    "clientStatus"
#define T_COMMAND         "command"

/* Setup des DHT-Sensors, abhängig vom Typ */
DHT dht(DHT_PIN, DHT_WEISS);

/* der BME280 wird über I2C angeschlossen SLC/SDA */
BME280 bme;

int toggle = LOW;

/* MQTT Publish/Subscribe */

// Callback function header
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, HOST, PORT, USERNAME, PASSWORD);
Adafruit_MQTT_Client mqtt2(&client, HOST, PORT, USERNAME, PASSWORD);

// Publications
const char TEMPERATURE_FEED[] PROGMEM = CLIENTID PRE_WS T_TEMPERATURE;
const char PRESSURE_FEED[] PROGMEM = CLIENTID PRE_WS T_PRESSURE;
const char HUMIDITY_FEED[] PROGMEM = CLIENTID PRE_WS T_HUMIDITY;
const char LIGHT_FEED[] PROGMEM = CLIENTID PRE_WS T_LIGHT;
const char LAMP_FEED[] PROGMEM = CLIENTID PRE_SR T_LAMP;
const char COMMAND_FEED[] PROGMEM = CLIENTID PRE_SR T_COMMAND;

Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);
Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, PRESSURE_FEED);
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);
Adafruit_MQTT_Publish light = Adafruit_MQTT_Publish(&mqtt, LIGHT_FEED);

// Subscriptions
Adafruit_MQTT_Subscribe command = Adafruit_MQTT_Subscribe(&mqtt2, LAMP_FEED);

/**
 * Verbinde mit WiFi mit den angegebenen Credentials
 **/
void connectToWiFi() {
    Serial.println("\n");
    Serial.print("Verbinde ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi verbunden");
    Serial.println("IP Adresse: ");
    Serial.println(WiFi.localIP());
    Serial.println();  
}

/**
 * Verbinde mit MQTT Broker
 **/
void connectToMQTT() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Verbinde mit MQTT Broker auf ");
  Serial.println(HOST);
  while ((ret = mqtt.connect()) != 0) {
       //Serial.println(mqtt.connectErrorString(ret));
       //Serial.println("Retrying MQTT connection in 1 second...");
       mqtt.disconnect();
       delay(1000);
  }
  Serial.println("MQTT ist verbunden!\n");
}

/**
 * Callback für den Internet-Button
 **/
void onoffcallback(char *data, uint16_t len) {
  Serial.print("Callback für OnOff-LED wurde gerufen mit payload ");
  Serial.println(data);
  toggle = !toggle;
  digitalWrite(13, toggle);
}

int receiveCommand() {
    Serial.println("Trying to receive a command");
  int clientSt = 99;
  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt2.readSubscription(10))) {
      Serial.println("readinf subscription");
      if (subscription == &command) {
        Serial.print(F("Got: "));
        Serial.println((char*)command.lastread);
        toggle = !toggle;
        digitalWrite(LED_PIN, toggle);
      }
  }
  return clientSt;
}

/**
 * setup - wird einmal beim Programmstart ausgeführt
 **/
void setup() {
    Serial.begin(115200);           // starten der seriellen Console mit Baudrate 115200
    Serial.println("Willkommen beim IOT Workshop!");
    pinMode(LED_PIN, OUTPUT);       // LED_PIN wird als Output-Pin geschaltet
    dht.begin();                    // Starten des Sensors
    Serial.println("DHT Sensor initialisiert...");

    /* BME280 initialisieren */
    bme.settings(0, 3, 5, 4, 3, 3, 3);
    if (!bme.begin()) {
        Serial.println("Konnte keinen BMP280 sensor finden, bitte prüfe Verkabelung!");
        while (1);
    }
    Serial.println("BME280 Sensor initialisiert...");

    /* WiFi */
    connectToWiFi();
    /* MQTT */
    connectToMQTT();
    delay(100);
    /* setze den Callback und subscribe für den Event */
    command.setCallback(onoffcallback);
    if(mqtt2.subscribe(&command)) {
        Serial.print("subscribed to ");
        Serial.println(command.topic);
    }
}

/**
 * das "Hauptmodul" wird in einem Loop ausgeführt
 **/
void loop() {
    // receiveCommand();

    sensorValue = analogRead(LDR_SENSOR); // read analog input pin 0
    /*
    Serial.print(" Helligkeit: ");
    Serial.println(sensorValue, DEC);
    */

    /* Auslesen Feuchtigkeit und Temperatur */
    dht_humidity = dht.readHumidity();
    dht_temp = dht.readTemperature();
    /*
    Serial.print(" DHT: ");
    Serial.print(dht_temp);
    Serial.print("ºC   ");
    Serial.print(dht_humidity);
    Serial.println("%");
    */

    /* Auslesen BME280 */
    bme_temp = bme.readTemp();
    bme_press = bme.readPressure();
    /*
    Serial.print(" BME: ");
    Serial.print(bme_temp);
    Serial.print("ºC   ");
    Serial.print(bme_press);
    Serial.println("Pa");
    */

    /* Publish to MQTT */
    temperature.publish(bme_temp);
    pressure.publish(bme_press);
    humidity.publish(dht_humidity);
    light.publish(sensorValue);
    delay(5000);
}