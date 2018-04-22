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
 **/

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <BME280.h>
#include <WiFi.h>

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

/* Setup des DHT-Sensors, abhängig vom Typ */
DHT dht(DHT_PIN, DHT_WEISS);

/* der BME280 wird über I2C angeschlossen SLC/SDA */
BME280 bme;

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
    Serial.println("\n");  
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
}

/**
 * das "Hauptmodul" wird in einem Loop ausgeführt
 **/
void loop() {
    digitalWrite(LED_PIN, HIGH);        // LED_PIN auf HIGH, d.h. LED leuchtet
    
    sensorValue = analogRead(LDR_SENSOR); // read analog input pin 0
    Serial.print(" Helligkeit: ");
    Serial.println(sensorValue, DEC);

    /* Auslesen Feuchtigkeit und Temperatur */
    dht_humidity = dht.readHumidity();
    dht_temp = dht.readTemperature();
    Serial.print(" DHT: ");
    Serial.print(dht_temp);
    Serial.print("ºC   ");
    Serial.print(dht_humidity);
    Serial.println("%");

    /* Auslesen BME280 */
    bme_temp = bme.readTemp();
    bme_press = bme.readPressure();
    Serial.print(" BME: ");
    Serial.print(bme_temp);
    Serial.print("ºC   ");
    Serial.print(bme_press);
    Serial.println("Pa");

    delay(500);     
 
    digitalWrite(LED_PIN, LOW);         // LED_PIN auf LOW, d.h. LED ist aus
    delay(500);
}