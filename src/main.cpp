/**
 * IOT Workshop for Softwaren, 25.04.2018
 * 
 * (c) 2018 Stefan Peter, Andre Wölfing
 * 
 * V1.2 Zusätzlich Temperatur und Feuchtigkeit messen
 **/

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define LED_PIN         5       // LED Pin auf LOLIN32
#define LDR_SENSOR      A0      // LDR auf analogem Eingang mit Spannungsteiler
#define DHT_PIN         A13     // Input Pin für DHT-Sensor
#define DHT_BLAU        DHT11
#define DHT_WEISS       DHT22

// globale Variablen
int sensorValue;                // Speichern des Helligkeitswertes
float dht_humidity;             // Feuchtigkeit
float dht_temp;                 // Temperaturvariable

/* Setup des DHT-Sensors, abhängig vom Typ */
DHT dht(DHT_PIN, DHT_WEISS);

/**
 * setup - wird einmal beim Programmstart ausgeführt
 **/
void setup() {
    Serial.begin(115200);           // starten der seriellen Console mit Baudrate 115200
    Serial.println("Willkommen beim IOT Workshop!");
    pinMode(LED_PIN, OUTPUT);       // LED_PIN wird als Output-Pin geschaltet
    dht.begin();                    // Starten des Sensors
    Serial.println("DHT Sensor initialisiert...");
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

    delay(500);     
 
    digitalWrite(LED_PIN, LOW);         // LED_PIN auf LOW, d.h. LED ist aus
    delay(500);
}