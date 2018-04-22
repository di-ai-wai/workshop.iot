/**
 * IOT Workshop for Softwaren, 25.04.2018
 * 
 * (c) 2018 Stefan Peter, Andre Wölfing
 * 
 * V1.1 Blinking LED & Helligkeit mit LDR
 **/

#include <Arduino.h>

#define LED_PIN         5       // LED Pin auf LOLIN32
#define LDR_SENSOR      A0      // LDR auf analogem Eingang mit Spannungsteiler

// globale Variablen
int sensorValue;                // Speichern des Helligkeitswertes

/**
 * setup - wird einmal beim Programmstart ausgeführt
 **/
void setup() {
    Serial.begin(115200);           // starten der seriellen Console mit Baudrate 115200
    pinMode(LED_PIN, OUTPUT);       // LED_PIN wird als Output-Pin geschaltet
    Serial.println("Willkommen beim IOT Workshop!");
}

/**
 * das "Hauptmodul" wird in einem Loop ausgeführt
 **/
void loop() {
    digitalWrite(LED_PIN, HIGH);        // LED_PIN auf HIGH, d.h. LED leuchtet
    
    sensorValue = analogRead(LDR_SENSOR); // read analog input pin 0
    Serial.print(" Helligkeit: ");
    Serial.println(sensorValue, DEC);

    delay(500);     
 
    digitalWrite(LED_PIN, LOW);         // LED_PIN auf LOW, d.h. LED ist aus
    delay(500);
}