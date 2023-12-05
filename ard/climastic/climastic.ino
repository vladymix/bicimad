#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Ticker.h>

#include <ClimaStick.h>

#define USERNAME "javigrz"
#define DEVICE_ID "thingerioMQTT"
#define DEVICE_CREDENTIAL "XSTld@si%RN0RG!B"

// Instancia a la clase Ticker
Ticker ticker;

// Pin LED azul
byte pinLed = D4;

ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

void parpadeoLed(){
 // Cambiar de estado el LED
byte estado = digitalRead(pinLed);
digitalWrite(pinLed, !estado);
}

void setup()
{
Serial.begin(115200);

 // Modo del pin
pinMode(pinLed, OUTPUT);

 // Empezamos el temporizador que hará parpadear el LED
ticker.attach(0.2, parpadeoLed);

 // Creamos una instancia de la clase WiFiManager
WiFiManager wifiManager;

 // Descomentar para resetear configuración
 //wifiManager.resetSettings();

 // Cremos AP y portal cautivo y comprobamos si
 // se establece la conexión
if(!wifiManager.autoConnect("ESP8266Temp")){
Serial.println("Fallo en la conexión (timeout)");
ESP.reset();
delay(1000);
}

Serial.println("Ya estás conectado");

 // Eliminamos el temporizador
ticker.detach();

 // Apagamos el LED
digitalWrite(pinLed, HIGH);

thing.init_sensors();
 // define the "environment" resource
thing.init_environment_resource();
}

void loop() {
Serial.println("Inicio");
thing.handle();
Serial.println("handle");
 // write to bucket BucketId
thing.write_bucket("data", "environment");
Serial.println("bucket");
 // sleep the device 60 seconds
thing.sleep(60);
}


