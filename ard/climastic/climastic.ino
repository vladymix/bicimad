
#include <ClimaStick.h>

#define USERNAME "fabricio"
#define DEVICE_ID "Bicimap"
#define DEVICE_CREDENTIAL "Zah9jS#f9m$&"

#define SSID "SBC"
#define SSID_PASSWORD "SBCwifi$"


ClimaStick thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
 // configure board wifi
  thing.add_wifi(SSID, SSID_PASSWORD);
  // initialize board sensors
  thing.init_sensors();
  // define resources for all features
  thing.init_resources();
}

void loop() {
  // put your main code here, to run repeatedly:
   thing.handle();

    // write to bucket BucketId
    thing.write_bucket("storage_climastick", "environment");

     Serial.println("Ready Gyroscope");
  
    float temperature = thing.get_temperature();
    thing.get_acceleration();
    Serial.println(temperature);


    // sleep the device 60 seconds
   thing.sleep(60);

}
