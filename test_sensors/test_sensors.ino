const int microphonePin = A6;
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

#define ADXL343_SCK 13
#define ADXL343_MISO 12
#define ADXL343_MOSI 11
#define ADXL343_CS 10

Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

float micro = 0;
int Piezo1 = 0;
int Piezo2 = 0;
float RMS = 0;
sensors_event_t event;

void setup() {
  Serial.begin(115200);

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL343 detected ... Check your wiring!");
    while(1);
  }
  
  accel.setRange(ADXL343_RANGE_2_G);
}

void loop() {
  Piezo1 = analogRead(A1);
  Piezo2 = analogRead(A2);
  accel.getEvent(&event);
  RMS = sqrt((sq(event.acceleration.x) + sq(event.acceleration.y) + sq(event.acceleration.z))/3);

  //Serial.print(millis());
  //Serial.print(",");
  Serial.print(Piezo1);
  Serial.print(",");
  Serial.print(Piezo2);
  Serial.print(",");
  Serial.println(RMS);

}
  //micro = analogRead(microphonePin);
  //Serial.print("Microphone:");
  //Serial.print(micro);
  //Serial.print(",");
  //Serial.print("P1:");
    //Serial.print("Accel:");
      //Serial.print("P2:");
