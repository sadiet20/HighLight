

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

//sensor pins
//#define micPin A6;
#define PIEZO_PIN1 A1
#define PIEZO_PIN2 A2

//readings from sensors
float micro = 0;
int Piezo1 = 0;
int Piezo2 = 0;
float RMS = 0;
sensors_event_t event;

//create accelerometer object with unique ID
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

unsigned int id_num = 0;

void setup() {
  Serial.begin(115200);

  if(!accel.begin())
  {
    // There was a problem detecting the ADXL343 ... check your connections
    Serial.println("Ooops, no ADXL343 detected ... Check your wiring!");
    while(1);
  }

  //set maximum for accelerometer readings
  accel.setRange(ADXL343_RANGE_8_G);

  Serial.println("Time,Piezo1,Piezo2,RMS");
}

void loop() {
  //wait for vibration
  while(analogRead(PIEZO_PIN1) < 1);

  //get 500 readings
  for(int i=0; i<500; i++) {
    Piezo1 = analogRead(PIEZO_PIN1);
    Piezo2 = analogRead(PIEZO_PIN2);
    accel.getEvent(&event);
    RMS = sqrt((sq(event.acceleration.x) + sq(event.acceleration.y) + sq(event.acceleration.z))/3);

    Serial.print(millis());
    Serial.print(",");
    Serial.print(Piezo1);
    Serial.print(",");
    Serial.print(Piezo2);
    Serial.print(",");
    Serial.println(RMS);
  }

  //print unique identifier after each shot
  Serial.print("#");
  Serial.println(id_num);
  id_num++;

}
