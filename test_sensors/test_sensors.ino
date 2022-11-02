

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

//sensor pins
//#define micPin A6;
#define PIEZO_PINRD1 A1
#define PIEZO_PINRD2 A2
#define PIEZO_PINSM A3
#define PIEZO_PINLG A2
#define PIEZO_PINLITE A4

//number of datapoints in one sample
#define SAMPLE_SIZE 500

//readings from sensors
//float micro = 0;
int PiezoRd1[SAMPLE_SIZE];
int PiezoRd2[SAMPLE_SIZE];
//int PiezoSm = 0;
//int PiezoLg = 0;
//int PiezoLite = 0;
float RMS[SAMPLE_SIZE];
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

  //Serial.println("Time,PiezoRd1,PiezoRd2,RMS,PiezoSm,PiezoLg,PiezoLite");
  Serial.println("PiezoRdRim,PiezoRdLeg,RMS");
}


void loop() {
  //wait for vibration
  while(analogRead(PIEZO_PINRD1) < 1);

  //get 500 readings
  //each data point is read in 1.38 ms
  for(int i=0; i<SAMPLE_SIZE; i++) {
    PiezoRd1[i] = analogRead(PIEZO_PINRD1);
    PiezoRd2[i] = analogRead(PIEZO_PINRD2);
    accel.getEvent(&event);
    RMS[i] = sqrt((sq(event.acceleration.x) + sq(event.acceleration.y) + sq(event.acceleration.z))/3);
    //PiezoSm = analogRead(PIEZO_PINSM);
    //PiezoLg = analogRead(PIEZO_PINLG);
    //PiezoLite = analogRead(PIEZO_PINLITE);
  }

  file_print_all();
  //plot_print_all();
  
  //print unique identifier after each shot
  Serial.print("#");
  Serial.println(id_num);
  id_num++;
}


void plot_print_all(){
  for(int i=0; i<SAMPLE_SIZE; i++){
    Serial.print("PiezoRd1Rim:");   
    Serial.print(PiezoRd1[i]);
    Serial.print(",");
    Serial.print("PiezoRd2Leg:");
    Serial.print(PiezoRd2[i]);
    Serial.print(",");
    Serial.print("RMS:");
    Serial.println(RMS[i]);
  
    /*
    Serial.print(",");
    Serial.print("PiezoSm:");   
    Serial.print(PiezoSm);
    Serial.print(",");
    Serial.print("PiezoLg:");   
    Serial.print(PiezoLg);
    Serial.print(",");
    Serial.print("PiezoLite:");   
    Serial.println(PiezoLite);
    */
  }
}


void file_print_all(){
  for(int i=0; i<SAMPLE_SIZE; i++){
    Serial.print(PiezoRd1[i]);
    Serial.print(",");
    Serial.print(PiezoRd2[i]);
    Serial.print(",");
    Serial.println(RMS[i]);
  
    /*
    Serial.print(",");
    Serial.print(PiezoSm);
    Serial.print(",");  
    Serial.print(PiezoLg);
    Serial.print(",");  
    Serial.println(PiezoLite);
   */
  }
}
