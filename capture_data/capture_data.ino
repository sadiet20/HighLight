/*
  IMU Capture
  This example uses the on-board IMU to start reading acceleration data 
  from on-board IMU and prints it to the Serial Monitor for one second
  when the significant motion is detected.
  You can also use the Serial Plotter to graph the data.
  
  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 BLE Sense board.
  
  Edited by Sadie Thomas and Alex Prestwich
  Reference: Don Coleman and Dominic Pajak, Sandeep Mistry
*/

#include <Arduino_LSM9DS1.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

const float accelerationThreshold = 1.5;      //NEED TO CHANGE THIS
const int numSamples = 119;                   //NEED TO CHANGE THIS

int samplesRead = numSamples;


//which pin on the Arduino is connected to the NeoPixels
#define PIN 6

//how many NeoPixels are attached to the Arduino
#define NUMPIXELS 13

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500        //time (in milliseconds) to pause between pixels


void setup() {
  //sets up serial monitor so arduino can send commands through USB connection
  //9600 is how fast the data is sent (baud rate)
  Serial.begin(9600);
  
  //pauses until you open the serial monitor
  //while (!Serial);

  //sets up the board
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);      //infinate loop
  }

  // print the header
  Serial.println("aX,aY,aZ");

  //initilize NeoPixel strip object
  pixels.begin();
}

void loop() {
  float aX, aY, aZ;
  
  pixels.fill(pixels.Color(0, 255, 0), 0);
  pixels.show();
  // wait for significant motion
  while (samplesRead == numSamples) {
    if (IMU.accelerationAvailable()) {
      // read the acceleration data
      IMU.readAcceleration(aX, aY, aZ);

      // sum up the float absolute values
      float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

      // check if it's above the threshold
      if (aSum >= accelerationThreshold) {
        // reset the sample read count
        samplesRead = 0;
        break;
      }
    }
  }
  pixels.fill(pixels.Color(255, 0, 0), 0);
  pixels.show();
  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  while (samplesRead < numSamples) {
    // check if both new acceleration data is available
    if (IMU.accelerationAvailable()) {
      // read the acceleration data
      // units are g's
      IMU.readAcceleration(aX, aY, aZ);

      samplesRead++;

      // print the data in CSV format
      Serial.print(aX, 3);
      Serial.print(", ");
      Serial.print(aY, 3);
      Serial.print(", ");
      Serial.print(aZ, 3);
      Serial.println();

      if (samplesRead == numSamples) {
        // add an empty line if it's the last sample
        Serial.println("End");
        Serial.println();
      }
    }
  }
  pixels.fill(pixels.Color(0, 0, 255), 0);
  pixels.show();
  delay(1000);
}
