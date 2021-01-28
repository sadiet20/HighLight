/*
  IMU Capture
  This example uses the on-board IMU to start reading acceleration and gyroscope
  data from on-board IMU and prints it to the Serial Monitor for one second
  when the significant motion is detected.
  You can also use the Serial Plotter to graph the data.
  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 BLE Sense board.
  Created by Don Coleman, Sandeep Mistry
  Modified by Dominic Pajak, Sandeep Mistry
  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>

const float accelerationThreshold = 2.5;      //NEED TO CHANGE THIS
const int numSamples = 119;                   //NEED TO CHANGE THIS

int samplesRead = numSamples;

void setup() {
  //sets up serial monitor so arduino can send commands through USB connection
  //9600 is how fast the data is sent (baud rate)
  Serial.begin(9600);
  
  //pauses until you open the serial monitor
  while (!Serial);

  //sets up the board
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);      //infinate loop
  }

  // print the header
  Serial.println("aX,aY,aZ");
}

void loop() {
  float aX, aY, aZ;

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
      Serial.print(',');
      Serial.print(aY, 3);
      Serial.print(',');
      Serial.print(aZ, 3);
      Serial.println();

      if (samplesRead == numSamples) {
        // add an empty line if it's the last sample
        Serial.println();
      }
    }
  }
}
