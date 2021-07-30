/*
  WARNING: When you use the Mouse.move() command, the Arduino takes over your
  mouse! Make sure you have control before you use the mouse commands.
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/KeyboardAndMouseControl
*/

#include "Mouse.h"


// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 parent_mpu;
Adafruit_MPU6050_Accelerometer child_mpu = Adafruit_MPU6050_Accelerometer(&parent_mpu);
Adafruit_Sensor *mpu_temp;

void setup() { // initialize the buttons' inputs:
  Mouse.begin();
  Wire.begin();
  Serial.begin(9600);

  Serial.println("Adafruit MPU6050 test!");
   // Try to initialize!
  while (!parent_mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(10);
  }
  Serial.println("MPU6050 Found!");

  parent_mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: +-2G");
  
  parent_mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  Serial.println("");
  delay(100);
  /**
   * TODO: setCycleRate
   */
  
}

void loop() {
//  delay(1000);
//  Mouse.move(0, 40);


  /* Get new sensor events with the readings */
  sensors_event_t event;
  child_mpu.getEvent(&event);

  /* Print out the values */
  Serial.print("X: ");
  Serial.print(event.acceleration.x);
  Serial.print("\t Y: ");
  Serial.print(event.acceleration.y);
  Serial.print("\t Z: ");
  Serial.print(event.acceleration.z);
//  Serial.print(" m/s^2");

//  Serial.print("Rotation X: ");
//  Serial.print(g.gyro.x);
//  Serial.print(", Y: ");
//  Serial.print(g.gyro.y);
//  Serial.print(", Z: ");
//  Serial.print(g.gyro.z);
//  Serial.print(" rad/s");
//
//  Serial.print("Temperature: ");
//  Serial.print(temp.temperature);
//  Serial.print(" degC");

  Serial.println("");
  delay(500);
  
}
