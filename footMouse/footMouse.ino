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

//https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050.html
Adafruit_MPU6050 mpu;
Adafruit_Sensor *accel;

sensors_event_t event;

void setup() { // initialize the buttons' inputs:
  Mouse.begin();
  Wire.begin();
  Serial.begin(9600);

  Serial.println("Min value for accelerometer");
  Serial.println(5);

  Serial.println("Adafruit MPU6050 test!");
   // Try to initialize!
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: +-2G");
  
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  accel = mpu.getAccelerometerSensor();
  
  Serial.println("");
  delay(100);
  /**
   * TODO: setCycleRate
   */
  
}

/**
 * @brief Transforms from an accelerometer value to -1 to 1
 * @returns -1 
 */
float transformToPct(float reading){
  return 0.0;
}

void loop() {
//  delay(1000);
//  Mouse.move(0, 40);

  /* Get new sensor events with the readings 
  https://github.com/adafruit/Adafruit_Sensor
  https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050___accelerometer.html#af35d87ead6e4d5d2e11ee9f3d1bdf3dd
  
  */
  
  accel->getEvent(&event);

  /** 
   *  Print out the accelerometer values from the sensors_event_t event
   *  event.acceleration is a sensors_vec_t defined:
   *  https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h#L73
   *  (you can also get roll, pitch and heading from it)
  */
  Serial.print("X: ");
  Serial.print(event.acceleration.x);
  Serial.print("\t Y: ");
  Serial.print(event.acceleration.y);
  Serial.print("\t Z: ");
  Serial.print(event.acceleration.z);

  Serial.println("");
  delay(500);
  
}
