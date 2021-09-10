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
// https://adafruit.github.io/Adafruit_MPU6050/html/_adafruit___m_p_u6050_8h_source.html
Adafruit_MPU6050 mpu;

// docs for the following 3 types: https://github.com/adafruit/Adafruit_Sensor
Adafruit_Sensor *accel; // general sensor class https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h
sensors_event_t event; // x,y,x struct pulled from the sensor
#define MAX_MOUSE_MOVEMENT (15)
#define READ_DELAY (50)
#define MIN_MOUSE_MOVEMENT (4.0F) // filters out small tilts when the spring is centered

/**
 * @brief prints stuff about the sensor
 * @see the sensors_event_t event variable
 * 
 */
void printSensorInfo(){
  sensor_t sensor_info; // information struct about the sensor
  accel->getSensor(&sensor_info);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor_info.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor_info.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor_info.sensor_id);
  Serial.println  ("Max Value:    "); Serial.print(sensor_info.max_value); 
  // Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor_info.min_value); 
  // Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor_info.resolution); 
  // Serial.println(" lux");  
  Serial.println("------------------------------------");
  //  Serial.print("Min value for accelerometer:\t");
  //  Serial.println(sensor_info.min_value);
  //  Serial.print("Max value for accelerometer:\t");
  //  Serial.println(sensor_info.max_value);
  Serial.print("Accelerometer name:\t");
  Serial.println(sensor_info.name);
  
  Serial.println("");  
}

/**
 * @brief sets various settings of the mpu variable
 * @see //https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050.html
 * @see the Adafruit_MPU6050 mpu variable
 */
void setupSensor(){
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.println("Accelerometer range set to: +-2G");

  /** 
   *  setCycleRate sets how often the accelerometer populates its registers
   *  note, you can still read from it faster than 20 hz
   *  https://github.com/adafruit/Adafruit_MPU6050/blob/master/examples/sleep_demo/sleep_demo.ino
   *  https://adafruit.github.io/Adafruit_MPU6050/html/_adafruit___m_p_u6050_8h.html#a581f03aa11b55de772b4234fc392ec8b
   */
  mpu.setCycleRate(MPU6050_CYCLE_20_HZ);
  mpu.enableCycle(true); 
  Serial.println("Accelerometer Cycle Rate set to: MPU6050_CYCLE_20_HZ");
  
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); // https://adafruit.github.io/Adafruit_MPU6050/html/_adafruit___m_p_u6050_8h.html#a581f03aa11b55de772b4234fc392ec8b
}

void setup() { // initialize the buttons' inputs:
  Mouse.begin();
  Wire.begin();
  Serial.begin(9600);
  
   // Try to initialize!
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(10);
  }
  
  setupSensor();

  accel = mpu.getAccelerometerSensor(); // https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050.html#aea9a5a4ec2c966011cbdd1634b0eb2ed

  delay(1000);
}

/**
 * @brief Transforms reading to a value beteen -1 to 1. 
 * @param reading the reading from the accelerometer
 * @returns a float between -1 and 1 calculated reading
 */
float transformToPct(float reading){
  return reading / SENSORS_GRAVITY_EARTH;
}

/**
 * @brief Print out the accelerometer values from the sensors_event_t event
 * @see event.acceleration is a sensors_vec_t defined here: 
   *  https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h#L73
 */
void printEvent(){
  Serial.print(event.acceleration.pitch);
  Serial.print("\t heading: ");
  Serial.print(event.acceleration.heading);  
}

void loop() {

  /* Get new sensor events with the readings 
   *  https://github.com/adafruit/Adafruit_Sensor  
   *  https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050___accelerometer.html#af35d87ead6e4d5d2e11ee9f3d1bdf3dd
   * @see http://adafruit.github.io/Adafruit_CircuitPlayground/html/structsensors__event__t.html
  */
  
  accel->getEvent(&event);

  printEvent();
  
  // these should get optimized by the compiler???

  if ( abs(event.acceleration.x) > SENSORS_GRAVITY_EARTH || abs(event.acceleration.y) > SENSORS_GRAVITY_EARTH){
    delay(500); // delaying half a second for when someone takes their foot off
  }else{
    int x = (int) MAX_MOUSE_MOVEMENT * transformToPct(event.acceleration.x);
    int y = (int) MAX_MOUSE_MOVEMENT * transformToPct(event.acceleration.y);
    if( abs(x) > MIN_MOUSE_MOVEMENT || abs(y) > MIN_MOUSE_MOVEMENT ){
      Mouse.move(-x, y); // (x,y);
    }  
  }
  

  Serial.println("");
  delay(READ_DELAY);
}
