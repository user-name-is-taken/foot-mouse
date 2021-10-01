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
#include <AsyncDelay.h> // https://github.com/stevemarple/AsyncDelay

//https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050.html
// https://adafruit.github.io/Adafruit_MPU6050/html/_adafruit___m_p_u6050_8h_source.html
Adafruit_MPU6050 mpu;

// docs for the following 3 types: https://github.com/adafruit/Adafruit_Sensor
Adafruit_Sensor *accel; // general sensor class https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h
sensors_event_t event; // x,y,x struct pulled from the sensor
#define MAX_MOUSE_MOVEMENT (25)
#define READ_DELAY (10)
#define MIN_MOUSE_MOVEMENT (1.0F) // filters out small tilts when the spring is centered
#define MOVING_AVG_SIZE (50.0)
#define READING_POW (1.4F)
#define WOBBLE_MOVING_AVG_DIFF_THRESHOLD (1.25F)

AsyncDelay ASYNC_READ_DELAY; // this lets the moving averages populate as fast as possible

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
//  Serial.println("Accelerometer range set to: +-2G");

  /** 
   *  setCycleRate sets how often the accelerometer populates its registers
   *  note, you can still read from it faster than 20 hz
   *  https://github.com/adafruit/Adafruit_MPU6050/blob/master/examples/sleep_demo/sleep_demo.ino
   *  https://adafruit.github.io/Adafruit_MPU6050/html/_adafruit___m_p_u6050_8h.html#a581f03aa11b55de772b4234fc392ec8b
   */
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
  mpu.enableCycle(true); 
//  Serial.println("Accelerometer Cycle Rate set to: MPU6050_CYCLE_40_HZ");
  
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ); // https://adafruit.github.io/Adafruit_MPU6050/html/_adafruit___m_p_u6050_8h.html#a581f03aa11b55de772b4234fc392ec8b
  Serial.println("x: y:");
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
  ASYNC_READ_DELAY.start(READ_DELAY, AsyncDelay::MILLIS);
}


/**
 * @name isWobbling_x
 * @brief tests for the rapid periodic motion of the spring independently wobbling. 
 * @note the current reading is weighted as 1/4 of the the moving averages
 * @note absMovingAvg is a reference for what non-periodic motion would look like (it would always have the same sign)
 * @note movingAvg is the actual motion is.
 * @note then absMovingAvg and movingAvg are compared. If they're a lot different, the motion is periodioc. 
 * @param reading the reading from the accelerometer
 * @returns true if the movingAvg is > 25% different than expected
 * @todo run this on a separate thread, reading constantly
 */
bool isWobbling_x (float reading){
  static float absMovingAvg = abs(reading); // only initialized on the first run of absMovingAvg
  static float movingAvg = reading; // only initialized on the first run of absMovingAvg
  absMovingAvg = (abs(reading) + ((MOVING_AVG_SIZE - 1) * absMovingAvg)) / MOVING_AVG_SIZE;
  movingAvg = (reading + ((MOVING_AVG_SIZE - 1) * movingAvg)) / MOVING_AVG_SIZE;
  if( reading > SENSORS_GRAVITY_EARTH ){ 
    // gives the moving averages time to populate when you first release the spring
    return true;
  }
  return (absMovingAvg / abs(movingAvg)) > WOBBLE_MOVING_AVG_DIFF_THRESHOLD;
} 
/**
 * @name isWobbling_y
 * @brief tests if y is wobbling
 * @see isWobbling_x
 */
bool isWobbling_y (float reading){
  static float absMovingAvg = abs(reading); // only initialized on the first run of absMovingAvg
  static float movingAvg = reading; // only initialized on the first run of absMovingAvg
  absMovingAvg = (abs(reading) + ((MOVING_AVG_SIZE - 1) * absMovingAvg)) / MOVING_AVG_SIZE;
  movingAvg = (reading + ((MOVING_AVG_SIZE - 1) * movingAvg)) / MOVING_AVG_SIZE;
  if( reading > SENSORS_GRAVITY_EARTH ){ 
    // gives the moving averages time to populate when you first release the spring
    return true;
  }
  return (absMovingAvg / abs(movingAvg)) > WOBBLE_MOVING_AVG_DIFF_THRESHOLD;
}




/**
 * @brief Transforms reading to a value beteen -1 to 1. 
 * @param reading the reading from the accelerometer
 * @returns a float between -1 and 1 calculated reading
 */
float transformToPct(float reading){
  if(reading > 0){
    return pow(reading, READING_POW) / pow(SENSORS_GRAVITY_EARTH, READING_POW);  
  } else{
    return -1 * (pow(abs(reading), READING_POW)) / pow(SENSORS_GRAVITY_EARTH, READING_POW);
  }
}

/**
 * @brief Print out the accelerometer values from the sensors_event_t event
 * @see event.acceleration is a sensors_vec_t defined here: 
   *  https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h#L73
 */
void printEvent(){
  float y = event.acceleration.y;
  float x = event.acceleration.x;
  
//  static y_sum = 0;
//  static x_sum = 0;
//  Serial.print(event.acceleration.pitch);
  Serial.print(x);
  Serial.print("\t");
  Serial.println(y);
//  Serial.print(event.acceleration.heading);  
}

void loop() {

  /* Get new sensor events with the readings 
   *  https://github.com/adafruit/Adafruit_Sensor  
   *  https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050___accelerometer.html#af35d87ead6e4d5d2e11ee9f3d1bdf3dd
   * @see http://adafruit.github.io/Adafruit_CircuitPlayground/html/structsensors__event__t.html
  */
  
  accel->getEvent(&event);

  printEvent();
  
  // these should get optimized by the compiler

  if(isWobbling_x (event.acceleration.x) ||  isWobbling_y (event.acceleration.y)){
    // wait until we're not wobbling any more
    ASYNC_READ_DELAY.restart();
  } else if ( ASYNC_READ_DELAY.isExpired() ){
    int x = (int) MAX_MOUSE_MOVEMENT * transformToPct(event.acceleration.x);
    int y = (int) MAX_MOUSE_MOVEMENT * transformToPct(event.acceleration.y);
    if( abs(x) > MIN_MOUSE_MOVEMENT || abs(y) > MIN_MOUSE_MOVEMENT ){
      Mouse.move(-x, y); // (x,y);
    }
    ASYNC_READ_DELAY.repeat();
  }

  Serial.println("");
}
