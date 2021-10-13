#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define CAL_SAMPLES 5
#include "IMUHelper.hpp"
//Delare the hardware driver here
Adafruit_MPU6050 mpu;

//Complimetery Filter Variable (lower value = faster response, but more noise)
#define tau 0.50

//Class to hold roll/pitch estimations and tares
void setup(void) 
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause until serial console opens
  }

  // Try to initialize the 6050!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    //loop forever
    while (1);
  }

  // Sets the range for accelerometer and gyroscope
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Sets the filter bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Calibration: Takes reading over a set period and averages the values
  printline("Calibrating gyro, place on level surface and do not move.");

  // Start your calibration sequence for gyro
  for (int cal_int = 0; cal_int < CAL_SAMPLES; cal_int ++)
  {
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    if(cal_int % 20 == 0)
      Serial.print(".");    // progress bar
    MPU_Calibration.acc_y_tare = (MPU_Calibration.acc_y_tare + a.acceleration.y) / 2.0;

    //acclerometer z is special case if the sensor is pointed down...
    MPU_Calibration.acc_z_tare = (MPU_Calibration.acc_z_tare - a.acceleration.z) / 2.0;

    #error Calibration NOT finished

    //time between calibration readings
    delay(1);                                   
  }
  printline();
  //take out that Z is pointing directly down
  MPU_Calibration.acc_z_tare -= 9.81;
  //These are the found calibration values...
  printline("Calibration complete...");
  printline("Ax\t Ay\t Az\t Gx\t Gy\t Gz");
  printtab(MPU_Calibration.acc_x_tare);
  printtab(MPU_Calibration.acc_y_tare);
  printtab(MPU_Calibration.acc_z_tare);
  printtab(MPU_Calibration.gyro_x_tare);
  printtab(MPU_Calibration.gyro_y_tare);
  printtab(MPU_Calibration.gyro_z_tare);
  printline();
  //Read a sample from the MPU-6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //dump our results
  printline("Raw read...");
  printline("Ax\t Ay\t Az\t Gx\t Gy\t Gz");
  printtab(a.acceleration.x);
  printtab(a.acceleration.y);
  printtab(a.acceleration.z);
  printtab(g.gyro.x);
  printtab(g.gyro.y);
  printtab(g.gyro.z);
  printline();
  printline("Calibrated read...");
  printline("Ax\t Ay\t Az\t Gx\t Gy\t Gz");

  printtab(MPU_Calibration.acc_x_tare-a.acceleration.x);
  printtab(MPU_Calibration.acc_y_tare-a.acceleration.y);
  printtab(MPU_Calibration.acc_z_tare-a.acceleration.z);
  printtab(MPU_Calibration.gyro_x_tare-g.gyro.x);
  printtab(MPU_Calibration.gyro_y_tare-g.gyro.y);
  printtab(MPU_Calibration.gyro_z_tare-g.gyro.z);
  printline();
  //make delay so humans can read the data
  delay(4000);
}

#define OUTPUT_RAW6DOF                  !true
#define OUTPUT_TARE6DOF                 false
#define OUTPUT_GYRO_INTEGRAL            false
#define OUTPUT_GYRO_CALIBRATED_INTEGRAL false
#define OUTPUT_ACC_VECTOR               false
#define OUTPUT_ACC_VECTOR_CALIBRATED    false
#define OUTPUT_COMPLEMENATARY_FILTER    false

void loop() 
{
  // Determines dt for use in the complementary filter
  static unsigned long lastTime = 0;
  unsigned long micros_now = micros();
  double dt = (micros_now - lastTime)/1e6;
  //save the time for next loop
  lastTime = micros_now;
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Raw 6dof
  if(OUTPUT_RAW6DOF)
  {
    printtab(a.acceleration.x);
    printtab(a.acceleration.y);
    printtab(a.acceleration.z);
    printtab(g.gyro.x);
    printtab(g.gyro.y);
    printtab(g.gyro.z);
  }
  if(OUTPUT_TARE6DOF)
  {
    printtab(MPU_Calibration.acc_x_tare-a.acceleration.x);
    printtab(MPU_Calibration.acc_y_tare-a.acceleration.y);
    printtab(MPU_Calibration.acc_z_tare-a.acceleration.z);
    printtab(MPU_Calibration.gyro_x_tare-g.gyro.x);
    printtab(MPU_Calibration.gyro_y_tare-g.gyro.y);
    printtab(MPU_Calibration.gyro_z_tare-g.gyro.z);
  }
  //Gyroscope integration only
  if(OUTPUT_GYRO_INTEGRAL)
  {
    //save this because of integration
    static double raw_g_pitch_angle_rad = 0;
    //do the pitch calc
    raw_g_pitch_angle_rad += g.gyro.x * dt;
    printtab(raw_g_pitch_angle_rad * radtodeg);
    //ROLL????
    //
    #warning roll using gyro not done
  }
  //Gyroscope integration with calibration
  if(OUTPUT_GYRO_CALIBRATED_INTEGRAL)
  {
    //save this because of integration
    static double pitch_angle_rad = 0;
    //do the pitch calc
    pitch_angle_rad += (MPU_Calibration.gyro_x_tare - g.gyro.x) * dt;
    printtab(pitch_angle_rad * radtodeg);
    //ROLL????
    //
    #warning roll using tared gyro not done
  }
  //Find the acclerometer vector
  if(OUTPUT_ACC_VECTOR)
  {
    double raw_acc_x, raw_acc_y, raw_acc_z;
    raw_acc_x = a.acceleration.x;
    raw_acc_y = a.acceleration.y;
    //need to invert Z if the sensor is facing down
    raw_acc_z = -a.acceleration.z;

  
    double accelPitch_rad  = -atan2(raw_acc_y, raw_acc_z);
    printtab(accelPitch_rad * radtodeg);
  } 
  //Find the acclerometer vector
  if(OUTPUT_ACC_VECTOR_CALIBRATED)
  {
    double Craw_acc_x, Craw_acc_y, Craw_acc_z;
    Craw_acc_y = MPU_Calibration.acc_y_tare - a.acceleration.y;
    Craw_acc_x = MPU_Calibration.acc_x_tare - a.acceleration.x;
    //tare is added to the z reading to handle sensor upside down
    Craw_acc_z = MPU_Calibration.acc_x_tare + a.acceleration.z;

    //z is *-1 to change output to +/- 0 instead of +/- 180
    double accelPitch_rad  = atan2(Craw_acc_y, -Craw_acc_z);
    printtab(accelPitch_rad * radtodeg);
    //ROLL????
    #warning roll using tared vector not done
  } 
  // Complemetary filter goes here
  if(OUTPUT_COMPLEMENATARY_FILTER)
  {
    //Find dt in seconds (most accurate with micros())
    //Calc. the calibrated 6dof measurements
    //Find the accleration vector angles (Z/Y and Z/X)
    //Invert the Z value to avoid gimbal lock unless your sensor is upside down
    //Calc. the gyro calibrated integrals for x and y
    //Apply the complementary filter
    //Profit? Tendies?

    #warning complementary filter not implemented

    double CFpitch_angle_rad = 0;
    printtab(CFpitch_angle_rad * radtodeg);
    //printtab(CFroll_angle_rad);
  }
  printline();
  delay(1);
}
