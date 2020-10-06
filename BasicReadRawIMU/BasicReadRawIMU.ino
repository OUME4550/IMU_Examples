#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//Delare the hardware driver here
Adafruit_MPU6050 mpu;

#define radtodeg 57.29746936176985516473022441508
//Macros to make dataoutput easier
#define printtab(x) Serial.print(String(x) + "\t ");
#define printline() Serial.println()

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
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Sets the filter bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  printtab("Ax");printtab("Ay");printtab("Az");
  printtab("Gx");printtab("Gy");printtab("Gz");
}
void loop() 
{
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  printtab(a.acceleration.x);
  printtab(a.acceleration.y);
  printtab(a.acceleration.z);
  printtab(g.gyro.x);
  printtab(g.gyro.y);
  printtab(g.gyro.z);
 
  printline();
  delay(10);
}
