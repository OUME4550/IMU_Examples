#define radtodeg 57.29746936176985516473022441508
//Macros to make dataoutput easier
#define printtab(x) Serial.print(String(x) + "\t ");
#define printline() Serial.println()
//Class to hold the acc/gyro tare values
class IMU_Calibration
{
  public:
    double gyro_x_tare, gyro_y_tare, gyro_z_tare;
    double acc_x_tare, acc_y_tare, acc_z_tare;
    static const int cal_samples = 250;
    IMU_Calibration()
    {
      acc_x_tare=acc_y_tare=acc_z_tare=0.0;
      gyro_x_tare=gyro_y_tare=gyro_z_tare=0.0;
    }
};

IMU_Calibration MPU_Calibration;
