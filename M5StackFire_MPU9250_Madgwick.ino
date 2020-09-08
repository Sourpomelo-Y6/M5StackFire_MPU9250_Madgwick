/* 
  M5Stack Fire
    
  MPU9250 Basic Example Code
  
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
  Modified by Brent Wilkins July 19, 2016

  reduced to the minimum by ChrisMicro, September 2018

  Demonstrate basic MPU-9250 functionality including parameterizing the register
  addresses, initializing the sensor, getting properly scaled accelerometer,
  gyroscope, and magnetometer data out. Added display functions to allow display
  to on breadboard monitor.
*/

#include <M5Stack.h>
#include "utility/MPU9250.h"

#define FLT_MIN -3.4028234663852886e+38
#define FLT_MAX  3.4028234663852886e+38

#include <Preferences.h>
Preferences preferences;

MPU9250 IMU;
#include "MadgwickAHRS.h"
Madgwick *filter = new Madgwick();

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

double sum_accX = 0.0;
double sum_accY = 0.0;
double sum_accZ = 0.0;

double sum_gyroX = 0.0;
double sum_gyroY = 0.0;
double sum_gyroZ = 0.0;

float calb_accX = 0.0;
float calb_accY = 0.0;
float calb_accZ = 0.0;

float calb_gyroX = 0.0;
float calb_gyroY = 0.0;
float calb_gyroZ = 0.0;

float magX_min = 0.0;
float magY_min = 0.0;
float magZ_min = 0.0;

float magX_max = 0.0;
float magY_max = 0.0;
float magZ_max = 0.0;

void setup()
{
  M5.begin();
  Wire.begin();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(1);

  preferences.begin("imu_calb_data", true);
  calb_accX = preferences.getFloat("ax", 0);
  calb_accY = preferences.getFloat("ay", 0);
  calb_accZ = preferences.getFloat("az", 0);
  calb_gyroX = preferences.getFloat("gx", 0);
  calb_gyroY = preferences.getFloat("gy", 0);
  calb_gyroZ = preferences.getFloat("gz", 0);
  
  magX_min = preferences.getFloat("mx_min", 0);
  magY_min = preferences.getFloat("my_min", 0);
  magZ_min = preferences.getFloat("mz_min", 0);
  magX_max = preferences.getFloat("mx_max", 0);
  magY_max = preferences.getFloat("my_max", 0);
  magZ_max = preferences.getFloat("mz_max", 0);
  
  Serial.println(calb_accX);
  Serial.println(calb_accY);
  Serial.println(calb_accZ);
  Serial.println(calb_gyroX);
  Serial.println(calb_gyroY);
  Serial.println(calb_gyroZ);
  
  Serial.println(magX_min);
  Serial.println(magY_min);
  Serial.println(magZ_min);
  Serial.println(magX_max);
  Serial.println(magY_max);
  Serial.println(magZ_max);
  preferences.end();
  
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();
  IMU.initAK8963(IMU.magCalibration);
}

unsigned int pre_time =0;
void loop()
{
  M5.update();
  
  if(getIMUData())
  {
    int x=64+10;
    int y=128+20;
    int z=192+30;
    
    M5.Lcd.setCursor(0, 0); M5.Lcd.print("MPU9250/AK8963");
    M5.Lcd.setCursor(0, 32); M5.Lcd.print("x");
    M5.Lcd.setCursor(x, 32); M5.Lcd.print("y");
    M5.Lcd.setCursor(y, 32); M5.Lcd.print("z");

    M5.Lcd.setTextColor(YELLOW , BLACK);
    M5.Lcd.setCursor(0, 48); M5.Lcd.printf("%5d  ",(int)(1000 * IMU.ax));
    M5.Lcd.setCursor(x, 48); M5.Lcd.printf("%5d  ",(int)(1000 * IMU.ay));
    M5.Lcd.setCursor(y, 48); M5.Lcd.printf("%5d  ",(int)(1000 * IMU.az));
    M5.Lcd.setCursor(z, 48); M5.Lcd.print("mg");

    M5.Lcd.setCursor(0, 64); M5.Lcd.printf("%5d  ",(int)(IMU.gx));
    M5.Lcd.setCursor(x, 64); M5.Lcd.printf("%5d  ",(int)(IMU.gy));
    M5.Lcd.setCursor(y, 64); M5.Lcd.printf("%5d  ",(int)(IMU.gz));
    M5.Lcd.setCursor(z, 64); M5.Lcd.print("o/s");

    M5.Lcd.setCursor(0, 80); M5.Lcd.printf("%5d  ",(int)(IMU.mx));
    M5.Lcd.setCursor(x, 80); M5.Lcd.printf("%5d  ",(int)(IMU.my));
    M5.Lcd.setCursor(y, 80); M5.Lcd.printf("%5d  ",(int)(IMU.mz));
    M5.Lcd.setCursor(z, 80); M5.Lcd.print("mG");

    M5.Lcd.setCursor(0, 96); M5.Lcd.printf("%5.3f  ",pitch);
    M5.Lcd.setCursor(x, 96); M5.Lcd.printf("%5.3f  ",roll);
    M5.Lcd.setCursor(y, 96); M5.Lcd.printf("%5.3f  ",yaw);
    M5.Lcd.setCursor(z, 96); M5.Lcd.printf("deg");
    
    //M5.Lcd.setCursor(0,  96); M5.Lcd.print("Gyro Temperature ");
    //M5.Lcd.setCursor(z,  96); M5.Lcd.print(IMU.temperature, 1);
    //M5.Lcd.print(" C");
    //delay(100);

    //Serial.print(millis()-pre_time);
    //Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print("\r\n");
    
    int deftime = millis() - pre_time;
    while(deftime < 10)
    {
      delay(1);
      deftime = millis() - pre_time;
    }
    M5.Lcd.setCursor(0, 112); M5.Lcd.printf("%5d",deftime);
    pre_time = millis();

  }

  if(M5.BtnA.isPressed()){
    Serial.println("Calibration Start");
    getCalibrationVal();
  }else if(M5.BtnB.isPressed()){
    Serial.println("Mag Calibration Start");
    getMagCalibrationVal();
  }
  
}

bool getIMUData()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres();

    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;

    IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
    IMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    //IMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    //IMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    //IMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] -
             IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] -
             IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] -
             IMU.magbias[2];

    IMU.tempCount = IMU.readTempData();  // Read the adc values
    // Temperature in degrees Centigrade
    IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;

//    filter->MadgwickAHRSupdateIMU(
//      PI/180.0F*(IMU.gx - calb_gyroX), 
//      PI/180.0F*(IMU.gy - calb_gyroY), 
//      PI/180.0F*(IMU.gz - calb_gyroZ),
//      IMU.ax - calb_accX, 
//      IMU.ay - calb_accY, 
//      IMU.az - calb_accZ
//    );

    filter->MadgwickAHRSupdate(
      PI/180.0F*(IMU.gx - calb_gyroX), 
      PI/180.0F*(IMU.gy - calb_gyroY), 
      PI/180.0F*(IMU.gz - calb_gyroZ),
      IMU.ax - calb_accX, 
      IMU.ay - calb_accY, 
      IMU.az - calb_accZ,
      (2.0*IMU.my-magY_max-magY_min)/(magY_max-magY_min),
      (2.0*IMU.mx-magX_max-magX_min)/(magX_max-magX_min),
      -(2.0*IMU.mz-magZ_max-magZ_min)/(magZ_max-magZ_min)
    );
  
    pitch = filter->getPitch()*180.0F/PI;
    roll = filter->getRoll()*180.0F/PI;
    yaw = filter->getYaw()*180.0F/PI;

    return true;
  }

  return false;
}

void getCalibrationVal()
{
  const int CalbNum = 1000;
  sum_accX = 0.0;
  sum_accY = 0.0;
  sum_accZ = 0.0;

  sum_gyroX = 0.0;
  sum_gyroY = 0.0;
  sum_gyroZ = 0.0;  
    
  for(int i = 4 ; i >= 0 ; i--){
    M5.Lcd.setCursor(0, 128);
    M5.Lcd.printf("Start Calb %d sec",i);
    delay(1000);
  }
    
  for(int i = 0 ; i < CalbNum ; i++){

      if(getIMUData()){
        sum_accX += IMU.ax;
        sum_accY += IMU.ay;
        sum_accZ += IMU.az;

        sum_gyroX += IMU.gx;
        sum_gyroY += IMU.gy;
        sum_gyroZ += IMU.gz;
        
        int deftime = millis() - pre_time;
        while(deftime < 10)
        {
          delay(1);
          deftime = millis() - pre_time;
        }
        M5.Lcd.setCursor(0, 112); M5.Lcd.printf("%5d",deftime);
        pre_time = millis();

        M5.Lcd.setCursor(0, 128);
        M5.Lcd.printf("Calb count : %3d",CalbNum-i);
      }
  }

  calb_accX = (float)(sum_accX/CalbNum);
  calb_accY = (float)(sum_accY/CalbNum);
  calb_accZ = (float)(sum_accZ/CalbNum) - 1.0F;

  calb_gyroX = (float)(sum_gyroX/CalbNum);
  calb_gyroY = (float)(sum_gyroY/CalbNum);
  calb_gyroZ = (float)(sum_gyroZ/CalbNum);

  preferences.begin("imu_calb_data", false);
  preferences.putFloat("ax", calb_accX);
  preferences.putFloat("ay", calb_accY);
  preferences.putFloat("az", calb_accZ);
  preferences.putFloat("gx", calb_gyroX);
  preferences.putFloat("gy", calb_gyroY);
  preferences.putFloat("gz", calb_gyroZ);
  preferences.end();
  
  M5.Lcd.setCursor(0, 128);
  M5.Lcd.printf("End Calb");
}


void getMagCalibrationVal()
{
  const int CalbNum = 1000;
  magX_min = FLT_MAX;
  magY_min = FLT_MAX;
  magZ_min = FLT_MAX;

  magX_max = FLT_MIN;
  magY_max = FLT_MIN;
  magZ_max = FLT_MIN;  
    
  for(int i = 4 ; i >= 0 ; i--){
    M5.Lcd.setCursor(0, 128);
    M5.Lcd.printf("Start Mag Calb %d sec",i);
    delay(1000);
  }
    
  for(int i = 0 ; i < CalbNum ; i++){

      if(getIMUData()){

        if(magX_min > IMU.mx){
          magX_min = IMU.mx;
        }
        
        if(magY_min > IMU.my){
          magY_min = IMU.my;
        }
        
        if(magZ_min > IMU.mz){
          magZ_min = IMU.mz;
        }

        if(magX_max < IMU.mx){
          magX_max = IMU.mx;
        }
        
        if(magY_max < IMU.my){
          magY_max = IMU.my;
        }
        
        if(magZ_max < IMU.mz){
          magZ_max = IMU.mz;
        }

        int deftime = millis() - pre_time;
        while(deftime < 10)
        {
          delay(1);
          deftime = millis() - pre_time;
        }
        M5.Lcd.setCursor(0, 112); M5.Lcd.printf("%5d",deftime);
        pre_time = millis();

        M5.Lcd.setCursor(0, 128);
        M5.Lcd.printf("Calb count : %3d",CalbNum-i);
      }
  }

  preferences.begin("imu_calb_data", false);
  preferences.putFloat("mx_min", magX_min);
  preferences.putFloat("my_min", magY_min);
  preferences.putFloat("mz_min", magZ_min);
  preferences.putFloat("mx_max", magX_max);
  preferences.putFloat("my_max", magY_max);
  preferences.putFloat("mz_max", magZ_max);
  preferences.end();
  
  M5.Lcd.setCursor(0, 128);
  M5.Lcd.printf("End Calb");
}
