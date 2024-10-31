
#include <Arduino_LSM9DS1.h>

/*
ACHTUNG: 
Abgewandelte Arduino_LSM9DS1 library nötig
Hier: https://github.com/FemmeVerbeek/Arduino_LSM9DS1/
*/


struct SensorData {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;
};

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }


  //--------------------
  // Calibration values
  //--------------------
  
  // Accelerometer code
  IMU.setAccelFS(3);
  IMU.setAccelODR(5);
  IMU.setAccelOffset(-0.027428, -0.019200, 0.006569);
  IMU.setAccelSlope (1.003544, 1.000383, 1.006805);


  // Gyroscope code
  IMU.setGyroFS(2);
  IMU.setGyroODR(5);
  IMU.setGyroOffset (2.990136, 2.151541, -1.156965);
  IMU.setGyroSlope (1.169691, 1.149312, 1.147530);


  // fixed on robot
  IMU.setMagnetFS(0);
  IMU.setMagnetODR(8);
  IMU.setMagnetOffset(-10.740356, 23.458862, 16.708984);
  IMU.setMagnetSlope (1.302702, 1.314390, 1.313250);

}

void loop() {
  float x, y, z;

  SensorData data;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    data.ax = x;
    data.ay = y;
    data.az = z;
  }
    
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    data.gx = x;
    data.gy = y;
    data.gz = z;
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    data.mx = x;
    data.my = y;
    data.mz = z;
  }
  Serial.write((uint8_t*)&data, sizeof(data)); //put struct data onto serial as bytes
}
