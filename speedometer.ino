#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"

MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;
int ax_p, ay_p, az_p;
float const_2g = 16384;
float const_16g = 2048;
float const_250 = 131;
float const_2000 = 16.4;
int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.setFullScaleAccelRange(0x03);
  accelgyro.setFullScaleGyroRange(0x03);

  Serial.print("getFullScaleAccelRange() ");
  Serial.println(accelgyro.getFullScaleAccelRange());
  Serial.print("getFullScaleGyroRange() ");
  Serial.println(accelgyro.getFullScaleGyroRange());
  Serial.print("Accel Offset X ");
  Serial.println(accelgyro.getXAccelOffset());
  Serial.print("Accel Offset Y ");
  Serial.println(accelgyro.getYAccelOffset());
  Serial.print("Accel Offset Z ");
  Serial.println(accelgyro.getZAccelOffset());
  Serial.print("Gyro Offset X ");
  Serial.println(accelgyro.getXGyroOffset());
  Serial.print("Gyro Offset Y ");
  Serial.println(accelgyro.getYGyroOffset());
  Serial.print("Gyro Offset Z ");
  Serial.println(accelgyro.getZGyroOffset());

  calibrate_sensors();
}

void loop() {
 accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("accel: ");
  Serial.print((ax - ax_offset)/const_16g);
  Serial.print(",");
  Serial.print((ay - ay_offset)/const_16g);
  Serial.print(",");
  Serial.print(az/const_16g);
  Serial.print("  gyro: ");
  Serial.print((gx-gx_offset)/const_2000);
  Serial.print(",");
  Serial.print((gy-gy_offset)/const_2000);
  Serial.print(",");
  Serial.println((gz-gz_offset)/const_2000);

  delay(100);
}

void calibrate_sensors() {
  int                   num_readings = 1000;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;

  Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print(i);
    Serial.print("-CALIBRATION: ");
    Serial.print(ax/const_16g);
    Serial.print(",");
    Serial.print(ay/const_16g);
    Serial.print(",");
    Serial.print(az/const_16g);
    Serial.print(",");
    Serial.print(gx/const_2000);
    Serial.print(",");
    Serial.print(gy/const_2000);
    Serial.print(",");
    Serial.println(gz/const_2000);
    x_accel += ax;
    y_accel += ay;
    z_accel += az;
    x_gyro += gx;
    y_gyro += gy;
    z_gyro += gz;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;

  // Store the raw calibration values globally
  ax_offset = x_accel;
  ay_offset = y_accel;
  az_offset = z_accel;
  gx_offset = x_gyro;
  gy_offset = y_gyro;
  gz_offset = z_gyro;

  Serial.print("Offsets: ");
  Serial.print((int16_t)ax_offset);
  Serial.print(", ");
  Serial.print((int16_t)ay_offset);
  Serial.print(", ");
  Serial.print((int16_t)az_offset);
  Serial.print(", ");
  Serial.print((int16_t)gx_offset);
  Serial.print(", ");
  Serial.print((int16_t)gy_offset);
  Serial.print(", ");
  Serial.println((int16_t)gz_offset);

  Serial.println("Finishing Calibration");
}
