#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int ax_p, ay_p, az_p;
float const_2g = 16384;
float const_16g = 2048;
int16_t ax_offset, ay_offset, az_offset;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.setFullScaleAccelRange(0x03);

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

  calibrate_sensors();
}

void loop() {
  accelgyro.getAcceleration(&ax, &ay, &az);

  Serial.print("accel: ");
  Serial.print((ax - ax_offset)/const_16g);
  Serial.print(",");
  Serial.print((ay - ay_offset)/const_16g);
  Serial.print(",");
  Serial.println(az/const_16g);

  delay(20);
}

void calibrate_sensors() {
  int                   num_readings = 100;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  /*float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;
  */
  Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  accelgyro.getAcceleration(&ax, &ay, &az);

  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    accelgyro.getAcceleration(&ax, &ay, &az);
    Serial.print("accel_cali: ");
    Serial.print(ax/const_16g);
    Serial.print(",");
    Serial.print(ay/const_16g);
    Serial.print(",");
    Serial.println(az/const_16g);
    x_accel += ax;
    y_accel += ay;
    z_accel += az;
    /*x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;*/
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  /*x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;*/

  // Store the raw calibration values globally
  ax_offset = x_accel;
  ay_offset = y_accel;
  az_offset = z_accel;

  Serial.print("Offsets: ");
  Serial.print((int16_t)ax_offset);
  Serial.print(", ");
  Serial.print((int16_t)ay_offset);
  Serial.print(", ");
  Serial.println((int16_t)az_offset);
  /*base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;*/

  Serial.println("Finishing Calibration");
}
