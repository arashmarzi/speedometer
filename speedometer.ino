#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"

const float CONST_16G = 2048;
const float CONST_2000 = 16.4;
const float CONST_G = 9.81;
const float RADIANS_TO_DEGREES = 180 / 3.14159;
const float ALPHA = 0.96;
MPU6050 accelgyro;
unsigned long last_read_time;
int16_t ax, ay, az, gx, gy, gz;
int16_t gyro_angle_x_l, gyro_angle_y_l;
int16_t angle_x_l, angle_y_l;
int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int16_t temperature;
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

  calibrate_sensors();
  set_last_time(millis());
}

void loop() {
  unsigned long t_now = millis();
  float dt = get_delta_time(t_now);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_p = (ax - ax_offset) / CONST_16G;
  float ay_p = (ay - ay_offset) / CONST_16G;
  float az_p = (az / CONST_16G);

  float accel_angle_y = atan(-1 * ax_p / sqrt(pow(ay_p, 2) + pow(az_p, 2))) * RADIANS_TO_DEGREES;
  float accel_angle_x = atan(ay_p / sqrt(pow(ax_p, 2) + pow(az_p, 2))) * RADIANS_TO_DEGREES;

  float gx_p = (gx - gx_offset) / CONST_2000;
  float gy_p = (gy - gy_offset) / CONST_2000;
  float gz_p = (gz - gz_offset) / CONST_2000;

  float gyro_angle_x = gx_p * dt + get_last_angle_x();
  float gyro_angle_y = gy_p * dt + get_last_angle_y();

  float angle_x = ALPHA * gyro_angle_x + (1.0 - ALPHA) * accel_angle_x;
  float angle_y = ALPHA * gyro_angle_y + (1.0 - ALPHA) * accel_angle_y;

  float vel_x = (ax_p * dt * CONST_G);
  float vel_y = (ay_p * dt * CONST_G);
  float vel = sqrt(pow(vel_x, 2) + pow(vel_y, 2));

  temperature = (accelgyro.getTemperature() + 12412) / 340;

  //Serial.print(dt, DEC);
  /*Serial.print("accel: ");
  Serial.print(ax_p);
  Serial.print(",");
  Serial.print(ay_p);
  Serial.print(",");
  Serial.print(az_p);
  Serial.print("\tgyro: ");
  Serial.print(gx_p);
  Serial.print(",");
  Serial.print(gy_p);
  Serial.print(",");
  Serial.println(gz_p);
  Serial.print("\tvel: ");
  Serial.print(vel, 4);
  Serial.print(",");
  Serial.print(vel_x, 4);
  Serial.print(",");
  Serial.println(vel_y, 4);
  */
  Serial.print("angle_x: ");
  Serial.print(angle_x);
  Serial.print("   angle_y: ");
  Serial.print(angle_y);

  Serial.print("   temp: ");
  Serial.println(temperature);
  set_last_time(t_now);

  set_last_gyro_angle_x(gyro_angle_x);
  set_last_gyro_angle_y(gyro_angle_y);

  set_last_angle_x(angle_x);
  set_last_angle_y(angle_y);

  delay(5);
}

void calibrate_sensors() {
  int                   num_readings = 100;
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
    Serial.print((ax / CONST_16G));
    Serial.print(",");
    Serial.print((ay / CONST_16G));
    Serial.print(",");
    Serial.print((az / CONST_16G));
    Serial.print(",");
    Serial.print(gx / CONST_2000);
    Serial.print(",");
    Serial.print(gy / CONST_2000);
    Serial.print(",");
    Serial.println(gz / CONST_2000);
    x_accel += ax;
    y_accel += ay;
    z_accel += az;
    x_gyro += gx;
    y_gyro += gy;
    z_gyro += gz;
    delay(10);
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
  Serial.print(ax_offset);
  Serial.print(", ");
  Serial.print(ay_offset);
  Serial.print(", ");
  Serial.print(az_offset);
  Serial.print(", ");
  Serial.print(gx_offset);
  Serial.print(", ");
  Serial.print(gy_offset);
  Serial.print(", ");
  Serial.println(gz_offset);

  Serial.println("Finishing Calibration");
}

inline unsigned long get_last_time() {
  return last_read_time;
}

inline void set_last_time(unsigned long _time) {
  last_read_time = _time;
}

inline float get_delta_time(unsigned long t_now) {
  return (t_now - get_last_time()) / 1000.0;
}

inline int16_t get_last_gyro_angle_x() {
  return gyro_angle_x_l;
}

inline void set_last_gyro_angle_x(int16_t _gyro_angle_x) {
  gyro_angle_x_l = _gyro_angle_x;
}

inline int16_t get_last_gyro_angle_y() {
  return gyro_angle_y_l;
}

inline void set_last_gyro_angle_y(int16_t _gyro_angle_y) {
  gyro_angle_y_l = _gyro_angle_y;
}

inline int16_t get_last_angle_x() {
  return angle_x_l;
}

inline void set_last_angle_x(int16_t _ang_x) {
  angle_x_l = _ang_x;
}

inline int16_t get_last_angle_y() {
  return angle_y_l;
}

inline void set_last_angle_y(int16_t _ang_y) {
  angle_y_l = _ang_y;
}

inline float get_accel_xy(float ax_p, float ay_p) {
  return sqrt(pow(ax_p, 2) + pow(ay_p, 2));
}

