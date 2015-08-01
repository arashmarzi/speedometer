#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"

const float const_16g = 2048;
const float const_2000 = 16.4;
const float const_g = 9.81;

MPU6050 accelgyro;
unsigned long last_read_time;
int16_t ax, ay, az, gx, gy, gz;
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
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_p = (ax - ax_offset) / const_16g;
  float ay_p = (ay - ay_offset) / const_16g;
  float az_p = (az / const_16g);

  float gx_p = (gx - gx_offset) / const_2000;
  float gy_p = (gy - gy_offset) / const_2000;
  float gz_p = (gz - gz_offset) / const_2000;

  float dt = get_delta_time(t_now);
  float vel_x = (ax_p * dt * const_g);
  float vel_y = (ay_p * dt * const_g);
  float vel = sqrt(pow(vel_x, 2) + pow(vel_y, 2));

  temperature = (accelgyro.getTemperature() + 12412)/340;

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
Serial.print("Temp: ");
Serial.println(temperature);
  set_last_time(t_now);
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
    Serial.print((ax / const_16g));
    Serial.print(",");
    Serial.print((ay / const_16g));
    Serial.print(",");
    Serial.print((az / const_16g));
    Serial.print(",");
    Serial.print(gx / const_2000);
    Serial.print(",");
    Serial.print(gy / const_2000);
    Serial.print(",");
    Serial.println(gz / const_2000);
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

/*inline float get_last_velocity() {
  return last_velocity;
}

inline void set_last_velocity(float _velocity) {
  last_velocity = _velocity;
}

inline float get_velocity(float accel_xy, float delta_time) {
  return  get_last_velocity() + (accel_xy * 9.81 * delta_time);
}
*/
inline float get_accel_xy(float ax_p, float ay_p) {
  return sqrt(pow(ax_p, 2) + pow(ay_p, 2));
}

