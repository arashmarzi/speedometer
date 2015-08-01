#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"

MPU6050 accelgyro;
unsigned long last_read_time;
int16_t ax, ay, az, gx, gy, gz;
float last_velocity_x;
float last_velocity_y;
float last_velocity;
float const_2g = 16384;
float const_16g = 2048;
float const_250 = 131;
float const_2000 = 16.4;
float const_g = 9.81;
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

  /*Serial.print("getFullScaleAccelRange() ");
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
  */
  calibrate_sensors();
  set_last_time(millis());
  set_last_velocity(0);
}

void loop() {
  unsigned long t_now = millis();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_p = ((ax - ax_offset) / const_16g) * const_g;
  float ay_p = ((ay - ay_offset) / const_16g) * const_g;
  float az_p = (az / const_16g) * const_g;

  float gx_p = (gx - gx_offset) / const_2000;
  float gy_p = (gy - gy_offset) / const_2000;
  float gz_p = (gz - gz_offset) / const_2000;

  float dt = get_delta_time(t_now);
  //float accel_xy = get_accel_xy(ax_p, ay_p);
  float new_velocity_x = ((ax_p * dt));
  float new_velocity_y = ((ay_p * dt));
  float new_velocity = sqrt(pow(new_velocity_x,2) + pow(new_velocity_y,2)); 
  
  //get_velocity(accel_xy, dt);
  
  /*Serial.print(dt, DEC);
  Serial.print("   accel: ");
  Serial.print(ax_p);
  Serial.print(",");
  Serial.print(ay_p);
  Serial.print(",");
  Serial.print(az_p);*/
  /*Serial.print("   gyro: ");
  Serial.print(gx_p);
  Serial.print(",");
  Serial.print(gy_p);
  Serial.print(",");
  Serial.println(gz_p);*/
  Serial.print("v_x: ");
  Serial.print(new_velocity_x, 4);
  Serial.print("  ov_x: ");
  Serial.print(last_velocity_x, 4);
  Serial.print("  ax_p*dt: ");
  Serial.print((ax_p * dt), 4);
  Serial.print("  V: ");
  Serial.print(new_velocity, 4);
  Serial.print("  v_y: ");
  Serial.print(new_velocity_y, 4);
  Serial.print("  ov_y: ");
  Serial.print(last_velocity_y, 4);
  Serial.print("  ay_p*dt: ");
  Serial.println((ay_p * dt), 4);
  /*Serial.print("   accel_xy: ");
  Serial.print(sqrt(pow(ax_p, 2) + pow(ay_p, 2)), 4);
  Serial.print("   dt * accel_xy: ");
  Serial.print(dt*sqrt(pow(ax_p, 2) + pow(ay_p, 2)), 4);
  Serial.print("  velocity: " );
  Serial.println(last_velocity + (dt*sqrt(pow(ax_p, 2) + pow(ay_p, 2))), 4);
*/
  set_last_time(t_now);
  set_last_velocity(new_velocity);
  last_velocity_x = new_velocity_x;
  last_velocity_y = new_velocity_y;
  
  delay(100);
}

void calibrate_sensors() {
  int                   num_readings = 50;
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
    Serial.print((ax / const_16g)*const_g);
    Serial.print(",");
    Serial.print((ay / const_16g)*const_g);
    Serial.print(",");
    Serial.print((az / const_16g)*const_g);
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

inline float get_last_velocity() {
  return last_velocity;
}

inline void set_last_velocity(float _velocity) {
  last_velocity = _velocity;
}

inline float get_velocity(float accel_xy, float delta_time) {
  return  get_last_velocity() + (accel_xy * 9.81 * delta_time);
}

inline float get_accel_xy(float ax_p, float ay_p) {
  return sqrt(pow(ax_p, 2) + pow(ay_p, 2));
}

