#include <Wire.h>

struct MPU6050
{
    const int MPU = 0x68; // MPU6050 I2C address
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
    float roll, pitch, yaw;
    float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
    float currentTime, previousTime;
    void init_gyro()
    {
        Wire2.begin();  
        Wire2.beginTransmission(MPU);
        Wire2.write(0x6B);
        Wire2.write(0x00);
        Wire2.endTransmission(true);
    }

    void update(double dt)
    {
        // === Read acceleromter data === //
        Wire2.beginTransmission(MPU);
        Wire2.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
        Wire2.endTransmission(false);
        Wire2.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
        // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
        AccX = (Wire2.read() << 8 | Wire2.read()) / 16384.0; // X-axis value
        AccY = (Wire2.read() << 8 | Wire2.read()) / 16384.0; // Y-axis value
        AccZ = (Wire2.read() << 8 | Wire2.read()) / 16384.0; // Z-axis value

        // Calculating Roll and Pitch from the accelerometer data
        accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58;      // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
        accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58

        // === Read gyroscope data === //
        Wire2.beginTransmission(MPU);
        Wire2.write(0x43); // Gyro data first register address 0x43
        Wire2.endTransmission(false);
        Wire2.requestFrom(MPU, 6, true);                   // Read 4 registers total, each axis value is stored in 2 registers
        GyroX = (Wire2.read() << 8 | Wire2.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
        GyroY = (Wire2.read() << 8 | Wire2.read()) / 131.0;
        GyroZ = (Wire2.read() << 8 | Wire2.read()) / 131.0;

        // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
        roll = roll + GyroX * dt; // deg/s * s = deg
        pitch = pitch + GyroY * dt;
        yaw = yaw + GyroZ * dt;

        // roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
        // pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    }
};