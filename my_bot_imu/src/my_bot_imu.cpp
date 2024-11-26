#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

// ROS 2 headers
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
// BNO055 sensor headers
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstdint>

using namespace std::chrono_literals;

// I2C device and address
const char *I2C_DEVICE = "/dev/i2c-7";  // Update if using a different bus
#define BNO055_ADDRESS 0x28             // Change to 0x29 if ADR pin is high

// BNO055 Register Addresses
#define BNO055_CHIP_ID_ADDR         0x00
#define BNO055_PAGE_ID_ADDR         0x07
#define BNO055_ACCEL_DATA_X_LSB     0x08
#define BNO055_MAG_DATA_X_LSB       0x0E
#define BNO055_GYRO_DATA_X_LSB      0x14
#define BNO055_EULER_H_LSB          0x1A
#define BNO055_CALIB_STAT_ADDR      0x35
#define BNO055_SYS_TRIGGER_ADDR     0x3F
#define BNO055_PWR_MODE_ADDR        0x3E
#define BNO055_OPR_MODE_ADDR        0x3D

// Operation Modes
#define OPERATION_MODE_CONFIG       0x00
#define OPERATION_MODE_NDOF         0x0C

// Power Modes
#define POWER_MODE_NORMAL           0x00

class my_bot_imu : public rclcpp::Node {
public:
    my_bot_imu();
    ~my_bot_imu();

private:
    void timer_callback();
    bool initializeBNO055(int file);
    bool readAccelerometer(int file, int16_t &x, int16_t &y, int16_t &z);
    bool readGyroscope(int file, int16_t &x, int16_t &y, int16_t &z);
    bool readMagnetometer(int file, int16_t &x, int16_t &y, int16_t &z);
    bool readEulerAngles(int file, int16_t &heading, int16_t &roll, int16_t &pitch);
    bool readCalibrationStatus(int file, uint8_t &sysCalib, uint8_t &gyroCalib, uint8_t &accelCalib, uint8_t &magCalib);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    int i2c_file_;
};

// Sleep function
void sleepMs(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

// Function to write a byte to a register
int writeByte(int file, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(file, buffer, 2) != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("BNO055"), "Failed to write to register 0x%02X", reg);
        return -1;
    }
    return 0;
}

// Function to read a byte from a register
int readByte(int file, uint8_t reg, uint8_t &value) {
    if (write(file, &reg, 1) != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("BNO055"), "Failed to select register 0x%02X", reg);
        return -1;
    }
    if (read(file, &value, 1) != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("BNO055"), "Failed to read from register 0x%02X", reg);
        return -1;
    }
    return 0;
}

// Function to read multiple bytes from a register
int readBytes(int file, uint8_t reg, uint8_t *buffer, int length) {
    if (write(file, &reg, 1) != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("BNO055"), "Failed to select register 0x%02X", reg);
        return -1;
    }
    if (read(file, buffer, length) != length) {
        RCLCPP_ERROR(rclcpp::get_logger("BNO055"), "Failed to read from register 0x%02X", reg);
        return -1;
    }
    return 0;
}

// Sensor initialization function
bool my_bot_imu::initializeBNO055(int file) {
    uint8_t id = 0;
    if (readByte(file, BNO055_CHIP_ID_ADDR, id) < 0) {
        return false;
    }
    if (id != 0xA0) {
        RCLCPP_ERROR(this->get_logger(), "Incorrect chip ID: 0x%02X", id);
        return false;
    }

    // Switch to config mode
    if (writeByte(file, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG) < 0) {
        return false;
    }
    sleepMs(25);

    // Reset the sensor
    if (writeByte(file, BNO055_SYS_TRIGGER_ADDR, 0x20) < 0) {
        return false;
    }
    sleepMs(650);

    // Check chip ID again after reset
    if (readByte(file, BNO055_CHIP_ID_ADDR, id) < 0) {
        return false;
    }
    while (id != 0xA0) {
        sleepMs(10);
        if (readByte(file, BNO055_CHIP_ID_ADDR, id) < 0) {
            return false;
        }
    }
    sleepMs(50);

    // Set power mode to normal
    if (writeByte(file, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL) < 0) {
        return false;
    }
    sleepMs(10);

    // Set page ID to 0
    if (writeByte(file, BNO055_PAGE_ID_ADDR, 0x00) < 0) {
        return false;
    }

    // Set operation mode to NDOF
    if (writeByte(file, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF) < 0) {
        return false;
    }
    sleepMs(20);

    return true;
}

// Function to read accelerometer data
bool my_bot_imu::readAccelerometer(int file, int16_t &x, int16_t &y, int16_t &z) {
    uint8_t buffer[6];
    if (readBytes(file, BNO055_ACCEL_DATA_X_LSB, buffer, 6) < 0) {
        return false;
    }
    x = (int16_t)((buffer[1] << 8) | buffer[0]);
    y = (int16_t)((buffer[3] << 8) | buffer[2]);
    z = (int16_t)((buffer[5] << 8) | buffer[4]);
    return true;
}

// Function to read gyroscope data
bool my_bot_imu::readGyroscope(int file, int16_t &x, int16_t &y, int16_t &z) {
    uint8_t buffer[6];
    if (readBytes(file, BNO055_GYRO_DATA_X_LSB, buffer, 6) < 0) {
        return false;
    }
    x = (int16_t)((buffer[1] << 8) | buffer[0]);
    y = (int16_t)((buffer[3] << 8) | buffer[2]);
    z = (int16_t)((buffer[5] << 8) | buffer[4]);
    return true;
}

// Function to read magnetometer data
bool my_bot_imu::readMagnetometer(int file, int16_t &x, int16_t &y, int16_t &z) {
    uint8_t buffer[6];
    if (readBytes(file, BNO055_MAG_DATA_X_LSB, buffer, 6) < 0) {
        return false;
    }
    x = (int16_t)((buffer[1] << 8) | buffer[0]);
    y = (int16_t)((buffer[3] << 8) | buffer[2]);
    z = (int16_t)((buffer[5] << 8) | buffer[4]);
    return true;
}

// Function to read Euler angles (heading, roll, pitch)
bool my_bot_imu::readEulerAngles(int file, int16_t &heading, int16_t &roll, int16_t &pitch) {
    uint8_t buffer[6];
    if (readBytes(file, BNO055_EULER_H_LSB, buffer, 6) < 0) {
        return false;
    }
    heading = (int16_t)((buffer[1] << 8) | buffer[0]);
    roll = (int16_t)((buffer[3] << 8) | buffer[2]);
    pitch = (int16_t)((buffer[5] << 8) | buffer[4]);
    return true;
}

// Function to read calibration status
bool my_bot_imu::readCalibrationStatus(int file, uint8_t &sysCalib, uint8_t &gyroCalib, uint8_t &accelCalib, uint8_t &magCalib) {
    uint8_t calibStat = 0;
    if (readByte(file, BNO055_CALIB_STAT_ADDR, calibStat) < 0) {
        return false;
    }
    sysCalib = (calibStat >> 6) & 0x03;
    gyroCalib = (calibStat >> 4) & 0x03;
    accelCalib = (calibStat >> 2) & 0x03;
    magCalib = calibStat & 0x03;
    return true;
}

my_bot_imu::my_bot_imu()
    : Node("my_bot_imu") {

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Open the I2C bus
    i2c_file_ = open(I2C_DEVICE, O_RDWR);
    if (i2c_file_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus.");
        rclcpp::shutdown();
        return;
    }

    // Specify the address of the BNO055
    if (ioctl(i2c_file_, I2C_SLAVE, BNO055_ADDRESS) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave.");
        close(i2c_file_);
        rclcpp::shutdown();
        return;
    }

    // Initialize the BNO055 sensor
    if (!initializeBNO055(i2c_file_)) {
        RCLCPP_ERROR(this->get_logger(), "Sensor initialization failed.");
        close(i2c_file_);
        rclcpp::shutdown();
        return;
    }

    // Create publishers
    // Create publishers with Reliable QoS
    rclcpp::QoS qos_settings(10);  // History depth of 10
    qos_settings.reliable();       // Set reliability to Reliable
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos_settings);
    // mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag/data", qos_settings);

    // Create timer
    timer_ = this->create_wall_timer(
        100ms, std::bind(&my_bot_imu::timer_callback, this));
}

my_bot_imu::~my_bot_imu() {
    // Close the I2C bus
    if (i2c_file_ >= 0) {
        close(i2c_file_);
    }
}

void my_bot_imu::timer_callback() {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t heading, roll, pitch;
    uint8_t sysCalib, gyroCalib, accelCalib, magCalib;

    // Read calibration status
    if (!readCalibrationStatus(i2c_file_, sysCalib, gyroCalib, accelCalib, magCalib)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read calibration status.");
        return;
    }

    // Read accelerometer data
    if (!readAccelerometer(i2c_file_, accel_x, accel_y, accel_z)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read accelerometer data.");
        return;
    }

    // Read gyroscope data
    if (!readGyroscope(i2c_file_, gyro_x, gyro_y, gyro_z)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read gyroscope data.");
        return;
    }

    // Read Euler angles
    if (!readEulerAngles(i2c_file_, heading, roll, pitch)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read Euler angles.");
        return;
    }

    // Publish IMU data
    auto imu_msg = sensor_msgs::msg::Imu();

    // Set header
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "base_link";

    // Convert raw data to SI units
    double accel_scale = 0.001 * 9.80665;        // 1 LSB = 1 mg, convert to m/s²
    double gyro_scale = (1.0 / 16.0) * (M_PI / 180.0); // 16 LSB per °/s, convert to rad/s

    // Assign accelerometer data
    imu_msg.linear_acceleration.x = accel_x * accel_scale;
    imu_msg.linear_acceleration.y = accel_y * accel_scale;
    imu_msg.linear_acceleration.z = accel_z * accel_scale;

    // Assign gyroscope data
    imu_msg.angular_velocity.x = gyro_x * gyro_scale;
    imu_msg.angular_velocity.y = gyro_y * gyro_scale;
    imu_msg.angular_velocity.z = gyro_z * gyro_scale;

    // Assign orientation (from Euler angles)
    double euler_scale = 1.0 / 16.0; // 16 LSB per degree
    double h = heading * euler_scale * (M_PI / 180.0);
    double r = roll * euler_scale * (M_PI / 180.0);
    double p = -pitch * euler_scale * (M_PI / 180.0);

    // Convert Euler angles to quaternion
    tf2::Quaternion q;
    q.setRPY(r, p, h); // Note the order of arguments: roll, pitch, yaw (heading)
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    
    imu_publisher_->publish(imu_msg);

    // Broadcast the transform
    geometry_msgs::msg::TransformStamped transform;

    // Set header
    transform.header.stamp = this->now();
    transform.header.frame_id = "base_link";  // Parent frame
    transform.child_frame_id = "imu_link";    // Child frame

    // Set translation (adjust based on IMU placement on the robot)
    transform.transform.translation.x = 0.0162;
    transform.transform.translation.y = -0.0122;
    transform.transform.translation.z = -0.1103; // Adjust as necessary
    // Set rotation based on IMU orientation
    transform.transform.rotation = imu_msg.orientation;

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform);

    // Read magnetometer data
    // int16_t magn_x, magn_y, magn_z;
    // if (!readMagnetometer(i2c_file_, magn_x, magn_y, magn_z)) {
    //     RCLCPP_WARN(this->get_logger(), "Failed to read magnetometer data.");
    //     return;
    // }

    // auto mag_msg = sensor_msgs::msg::MagneticField();

    // // Set header
    // mag_msg.header.stamp = this->get_clock()->now();
    // mag_msg.header.frame_id = "mag_link";

    // // Assign magnetometer data
    // double magn_scale = 1.0 / 16.0 * 1e-6; // 16 LSB per µT, convert to Tesla
    // mag_msg.magnetic_field.x = magn_x * magn_scale;
    // mag_msg.magnetic_field.y = magn_y * magn_scale;
    // mag_msg.magnetic_field.z = magn_z * magn_scale;

    // // Publish the MagneticField message
    // mag_publisher_->publish(mag_msg);

    
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<my_bot_imu>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
