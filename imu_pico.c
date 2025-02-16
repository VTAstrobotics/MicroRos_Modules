#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <hardware/i2c.h>
#include <sensor_msgs/msg/imu.h>

#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transport.h"

// I2C Configuration
#define I2C_SDA_PIN 11
#define I2C_SCL_PIN 13
#define I2C_PORT i2c0
#define MPU6050_ADDRESS 0x68

static int addr = MPU6050_ADDRESS;

// ROS 2 Variables
static rcl_publisher_t imu_publisher;
static sensor_msgs__msg__Imu imu_msg;
static rcl_timer_t timer;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rclc_executor_t executor;

// Kalman filter variables
static float roll = 0.0, pitch = 0.0, yaw = 0.0;  // State estimates
static float P_roll = 1, P_pitch = 1;  // Uncertainty
static float Q_angle = 0.001, Q_gyro = 0.003;  // Process noise
static float R_measure = 0.03;  // Measurement noise

// Kalman Filter Function
static float kalman_filter(float angle, float gyro_rate, float accel_angle, float *P)
{
    float dt = 0.05;  // Time step (50ms)
    
    // Prediction Step
    angle += gyro_rate * dt;
    *P += Q_angle;

    // Update Step
    float K = *P / (*P + R_measure);
    angle += K * (accel_angle - angle);
    *P *= (1 - K);

    return angle;
}

// I2C Write Function
static void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[2] = {reg_addr, data};
    i2c_write_blocking(I2C_PORT, dev_addr, buf, 2, false);
}

// I2C Read Function
static void i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_write_blocking(I2C_PORT, dev_addr, &reg_addr, 1, true);
    i2c_read_blocking(I2C_PORT, dev_addr, data, len, false);
}

// MPU6050 Reset & Initialization
static void mpu6050_reset()
{
    uint8_t buf[] = {0x6B, 0x80};  // Reset command
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
    sleep_ms(100);

    buf[1] = 0x00;  // Wake up the device
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
    sleep_ms(10);
}

// Read raw IMU data
static void imu_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[6];

    // Read accelerometer data
    uint8_t reg = 0x3B;
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Read gyro data
    reg = 0x43;
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    // Read temperature
    reg = 0x41;
    i2c_write_blocking(i2c_default, addr, &reg, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

// IMU Data Processing & Kalman Filter Application
static void imu_fill_message(sensor_msgs__msg__Imu *msg)
{
    int16_t acceleration[3], gyro[3], temp;
    imu_read_raw(acceleration, gyro, &temp);

    // Convert to meaningful units
    float ax = acceleration[0] / 16384.0f * 9.80665f;
    float ay = acceleration[1] / 16384.0f * 9.80665f;
    float az = acceleration[2] / 16384.0f * 9.80665f;

    float gx = (gyro[0] / 131.0f) * (M_PI / 180.0f);
    float gy = (gyro[1] / 131.0f) * (M_PI / 180.0f);
    float gz = (gyro[2] / 131.0f) * (M_PI / 180.0f);

    // Compute roll & pitch from accelerometer
    float accel_roll = atan2f(ay, az) * (180.0f / M_PI);
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);

    // Apply Kalman Filter
    roll = kalman_filter(roll, gx * (180.0f / M_PI), accel_roll, &P_roll);
    pitch = kalman_filter(pitch, gy * (180.0f / M_PI), accel_pitch, &P_pitch);
    yaw += gz * 0.05 * (180.0f / M_PI);

    // Convert to quaternion
    float cy = cos(yaw * 0.5 * M_PI / 180.0f);
    float sy = sin(yaw * 0.5 * M_PI / 180.0f);
    float cp = cos(pitch * 0.5 * M_PI / 180.0f);
    float sp = sin(pitch * 0.5 * M_PI / 180.0f);
    float cr = cos(roll * 0.5 * M_PI / 180.0f);
    float sr = sin(roll * 0.5 * M_PI / 180.0f);

    msg->orientation.w = cr * cp * cy + sr * sp * sy;
    msg->orientation.x = sr * cp * cy - cr * sp * sy;
    msg->orientation.y = cr * sp * cy + sr * cp * sy;
    msg->orientation.z = cr * cp * sy - sr * sp * cy;

    msg->angular_velocity.x = gx;
    msg->angular_velocity.y = gy;
    msg->angular_velocity.z = gz;

    msg->linear_acceleration.x = ax;
    msg->linear_acceleration.y = ay;
    msg->linear_acceleration.z = az;
}

// Timer Callback
static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL) return;
    imu_fill_message(&imu_msg);
    rcl_publish(&imu_publisher, &imu_msg, NULL);
}

int main()
{
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);

    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    mpu6050_reset(); 

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "imu_node", "", &support);
    rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    
    while (true) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
