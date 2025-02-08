#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transport.h"

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13
#define I2C_PORT i2c0

#define MPU6050_ADDRESS           0x68
#define MPU6050_REG_POWER_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43

static rcl_publisher_t imu_publisher;
static sensor_msgs__msg__Imu imu_msg;
static rcl_timer_t timer;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rclc_executor_t executor;

/**
 * @brief Initialize I2C for MPU6050
 */
static bool i2c_init_imu(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    sleep_ms(100);
    return true;
}

/**
 * @brief Write a byte to MPU6050 via I2C
 */
static void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t buf[2] = {reg_addr, data};
    i2c_write_blocking(I2C_PORT, dev_addr, buf, 2, false);
}

/**
 * @brief Read bytes from MPU6050 via I2C
 */
static bool i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (i2c_write_blocking(I2C_PORT, dev_addr, &reg_addr, 1, true) < 0) {
        return false;
    }
    return i2c_read_blocking(I2C_PORT, dev_addr, data, len, false) >= 0;
}

/**
 * @brief Wake up MPU6050 and set default configurations
 */
static bool imu_setup(void) {
    i2c_write_byte(MPU6050_ADDRESS, MPU6050_REG_POWER_MGMT_1, 0x00);
    sleep_ms(100);
    return true;
}

/**
 * @brief Read raw accelerometer & gyroscope values from MPU6050
 */
static bool imu_read_raw(int16_t* accel_x, int16_t* accel_y, int16_t* accel_z,
                         int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z) {
    uint8_t accel_buf[6], gyro_buf[6];

    if (!i2c_read_bytes(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, accel_buf, 6)) {
        return false;
    }
    if (!i2c_read_bytes(MPU6050_ADDRESS, MPU6050_REG_GYRO_XOUT_H, gyro_buf, 6)) {
        return false;
    }

    *accel_x = (accel_buf[0] << 8) | accel_buf[1];
    *accel_y = (accel_buf[2] << 8) | accel_buf[3];
    *accel_z = (accel_buf[4] << 8) | accel_buf[5];

    *gyro_x = (gyro_buf[0] << 8) | gyro_buf[1];
    *gyro_y = (gyro_buf[2] << 8) | gyro_buf[3];
    *gyro_z = (gyro_buf[4] << 8) | gyro_buf[5];

    return true;
}

/**
 * @brief Fill IMU ROS message with processed data
 */
static void imu_fill_message(sensor_msgs__msg__Imu *msg) {
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;

    if (!imu_read_raw(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw)) {
        printf("Error reading IMU data!\n");
        return;
    }

    float ax = ax_raw / 16384.0f * 9.80665f;
    float ay = ay_raw / 16384.0f * 9.80665f;
    float az = ax_raw / 16384.0f * 9.80665f;

    float gx = (gx_raw / 131.0f) * (M_PI / 180.0f);
    float gy = (gy_raw / 131.0f) * (M_PI / 180.0f);
    float gz = (gz_raw / 131.0f) * (M_PI / 180.0f);

    msg->orientation.x = 0.0;
    msg->orientation.y = 0.0;
    msg->orientation.z = 0.0;
    msg->orientation.w = 1.0;

    msg->angular_velocity.x = gx;
    msg->angular_velocity.y = gy;
    msg->angular_velocity.z = gz;

    msg->linear_acceleration.x = ax;
    msg->linear_acceleration.y = ay;
    msg->linear_acceleration.z = az;
}

/**
 * @brief Timer callback to publish IMU data
 */
static void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;

    imu_fill_message(&imu_msg);
    rcl_publish(&imu_publisher, &imu_msg, NULL);
}

int main() {
    rmw_uros_set_custom_transport(true, NULL,
                                  pico_serial_transport_open,
                                  pico_serial_transport_close,
                                  pico_serial_transport_write,
                                  pico_serial_transport_read);
    
    stdio_init_all();
    
    if (!i2c_init_imu() || !imu_setup()) {
        printf("IMU Initialization Failed\n");
        return -1;
    }

    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
        printf("Micro-ROS agent not found!\n");
        return -1;
    }

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "imu_node", "", &support);

    rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    memset(&imu_msg, 0, sizeof(sensor_msgs__msg__Imu));

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        sleep_ms(5);
    }

    rcl_publisher_fini(&imu_publisher, &node);
    rcl_node_fini(&node);
    
    return 0;
}
