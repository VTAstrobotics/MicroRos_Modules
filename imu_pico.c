#include <stdio.h>


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <hardware/i2c.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joy.h>

#include <rmw_microros/rmw_microros.h>


#include "pico/stdlib.h"
#include "pico_uart_transport.h"


#define I2C_SDA_PIN 11 //16
#define I2C_SCL_PIN 13 //17
#define I2C_PORT i2c0


#define MPU6050_ADDRESS           0x68
#define MPU6050_REG_POWER_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
static int addr = 0x68;


static rcl_publisher_t imu_publisher;
static sensor_msgs__msg__Imu imu_msg;
static rcl_timer_t timer;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rclc_executor_t executor;

float eTime, cTime, pTime;

typedef struct{
    double w;
    double x;
    double y;
    double z;
}Quaterniond;

Quaterniond toQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    //Degree to radius:
    yaw = yaw * M_PI / 180;
    pitch = pitch * M_PI / 180;
    roll = roll * M_PI / 180;


    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaterniond q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(i2c_default, addr, buf, 2, false); 
    sleep_ms(10); // Allow stabilization after waking up
}

static void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    uint8_t timeout = 15000;
    uint8_t buf[2];
    buf[0] = reg_addr;
    buf[1] = data;
    i2c_write_timeout_us(I2C_PORT, dev_addr, buf, 2, false, timeout);
}


static void i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    uint8_t timeout = 15000;
    i2c_write_timeout_us(I2C_PORT, dev_addr, &reg_addr, 1, true, timeout);
    i2c_read_timeout_us(I2C_PORT, dev_addr, data, len, false, timeout);
}


static void imu_setup(void)
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(i2c_default, addr, buf, 2, false); 
    sleep_ms(10); // Allow stabilization after waking up

}

static void imu_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
   
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

static void imu_fill_message(sensor_msgs__msg__Imu *msg)
{

    int16_t acceleration[3], gyro[3], temp;

    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    imu_read_raw(acceleration, gyro, &temp);

    float ax = acceleration[0] / 16384.0f * 9.80665f; // from g to m/s^2
    float ay = acceleration[1] / 16384.0f * 9.80665f;
    float az = acceleration[2] / 16384.0f * 9.80665f;

    float gx = (gyro[0] / 131.0f) * (3.14159265359f / 180.0f);
    float gy = (gyro[1] / 131.0f) * (3.14159265359f / 180.0f);
    float gz = (gyro[2] / 131.0f) * (3.14159265359f / 180.0f);


    //TODO

    msg->angular_velocity.x = gx;
    msg->angular_velocity.y = gy;
    msg->angular_velocity.z = gz;

    msg->linear_acceleration.x = ax;
    msg->linear_acceleration.y = ay;
    msg->linear_acceleration.z = az;
    //TODO

    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    static uint32_t previous_time = 0; //static so doesn't get reassigned every time
    if (previous_time == 0) {
        previous_time = current_time;
    }

    uint32_t dt_ms = current_time - previous_time;
    previous_time = current_time;
    float dt = dt_ms / 1000.0f;

    float accAngleX = (atan(ay / sqrt(ax * ax + az * az)) * 180.0f / 3.14159265359f) - 0.58f;
    float accAngleY = (atan(-ax / sqrt(ay * ay + az * az)) * 180.0f / 3.14159265359f) + 1.58f;

    float gx_deg = gx * 180.0f / 3.14159265359f;
    float gy_deg = gy * 180.0f / 3.14159265359f;
    float gz_deg = gz * 180.0f / 3.14159265359f;

    gx_deg += 0.02f;
    gy_deg += 3.898f;
    gz_deg += 0.25f;

    static float gyroAngleX = 0.0f;
    static float gyroAngleY = 0.0f;
    static float yaw_angle   = 0.0f;
    gyroAngleX += gx_deg * dt;
    gyroAngleY += gy_deg * dt;
    yaw_angle   += gz_deg * dt;

    float roll  = 0.96f * gyroAngleX + 0.04f * accAngleX;
    float pitch = 0.96f * gyroAngleY + 0.04f * accAngleY;

    Quaterniond q;
    q = toQuaternion(yaw_angle, pitch, roll);

    
    msg->orientation.x = q.x;
    msg->orientation.y = q.y;
    msg->orientation.z = q.z;
    msg->orientation.w = q.w;

}

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL) {
        return;
    }

    imu_fill_message(&imu_msg);

    rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (ret != RCL_RET_OK) {
        return;
    }
}

int main()
{
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );


    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool


    mpu6050_reset(); 

    const int timeout_ms = 1000;  // 1 second per attempt
    const uint8_t attempts = 120; // up to 120 seconds
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
        return -1;
    }

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "imu_node", "", &support);

    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    );

    const unsigned int timer_timeout = 50; // ms
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    memset(&imu_msg, 0, sizeof(sensor_msgs__msg__Imu));

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    rcl_publisher_fini(&imu_publisher, &node);
    rcl_node_fini(&node);

    return 0;
}
