#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transport.h"
#include "pico/stdlib.h"

//second file stuff below

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joy.h>
#include <math.h>


#define IMU_SDA_PIN 4 //16
#define IMU_SCL_PIN 5 //17
#define IMU_PORT i2c0
#define timeout 10000


#define MPU6050_ADDRESS           0x68
#define MPU6050_REG_POWER_MGMT_1  0x6B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
static int addr = 0x68;


static rcl_publisher_t publisher;
static sensor_msgs__msg__Imu imu_msg;
static rcl_timer_t timer;
static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rclc_executor_t executor;

float eTime, cTime, pTime;

/*
This file is an attempted conversion of a file that reads a hx711 force sensor using pico into one
that reads a fx29 sensor using pico. Also included is micro-ros publisher functions.

This HAS NOT been tested and there is a good chance this does not work yet.

This file has also included things for the pico. It is an attempted merge file. 
*/
static void read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

struct fx_29_sensor{
    int I2C_Address;
};
typedef struct fx_29_sensor FX29;

// Ports and pins
#define FX_29_I2C_PORT i2c1
#define FX_29_SDA_PIN 2
#define FX_29_SCL_PIN 3
#define FX29_I2C_ADDR 0x28  // Default FX29 I2C address
#define FX29_MAX_COUNTS 15000.0f  // Maximum digital counts from datasheet
#define FX29_MAX_LBF 200.0f  // Maximum force in pounds (adjust based on sensor range)
rcl_publisher_t imu_publisher;
std_msgs__msg__Float32 msg;
rcl_timer_t imu_timer;

// Registers
static const uint8_t REG_DEVID = 0x00;
static const uint8_t REG_POWER_CTL = 0x2D;
static const uint8_t REG_DATAX0 = 0x32;

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


//making non static for compiling
void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_timeout_us(i2c_default, addr, buf, 2, false, timeout);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_timeout_us(i2c_default, addr, buf, 2, false, timeout); 
    sleep_ms(10); // Allow stabilization after waking up
}

//getting rid of the static declaration
void fill_message(sensor_msgs__msg__Imu *msg)
{

    int16_t acceleration[3], gyro[3], temp;

    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    read_raw(acceleration, gyro, &temp);

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
    uint32_t sec = current_time / 1000;
    uint32_t nanosec = (current_time % 1000) * 1000000;
    msg->header.stamp.sec = sec;
    msg->header.stamp.nanosec = nanosec;
    msg->header.stamp.sec = current_time/1000.0;
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

void fx29_init() {

    i2c_init(i2c1, 1000 * 100);
    gpio_set_function(FX_29_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(FX_29_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(FX_29_SDA_PIN);
    gpio_pull_up(FX_29_SCL_PIN);
    sleep_ms(1000); // Allow device to reset and stabilize
    i2c_write_timeout_us(i2c1, FX29_I2C_ADDR, 0x55, 1, true, 100000 );
}


int fx29_read_force_raw() {

    // Address reading
    uint8_t buf[2];
    i2c_write_timeout_us(i2c1, FX29_I2C_ADDR, 0x04, 1, true, 100000 );
    buf[0] = 0;
    buf[1] = 0;
    int result = i2c_read_timeout_us(i2c1, FX29_I2C_ADDR, buf, 2, false, 100000);
    int force;
    if(result<0){
        force = -2222;
    }
    else{
    force =  ((buf[0] & 0x3F ) << 8) | (buf[1] << 0 );
    }
    return force;
}

float fx29_convert_to_lbf(int raw_force) {

    // Convertion based on data sheet values
    // https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=FX29&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=20009605-23
    return raw_force;
    // return (raw_force / FX29_MAX_COUNTS) * FX29_MAX_LBF;

}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    int raw_force = fx29_read_force_raw();
 
    msg.data = fx29_convert_to_lbf(raw_force);
    rcl_publish(&publisher, &msg, NULL);

}

//relocation from bottom to above main
static void timer_callback_imu(rcl_timer_t *timer, int64_t last_call_time)
{

    fill_message(&imu_msg);

    rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (ret != RCL_RET_OK) {
        return;
    }
}

int main() {

    stdio_init_all();
 
    rmw_uros_set_custom_transport(
        true, NULL, pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    const int timeout_ms1 = 1000; 
    const uint8_t attempts2 = 120;
    if (rmw_uros_ping_agent(timeout_ms1, attempts2) != RCL_RET_OK) {
        printf("Micro-ROS agent not found\n");
        return -1;
    }

    // Initialize Microros components, attempting to pattern match the example
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "fx29_force"
    );
 
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(3000), timer_callback);
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    //second file, IMU things in main


    i2c_init(IMU_PORT, 400 * 1000);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
    fx29_init();
    // Make the I2C pins available to picotool


    mpu6050_reset(); 

    const int timeout_ms2 = 1000;  // 1 second per attempt
    const uint8_t attempts1 = 120; // up to 120 seconds
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms2, attempts1);
    if (ret != RCL_RET_OK) {
        return -1;
    }


    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    );

    const unsigned int timer_timeout = 10; // ms
    rclc_timer_init_default(&imu_timer,&support,RCL_MS_TO_NS(timer_timeout),timer_callback_imu);
    rclc_executor_add_timer(&executor, &imu_timer);

    memset(&imu_msg, 0, sizeof(sensor_msgs__msg__Imu));

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);

    return 0;
}

/**
 * second file, IMU things
 */

// static void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
// {
//     uint8_t buf[2];
//     buf[0] = reg_addr;
//     buf[1] = data;
//     i2c_write_timeout_us(I2C_PORT, dev_addr, buf, 2, false, timeout);
// }


// static void i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     i2c_write_timeout_us(I2C_PORT, dev_addr, &reg_addr, 1, true, timeout);
//     i2c_read_timeout_us(I2C_PORT, dev_addr, data, len, false, timeout);
// }


static void setup(void)
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_timeout_us(i2c_default, addr, buf, 2, false, timeout);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_timeout_us(i2c_default, addr, buf, 2, false, timeout); 
    sleep_ms(10); // Allow stabilization after waking up

}

static void read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
   
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];
    buffer[0] = 0;
    buffer[1] = 0;
    buffer[2] = 0;
    buffer[3] = 0;
    buffer[4] = 0;
    buffer[5] = 0;

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_timeout_us(i2c_default, addr, &val, 1, true, timeout); // true to keep master control of bus
    i2c_read_timeout_us(i2c_default, addr, buffer, 6, false, timeout);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_timeout_us(i2c_default, addr, &val, 1, true, timeout);
    i2c_read_timeout_us(i2c_default, addr, buffer, 6, false, timeout);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_timeout_us(i2c_default, addr, &val, 1, true, timeout);
    i2c_read_timeout_us(i2c_default, addr, buffer, 2, false, timeout);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

//relocating above main
// static void timer_callback_imu(rcl_timer_t *timer, int64_t last_call_time)
// {
//     if (timer == NULL) {
//         return;
//     }

//     fill_message(&msg);

//     rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
//     if (ret != RCL_RET_OK) {
//         return;
//     }



 
