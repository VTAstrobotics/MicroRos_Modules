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

/*
This file is an attempted conversion of a file that reads a hx711 force sensor using pico into one
that reads a fx29 sensor using pico. Also included is micro-ros publisher functions.

This HAS NOT been tested and there is a good chance this does not work yet.
*/

// Ports and pins
#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5
#define FX29_I2C_ADDR 0x28  // Default FX29 I2C address
#define FX29_MAX_COUNTS 15000.0f  // Maximum digital counts from datasheet
#define FX29_MAX_LBF 200.0f  // Maximum force in pounds (adjust based on sensor range)
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

// Registers
static const uint8_t REG_DEVID = 0x00;
static const uint8_t REG_POWER_CTL = 0x2D;
static const uint8_t REG_DATAX0 = 0x32;


void fx29_init() {

    // Initialization functions
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

int reg_write(i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes);

int reg_read(   i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

int reg_write(  i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(  i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}

int fx29_read_force_raw() {

    // Address reading
    uint8_t data[6];
    return reg_read(I2C_PORT, FX29_I2C_ADDR, REG_DEVID, data, 1);
}

float fx29_convert_to_lbf(int raw_force) {

    // Convertion based on data sheet values
    // https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=FX29&DocType=Data%20Sheet&DocLang=English&DocFormat=pdf&PartCntxt=20009605-23
    //return (raw_force / FX29_MAX_COUNTS) * FX29_MAX_LBF;
    return raw_force;

}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    int raw_force = fx29_read_force_raw();
 
    msg.data = fx29_convert_to_lbf(raw_force);
    rcl_publish(&publisher, &msg, NULL);

}

int main() {

    stdio_init_all();
    fx29_init();
 
    rmw_uros_set_custom_transport(
        true, NULL, pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    if (rmw_uros_ping_agent(timeout_ms, attempts) != RCL_RET_OK) {
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
 
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(500), timer_callback);
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
 
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    return 0;

}
