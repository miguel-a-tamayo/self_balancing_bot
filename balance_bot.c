/*
 * Author: Miguel Tamayo (1658819)
 * Date: 05/27/2023
 * balance_bot.c
 * 
 * File implements sensor fusion using a kalman filter to get an angle estimation of the robot.
 * A PID loop is implemented as the controller for the robot in attempts of balancing it
 * 
 * NOTE: Gyro and accelerometer are sampling at 800Hz. dT = 1/800 = 0.00125S = 1250uS
*/
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "hal/i2c_types.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "freertos/projdefs.h"
#include "esp_timer.h"
#include "freertos/timers.h"
#include "driver/ledc.h"

// right motor pins
#define IN1             GPIO_NUM_1
#define IN2             GPIO_NUM_3
#define ENA             GPIO_NUM_0
#define PWM_CHANNEL0    LEDC_CHANNEL_0

// left motor pins
#define IN3             GPIO_NUM_4
#define IN4             GPIO_NUM_5
#define ENB             GPIO_NUM_6
#define PWM_CHANNEL1    LEDC_CHANNEL_1

#define HIGH    1
#define LOW     0

// PWM parameters
#define PWM_TIMER              LEDC_TIMER_0 // timer used for the pwm signal
#define PWM_MODE               LEDC_LOW_SPEED_MODE
#define PWM_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define PWM_FREQUENCY         (20000) // Frequency in Hertz (10k)

#define M_PI		3.14159265358979323846

#define I2C_MASTER_SCL_IO			8			// SCL pin
#define I2C_MASTER_SDA_IO			10			// SDA pin
#define I2C_MASTER_NUM				I2C_NUM_0	// I2C port number for master
#define I2C_MASTER_TX_BUF_DISABLE	0			// don't need buffer
#define I2C_MASTER_RX_BUF_DISABLE	0			// don't need buffer
#define I2C_MASTER_FREQ_HZ			100000		// I2C master clock frequency

#define ICM42670_ADDRESS    0x68		// I2C address for IMU sensor

// power configuration
#define POWER_REG           0x1F
#define IMU_CONFIG          0b00001111 // turn on accel and gyro to LN mode

// accelerometer config0
#define ACCEL_REG0      0x21
#define ACCEL_CONFIG0   0b01100110 // +-2g scale with 200Hz frequency

// accelerometer config1
#define ACCEL_REG1      0x24
#define ACCEL_CONFIG1   0b00000000 // x2 average filter and bypass LP filter

// gyro config 0
#define GYRO_REG0          0x20
#define GYRO_CONFIG0       0b01100110 // gyro at +-250dps at 400Hz

// gyro config 1
#define GYRO_REG1          0x23
#define GYRO_CONFIG1       0b00000000 // Bypass LP filter

#define SAMPLING_FREQ         50 // 50Hz sampling rate

// registers address of MSB
// LSB are located one register over
#define GYRO_YH     0x13
#define ACCEL_XH    0x0B
#define ACCEL_ZH    0x0F

#define FIFO_REG            0x3F
#define FIFO_DATA_LENGTH    15

typedef enum {
    x_axis,
    z_axis,
} accel_axis_t;

/* tags */
static const char *I2C_TAG = "I2C";
static const char *IMU_TAG = "IMU";
static const char *ACCEL_TAG = "ACCEL";
static const char *GYRO_TAG = "GYRO";
const char *PWM_TAG = "PWM";

static void pwm_init(void) {
    esp_err_t err;

    // create PWM timer
    ledc_timer_config_t pwm_timer = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_DUTY_RES,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&pwm_timer);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to setup pwm timer %d", err);

    // create PWM channels for the motors
    ledc_channel_config_t R_motor_channel = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL0,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ENA,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config_t L_motor_channel = {
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL1,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ENB,
        .duty = 0,
        .hpoint = 0
    };
    err = ledc_channel_config(&R_motor_channel);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to set R motor channel %d", err);

    vTaskDelay(pdMS_TO_TICKS(100)); // delay just in case that the setup needs some time

    err = ledc_channel_config(&L_motor_channel);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to set L motor channel %d", err);
}

void l298n_init(void) {
    // set enable pins for right motor
    esp_rom_gpio_pad_select_gpio(IN1); gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(IN2); gpio_set_direction(IN2, GPIO_MODE_OUTPUT);

    // set enable pins for left motor
    esp_rom_gpio_pad_select_gpio(IN3); gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(IN4); gpio_set_direction(IN4, GPIO_MODE_OUTPUT);
}

/* move motors forward */
void forward(float duty_cycle) {
    esp_err_t err;
    uint32_t duty;

    // get the duty cycle
    duty = (uint32_t)(duty_cycle); // convert the percentage of duty cycle into the raw value

    // set motor direction here
    gpio_set_level(IN1, HIGH);
    gpio_set_level(IN2, LOW);

    // @TODO: set left motor here
    gpio_set_level(IN3, HIGH);
    gpio_set_level(IN4, LOW);

    // set right motor
    err = ledc_set_duty(PWM_MODE, PWM_CHANNEL0, duty);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to set right motor PWM %d", err);
    err = ledc_update_duty(PWM_MODE, PWM_CHANNEL0);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to update right motor PWM %d", err);

    // set the left motor
    err = ledc_set_duty(PWM_MODE, PWM_CHANNEL1, duty);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to set left motor PWM %d", err);
    err = ledc_update_duty(PWM_MODE, PWM_CHANNEL1);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to update left motor PWM %d", err);
}

/* move motors forward */
void backward(float duty_cycle) {
    esp_err_t err;
    uint32_t duty;

    // get the duty cycle
    // duty_cycle = 8191.0 * (duty_percent / 100.0); // using the 13 bit timer
    duty = (uint32_t)duty_cycle;

    // set motor direction here
    gpio_set_level(IN1, LOW);
    gpio_set_level(IN2, HIGH);

    // @TODO: set left motor here
    gpio_set_level(IN3, LOW);
    gpio_set_level(IN4, HIGH);

    // set right motor
    err = ledc_set_duty(PWM_MODE, PWM_CHANNEL0, duty);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to set right motor PWM %d", err);
    err = ledc_update_duty(PWM_MODE, PWM_CHANNEL0);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to update right motor PWM %d", err);

    // set the left motor
    err = ledc_set_duty(PWM_MODE, PWM_CHANNEL1, duty);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to set left motor PWM %d", err);
    err = ledc_update_duty(PWM_MODE, PWM_CHANNEL1);
    if (err != ESP_OK) ESP_LOGE(PWM_TAG, "Unable to update left motor PWM %d", err);
}

/* function to initialize the i2c communication */
static esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(I2C_TAG, "Failed to i2c_param_config %d\n", err);
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(I2C_TAG, "Failed to install i2c driver %d\n", err);
        return err;
    }

    return err;
}

/* initialize the ICM42670 module */
void icm42670_init() {
    ESP_LOGI(IMU_TAG, "Initializing ICM42670 Module");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ICM42670_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, POWER_REG, true);
    i2c_master_write_byte(cmd, IMU_CONFIG, true);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) ESP_LOGE(IMU_TAG, "Unable to initialize icm42670 %d\n", err);

    ESP_LOGI(IMU_TAG, "Successuflly initialized ICM42670 Module");

}

/* Set the settings we want for the gyroscope */
void gyro_init() {
    i2c_cmd_handle_t cmd;
    esp_err_t err;

    ESP_LOGI(I2C_TAG, "Initializing gyroscope");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ICM42670_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, GYRO_REG0, true);
    i2c_master_write_byte(cmd, GYRO_CONFIG0, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) ESP_LOGE(GYRO_TAG, "Unable to set gyro config 0 %d\n", err);

    vTaskDelay(pdMS_TO_TICKS(25));

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ICM42670_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, GYRO_REG1, true);
    i2c_master_write_byte(cmd, GYRO_CONFIG1, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) ESP_LOGE(GYRO_TAG, "Unable to set gyro config1 %d\n", err);

    ESP_LOGI(GYRO_TAG, "Successuflly initialized gyroscope");
}

/* Set the settings we want for the accelerometer */
void accel_init() {
    i2c_cmd_handle_t cmd;
    esp_err_t err;

    ESP_LOGI(I2C_TAG, "Initializing accelerometer");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ICM42670_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ACCEL_REG0, true);
    i2c_master_write_byte(cmd, ACCEL_CONFIG0, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) ESP_LOGE(ACCEL_TAG, "Unable to set accel config0 %d\n", err);

    vTaskDelay(pdMS_TO_TICKS(25));

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ICM42670_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, ACCEL_REG1, true);
    i2c_master_write_byte(cmd, ACCEL_CONFIG1, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) ESP_LOGE(ACCEL_TAG, "Unable to set accel config1 %d\n", err);

    ESP_LOGI(ACCEL_TAG, "Successuflly initialized accelerometer");
}

/* read byte from the IMU */
uint8_t read_byte(uint8_t reg_address) {
    uint8_t data = 0;
    i2c_cmd_handle_t cmd;
    esp_err_t err;

    // write to the IMU the register we want to read from
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ICM42670_ADDRESS << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500));
    if (err != ESP_OK) ESP_LOGE(I2C_TAG, "Unable to write to icm42670 %d\n", err);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(25));

    // read data from the register set above
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ICM42670_ADDRESS << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500));
    if (err != ESP_OK) ESP_LOGE(I2C_TAG, "Unable to read from icm42670 %d\n", err);
    i2c_cmd_link_delete(cmd);

    return data;
}

/* get data from gyroscope */
int16_t read_gyro() {
    uint8_t gyro_h = 0, gyro_l = 0; // MSB and LSB bydtes for acceletomenter
    uint16_t gyro = 0;

    gyro_h = read_byte(GYRO_YH); 
    gyro_l = read_byte(GYRO_YH + 1);
    gyro = ((int16_t)gyro_h << 8) | gyro_l; 

    return gyro;
}

/* get data from accelerometer */
int16_t read_accel(uint8_t axis) {
    uint8_t accel_h = 0, accel_l = 0; // MSB and LSB bydtes for acceletomenter
    uint16_t accel = 0;
    
    if (axis == x_axis) {
        accel_h = read_byte(ACCEL_XH);
        accel_l = read_byte(ACCEL_XH + 1);
        accel = ((int16_t)accel_h << 8) | accel_l; 
    }
    
    if (axis == z_axis) {
        accel_h = read_byte(ACCEL_ZH);
        accel_l = read_byte(ACCEL_ZH + 1);
        accel = ((int16_t)accel_h << 8) | accel_l; 
    }

    return accel;
}

// float get_duty(float command) {

// }

/* ----- GLOBAL VARIABLES -----*/
const float dT = 0.00125; // time step in seconds

/* ----- SENSOR VARIABLES ----- */
// gyroscope
const float gyro_scalar = 1.875;
const float gyro_bias = -0.0436;
int16_t gyro_raw = 0; // raw gyroscope output
float q_meas = 0.0; // gyro rate q long the y-axis in correct units
// static float gyro_angle = 0.0; // gyro angle

// accelerometer
const float accel_xbias = -7.2709e-04;
const float accel_xscalar = 0.9955;
const float accel_zbias = -0.0045;
const float accel_zscalar = 1.0107;
int16_t accel_xraw = 0; // raw accelerometer x output
int16_t accel_zraw = 0; // raw accelerometer y output
float accel_x = 0.0; // converted x acceleration
float accel_z = 0.0; // converted z accelerations
static float  accel_angle = 0.0;

// Kalman Filter
float theta = 0.0; // intialize angle estimate
float theta_prev = 0.0;
float b_dot = 0.0; // initialize bias from accelerometer
float b_hat = 0.0;
float q_hat = 0.0;
float error = 0; // error
const float Kp = 170; // proportional gain
const float Ki = Kp/20; // integral gain

/* ----- CONTROLLER VARIABLES ----- */
float motor_bias = 155.0; // this is the speed at which the motor break stall speed

float ref = 0.0; // because why not
float e = 0.0; // error from zero
float e_prev = 0.0; // previous error for derivative
float u = 0.0; // command
float u_prev = 0.0; // previous command
float up = 0.0; // proportional term
float ui = 0.0; // integral term of the controller
float ud = 0.0; // derivative term of the controller
float umax = 200; // maximum pwm signal we can send the motor
float A = 0.0; // accumulator for the integral
float A_prev = 0.0; // previous accumulator

// controller gains
const float cKp = 2.5;
const float cKd = 0.00;
const float cKi = 0;

static void bot_balance(void* arg) {
    // curr_time = esp_timer_get_time(); // gets the time in micro seconds

    // if ((curr_time - prev_time) >= 1250) { // 0.00125 seconds have passed in micro
        // prev_time = curr_time; // update time

        /* ----- READ CURRENT ANGLE -----*/
        // process the gyroscope
        gyro_raw = read_gyro(); // read the raw output from gyroscope
        q_meas = ((float)gyro_raw / 131.0) * gyro_scalar + gyro_bias; // convert to deg/sec and calibrate
        // gyro_angle = gyro_angle + q_meas*((float)dT / 1000.0);

        // process the accelerometer
        accel_xraw = read_accel(x_axis); // read raw output
        accel_zraw = read_accel(z_axis);
        accel_x = ((float)(accel_xraw) / 16384.0 ) * accel_xscalar + accel_xbias;
        accel_z = ((float)(accel_zraw) / 16384.0 ) * accel_zscalar + accel_zbias;
        accel_angle = -atan2f(accel_x, accel_z) * (180.0 / M_PI);

        error = accel_angle - theta; // measure the error
        b_dot = -Ki * error; // get bias
        b_hat = b_hat + b_dot * (dT); // integrate bias
        q_hat = q_meas - b_hat + (Kp * error); // calculate rate
        theta = theta + (q_hat * (dT)); // integrate to get angle

        /* ----- PID LOOP ----- */
        e = ref - round(theta); // calculate the error
        up = cKp * e; // calculate proportianal term
        ud = (cKd/dT) * (theta - theta_prev); // calculate proportional term
        A = A + e*dT; // calculate accumulation term
        ui = cKi * A;

        u = up + ud + ui; // calculate command should be between 0 and 100 (function takes percent)
        theta_prev = theta;

        // add the motor bias
        if (u < 0) {
            u = u - motor_bias;
        } 
        else if (u > 0){
            u = u + motor_bias;
        }

        // control the accumulator
        if (u > umax) {
            A = A - e*dT; // remove accumulator
            u = umax; // set command to its maximum
        } 
        else if (u < -umax) {
            A = A - e*dT;
            u = -umax;
        } else {
            A_prev = A; // update accumulator
            u_prev = u; // update previous command
        }
        
        // get direction and send signal
        if (u < 0) {
            backward(fabs(u));
        } 
        else if (u > 0){
            forward(fabs(u));

        } else {
            gpio_set_level(IN1, LOW);
            gpio_set_level(IN2, LOW);

            // @TODO: set left motor here
            gpio_set_level(IN3, LOW);
            gpio_set_level(IN4, LOW);
        }

        /* ----- PRINTING -----*/
        // printf("%0.0f | %0.0f\n", theta, u);
    // }
    // ESP_LOGI(IMU_TAG, "time passed: %llu\n", curr_time - prev_time);
}

float curr_rate = 0.0;
float prev_rate = 0.0;

void app_main() {
    uint64_t clock_period = 5000; // every 10uS

    // initialize i2c and sensors
    i2c_master_init();
    icm42670_init();
    gyro_init();
    vTaskDelay(pdMS_TO_TICKS(45));
    accel_init();
    vTaskDelay(pdMS_TO_TICKS(45));

    // initialize motor drivers
    pwm_init();
    vTaskDelay(pdMS_TO_TICKS(45));
    l298n_init();

    // initialize timer for non-blocking code
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &bot_balance,
            .name = "bot_timer",
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, clock_period));
}

    // ESP_LOGI(IMU_TAG, "----- Finished Collecting Data -----");