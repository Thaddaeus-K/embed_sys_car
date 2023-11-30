#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "hardware/i2c.h"

#include "hardware/pwm.h"
#include "pico/time.h" 

// I2C address of the LSM303DLHC magnetometer and accelerometer
#define LSM303DLHC_MAG_ADDR 0x1E 
#define LSM303DLHC_ACC_ADDR 0x19 

// DEfine magnetometer register addresses
#define MAG_CRA_REG 0x00
#define MAG_CRB_REG 0x01
#define MAG_MR_REG 0x02
#define MAG_OUT_X_H 0x03
#define MAG_OUT_X_L 0x04
#define MAG_OUT_Y_H 0x07
#define MAG_OUT_Y_L 0x08
#define MAG_OUT_Z_H 0x05
#define MAG_OUT_Z_L 0x06

// Define accelerometer register addresses
#define ACC_CTRL_REG1 0x20 // Control register 1 for accelerometer
#define ACC_OUT_X_L 0x28 // Output register for X-axis accelerometer
#define ACC_OUT_X_H 0x29
#define ACC_OUT_Y_L 0x2A
#define ACC_OUT_Y_H 0x2B
#define ACC_OUT_Z_L 0x2C
#define ACC_OUT_Z_H 0x2D

// Scale factor for converting magnetometer data to uT
#define MAG_SCALE_FACTOR 0.92

// Scale factor for converting accelerometer data to g (gravity)
#define ACC_SCALE_FACTOR 0.001

// Define pins for motor 1
#define MOTOR1_PWM_PIN 6
#define MOTOR1_IN1_PIN 5
#define MOTOR1_IN2_PIN 4

// Define pins for motor 2
#define MOTOR2_PWM_PIN 7
#define MOTOR2_IN1_PIN 3
#define MOTOR2_IN2_PIN 2

// Define pins for motor 3
#define MOTOR3_PWM_PIN 16
#define MOTOR3_IN1_PIN 17
#define MOTOR3_IN2_PIN 18

// Define pins for motor 4
#define MOTOR4_PWM_PIN 19
#define MOTOR4_IN1_PIN 20
#define MOTOR4_IN2_PIN 21

// Define mode for car control
#define MANUAL 0
#define AUTO 1
int car_mode = MANUAL;

// Define turning cardinal direction
#define LEFT 3
#define RIGHT 4
#define FORWARD 5
#define BACKWARD 6
#define STOP 7


int16_t read16(uint8_t addr, uint8_t deviceAddr) {
    uint8_t data[2];
    i2c_write_blocking(i2c0, deviceAddr, &addr, 1, true);
    i2c_read_blocking(i2c0, deviceAddr, data, 2, false);
    return (data[0] << 8) | data[1];
}

// Define constants for motor control
#define WHEEL_CIRCUMFERENCE_CM 18.85 // need to update
#define PULSES_PER_REVOLUTION 360  // Change with actual value

static uint32_t last_edge_time; 
static uint32_t pulse_count; // Count of pulses or notches detected 
static double wheel_distance; // Distance traveled in centimeters 
static uint32_t speed; 

// Define constants for ultrasonic sensor
int timeout = 30000; // Increase the timeout to handle longer distances
int car_angle_buffer = 32;
int car_length = 28;

const uint trigPin = 13; 
const uint echoPin = 12;

// Function for wheel encoder to calculate distance and speed
void gpio_callback(uint gpio, uint32_t events) { 
    uint32_t current_time = time_us_32(); 
    uint32_t time_since_last_edge = current_time - last_edge_time; 
 
    if (events & GPIO_IRQ_EDGE_RISE) { 
        pulse_count++; // Increment the pulse count 
        wheel_distance = (double)pulse_count * WHEEL_CIRCUMFERENCE_CM / PULSES_PER_REVOLUTION; 
        speed = 1000000 / time_since_last_edge; 
    } 
 
    last_edge_time = current_time; 
} 

// Function to configure and set PWM for a motor
void setPWM(uint m,int s) {
    pwm_set_clkdiv(m, 100);
    pwm_set_wrap(m, 12500);
    pwm_set_chan_level(m, PWM_CHAN_A, (12500/100)*s);
    pwm_set_chan_level(m, PWM_CHAN_B, (12500/100)*s);

}

// Function to move the car based on received instruction
void move(int ins){
    if(ins==FORWARD)
    {
        gpio_put(MOTOR1_IN1_PIN, 1);
        gpio_put(MOTOR1_IN2_PIN, 0);
        gpio_put(MOTOR2_IN1_PIN, 1);
        gpio_put(MOTOR2_IN2_PIN, 0);
        gpio_put(MOTOR3_IN1_PIN, 1);
        gpio_put(MOTOR3_IN2_PIN, 0);
        gpio_put(MOTOR4_IN1_PIN, 1);
        gpio_put(MOTOR4_IN2_PIN, 0);
        // sleep_ms(1000);
    }
    else if (ins==BACKWARD)
    {
        gpio_put(MOTOR1_IN1_PIN, 0);
        gpio_put(MOTOR1_IN2_PIN, 1);
        gpio_put(MOTOR2_IN1_PIN, 0);
        gpio_put(MOTOR2_IN2_PIN, 1);
        gpio_put(MOTOR3_IN1_PIN, 0);
        gpio_put(MOTOR3_IN2_PIN, 1);
        gpio_put(MOTOR4_IN1_PIN, 0);
        gpio_put(MOTOR4_IN2_PIN, 1);
        // sleep_ms(1000);
    }
    else if (ins==LEFT)
    {
        gpio_put(MOTOR1_IN1_PIN, 1);
        gpio_put(MOTOR1_IN2_PIN, 0);
        gpio_put(MOTOR2_IN1_PIN, 0);
        gpio_put(MOTOR2_IN2_PIN, 1);
        gpio_put(MOTOR3_IN1_PIN, 0);
        gpio_put(MOTOR3_IN2_PIN, 1);
        gpio_put(MOTOR4_IN1_PIN, 1);
        gpio_put(MOTOR4_IN2_PIN, 0);
        // sleep_ms(1000);
    }
    else if (ins==STOP)
    {
        gpio_put(MOTOR1_IN1_PIN, 0);
        gpio_put(MOTOR1_IN2_PIN, 0);
        gpio_put(MOTOR2_IN1_PIN, 0);
        gpio_put(MOTOR2_IN2_PIN, 0);
        gpio_put(MOTOR3_IN1_PIN, 0);
        gpio_put(MOTOR3_IN2_PIN, 0);
        gpio_put(MOTOR4_IN1_PIN, 0);
        gpio_put(MOTOR4_IN2_PIN, 0);
        // sleep_ms(1000);
    }
}

// Function to move by fixed distance
void moveForwardDistance(int distance_to_move){
    while (wheel_distance < distance_to_move){
        move(FORWARD);
    }
}

// Function to intialize the pins for the ultrasonic sensor
void setupUltrasonicPins(uint trigPin, uint echoPin)
{
    gpio_init(trigPin);
    gpio_init(echoPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);
}

// Function to get the pulse time in microseconds
uint64_t getPulse(uint trigPin, uint echoPin)
{
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    uint64_t width = 0;

    while (gpio_get(echoPin) == 0) tight_loop_contents();
    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echoPin) == 1)
    {
        width++;
        sleep_us(1);
        if (width > timeout)
            return 0;
    }
    absolute_time_t endTime = get_absolute_time();

    return absolute_time_diff_us(startTime, endTime);
}

// Function to get the ultrasonic-measured distance in cm
float getCm(uint trigPin, uint echoPin)
{
    uint64_t pulseLength = getPulse(trigPin, echoPin);
    if (pulseLength == 0)
        return -1.0; // Error condition

    // Calculate the distance in centimeters
    float distance_cm = (float)pulseLength * 0.01715; // Speed of sound in air = 343 m/s

    return distance_cm;
}

// Function to calculate travel distance around an obstacle
float travel_distance(float dist_to_object, uint trigPin, uint echoPin, float turn_towards)
{
    float saved_distance = dist_to_object;
    float latest_distance = 0;
    float distance_difference = 0;
    float distance_to_travel = 0;

    while (distance_difference < 5)
    {
        // Turn in the direction of the turn_towards

        // Measure current distance, calculate difference
        latest_distance = getCm(trigPin, echoPin);
        distance_difference = latest_distance - saved_distance;

        // Update saved distance to latest distance
        saved_distance = latest_distance;
    }

    // Stop turning
     
    distance_to_travel = latest_distance;

    return distance_to_travel;
}

// Function to caluculate dot product of two vectors
    double dotProduct(double x1, double y1, double x2, double y2) {
    return x1 * x2 + y1 * y2;
}

// Function to normalize a vector
void normalizeVector(double x, double y, double* u, double* v) {
    double magnitude = sqrt(x * x + y * y);
    *u = x / magnitude;
    *v = y / magnitude;
}

// Function to project a point onto a line
double calculateOrthoDistance(double x0, double y0, double heading, double x1, double y1) {
    // Calculate the direction vector of the original line
    double dx = cos(heading);
    double dy = sin(heading);

    // Normalize the direction vector and store the result in u0 and v0
    double u0, v0;
    normalizeVector(dx, dy, &u0, &v0);

    // Calculate the vector for difference between the current and original points
    double dx1 = x1 - x0;
    double dy1 = y1 - y0;

    // Project the vector (dx1, dy1) onto the original line vector (u0, v0)
    double projection = dotProduct(dx1, dy1, u0, v0);

    // Calculate the perpendicular distance between the current point and the original line
    double ortho_distance = projection * sqrt(u0 * u0 + v0 * v0);

    return ortho_distance;
}

int main()
{
    stdio_init_all();
    setupUltrasonicPins(trigPin, echoPin);

    i2c_init(i2c0, 100000);
    gpio_set_function(20, GPIO_FUNC_I2C); // Use GPIO 20 for SDA
    gpio_set_function(21, GPIO_FUNC_I2C); // Use GPIO 21 for SCL
    gpio_pull_up(20);
    gpio_pull_up(21);

        // Initialize and configure the magnetometer
    if (!i2c_write_blocking(i2c0, LSM303DLHC_MAG_ADDR, (const uint8_t[2]){MAG_CRA_REG, 0x14}, 2, true) ||
        !i2c_write_blocking(i2c0, LSM303DLHC_MAG_ADDR, (const uint8_t[2]){MAG_CRB_REG, 0x60}, 2, true) ||
        !i2c_write_blocking(i2c0, LSM303DLHC_MAG_ADDR, (const uint8_t[2]){MAG_MR_REG, 0x00}, 2, true)) {
        printf("Magnetometer I2C write error\n");
        return 1;
    }

    // Initialize and configure the accelerometer
    if (!i2c_write_blocking(i2c0, LSM303DLHC_ACC_ADDR, (const uint8_t[2]){ACC_CTRL_REG1, 0x57}, 2, true)) {
        printf("Accelerometer I2C write error\n");
        return 1;
    }

    // Motor 1
    gpio_set_function(MOTOR1_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num1 = pwm_gpio_to_slice_num(MOTOR1_PWM_PIN);
    setPWM(slice_num1, 100); // Initialize motor 1 to 100%
    gpio_init(MOTOR1_IN1_PIN);
    gpio_init(MOTOR1_IN2_PIN);
    gpio_set_dir(MOTOR1_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR1_IN2_PIN, GPIO_OUT);

    // Motor 2
    gpio_set_function(MOTOR2_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num2 = pwm_gpio_to_slice_num(MOTOR2_PWM_PIN);
    setPWM(slice_num2, 100); // Initialize motor 2 to 100%
    gpio_init(MOTOR2_IN1_PIN);
    gpio_init(MOTOR2_IN2_PIN);
    gpio_set_dir(MOTOR2_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR2_IN2_PIN, GPIO_OUT);

    // Motor 3
    gpio_set_function(MOTOR3_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num3 = pwm_gpio_to_slice_num(MOTOR3_PWM_PIN);
    setPWM(slice_num3, 100); // Initialize motor 3 to 100%
    gpio_init(MOTOR3_IN1_PIN);
    gpio_init(MOTOR3_IN2_PIN);
    gpio_set_dir(MOTOR3_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR3_IN2_PIN, GPIO_OUT);

    // Motor 4
    gpio_set_function(MOTOR4_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num4 = pwm_gpio_to_slice_num(MOTOR4_PWM_PIN);
    setPWM(slice_num4 ,100); // Initialize motor 4 to 100%
    gpio_init(MOTOR4_IN1_PIN);
    gpio_init(MOTOR4_IN2_PIN);
    gpio_set_dir(MOTOR4_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR4_IN2_PIN, GPIO_OUT);

    // Enable motors
    //pwm_set_enabled(slice_num1, true);
    pwm_set_enabled(slice_num1, true);
    pwm_set_enabled(slice_num2, true);
    pwm_set_enabled(slice_num3, true);
    pwm_set_enabled(slice_num4, true);

    gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_RISE, true, &gpio_callback); 
 
    last_edge_time = time_us_32(); 

    while (true)
    {

        // Magnetometer
        // int16_t magX = read16(MAG_OUT_X_H, LSM303DLHC_MAG_ADDR);
        // int16_t magY = read16(MAG_OUT_Y_H, LSM303DLHC_MAG_ADDR);
        // int16_t magZ = read16(MAG_OUT_Z_H, LSM303DLHC_MAG_ADDR);
        
        // int16_t accX = read16(ACC_OUT_X_L, LSM303DLHC_ACC_ADDR);
        // int16_t accY = read16(ACC_OUT_Y_L, LSM303DLHC_ACC_ADDR);
        // int16_t accZ = read16(ACC_OUT_Z_L, LSM303DLHC_ACC_ADDR);

        // double magXuT = magX * MAG_SCALE_FACTOR;
        // double magYuT = magY * MAG_SCALE_FACTOR;
        // double magZuT = magZ * MAG_SCALE_FACTOR;

        // double accXg = accX * ACC_SCALE_FACTOR;
        // double accYg = accY * ACC_SCALE_FACTOR;
        // double accZg = accZ * ACC_SCALE_FACTOR;

        // double magHeading = atan2(magYuT, magXuT) * 180.0 / M_PI;

        // if (magHeading < 0) {
        //     magHeading += 360.0;
        // }

        // printf("Magnetometer - X: %.2f uT, Y: %.2f uT, Z: %.2f uT, Heading: %.2f°\n", magXuT, magYuT, magZuT, magHeading);
        // printf("Accelerometer - X: %.2f g, Y: %.2f g, Z: %.2f g\n", accXg, accYg, accZg);

        // Motor movements
        move(BACKWARD);
        // sleep_ms(2000); 
        // move(FORWARD);
        // sleep_ms(3000); 
        // move(LEFT);
        // sleep_ms(1500); 
        // Print the measured distance and speed 
        printf("Distance: %.2lf cm\n", wheel_distance); 
        printf("Speed: %d pulses per second\n", speed); 
 
        // Sleep for a short interval (adjust as needed) 
        //sleep_ms(2000); 


        // Ultrasonic sensor
        float object_distance = getCm(trigPin, echoPin);
        // float straight_line = sqrt(pow(object_distance, 2) - pow(object_distance, 2));

        // float turn_direction = LEFT;

        // If object is not near, follow manual control
        if (object_distance >= 0)
        {
            printf("Object Distance: %.2f cm\n", object_distance);
            // printf("Straight line distance: %.2f cm\n", straight_line);

            if (object_distance <= 30)
            {
                printf("Object detected ahead.\n");
            }
        }
        // If object is near, auto maneuver around obstacle
        else if (object_distance <= 30)
        {
            printf("Object detected ahead.\n");

            // Disengage manual control, engage auto maneuvering around obstacle
            car_mode = AUTO;

            printf("Auto mode engaged.\n");

        //     // Save current position and heading
        //     double og_x = magXuT;
        //     double og_y = magYuT;
        //     double og_heading = magHeading;

        //     // Find leftmost corner of obstacle and distance to travel to pass obstacle
        //     float leftward_distance = travel_distance(object_distance, trigPin, echoPin, turn_direction);
        //     float left_straight_line = sqrt(pow(object_distance, 2) - pow(leftward_distance, 2));

        //     // Set turn_direction to right
        //     turn_direction = RIGHT;

        //     // Turn back to saved heading

        //     // Find rightmost corner of obstacle and distance to travel to pass obstacle
        //     float rightward_distance = travel_distance(object_distance, trigPin, echoPin, turn_direction);
        //     float right_straight_line = sqrt(pow(object_distance, 2) - pow(rightward_distance, 2));

        //     // Determine if object is a wall
        //     if (left_straight_line > 50 && right_straight_line > 50)
        //     {
        //         printf("Likely a wall\n");

        //         // Stop and turn 90 degrees to the right
        //     }
        //     else
        //     {
        //         // Compare leftward and rightward distances and turn in the direction of the shorter distance
        //         if (leftward_distance < rightward_distance)
        //         {
        //             // Turn back to the left + 32 degrees (to allow for the width of the robot)
                    
        //             // Move forward leftward_distance + car_length;
        //             moveForwardDistance(leftward_distance + car_length);

        //             // Set turn_direction back to left
        //             turn_direction = LEFT;
        //         }
        //         else
        //         {
        //             // Turn right by another 32 degrees (to allow for the width of the robot)

        //             // Move forward rightward_distance + car_length;
        //             moveForwardDistance(rightward_distance + car_length);
        //         }
                
        //         // Turn back to the saved heading and move forward by one car length cm
        //         moveForwardDistance(car_length);

        //         // Turn to face obstacle again
        //         // If obstacle detected, repeat the above steps
        //         // Find distance to travel to pass the corner of the obstacle
        //         float obstacle_distance = travel_distance(object_distance, trigPin, echoPin, turn_direction);

        //         // Move forward by the obstacle_distance + car_length;
        //         moveForwardDistance(obstacle_distance + car_length);

        //         // Turn back to the saved heading and move forward until algined with previous gradient

        //         // Current position
        //         double curr_x = magXuT;
        //         double curr_y = magYuT;

        //         // Calculate the distance from the current position to the original line
        //         double ortho_travel_distance = calculateOrthoDistance(og_x, og_y, og_heading, curr_x, curr_y);

        //         // Return to projected position on original line
        //         // Insert function to turn to face the original line

        //         // Move towards the original line path, and account the length of the car
        //         double move_ortho_distance = ortho_travel_distance + (car_length / 2);
        //         moveForwardDistance(move_ortho_distance);
        //         // Turn to face original heading
        //     }
            
        //     // Resume manual control
        //     car_mode = MANUAL;
        }
        // // If ultrasonic sensor fails to read distance
        // else
        // {
        //     printf("Error reading distance\n");

        //     // Stop the car maybe???
        // }
        // sleep_ms(100); // Update distance and magnetometer ever 0.001s
    }

    return 0;
}
