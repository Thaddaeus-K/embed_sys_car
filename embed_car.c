#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "hardware/i2c.h"

#include "hardware/pwm.h"
#include "pico/time.h" 



#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1


// MAGNETOMETER
#define LSM303DLHC_MAG_ADDR 0x1E // I2C address of the LSM303DLHC magnetometer
#define LSM303DLHC_ACC_ADDR 0x19 // I2C address of the LSM303DLHC accelerometer

// Magnetometer register addresses
#define MAG_CRA_REG 0x00
#define MAG_CRB_REG 0x01
#define MAG_MR_REG 0x02
#define MAG_OUT_X_H 0x03
#define MAG_OUT_X_L 0x04
#define MAG_OUT_Y_H 0x07
#define MAG_OUT_Y_L 0x08
#define MAG_OUT_Z_H 0x05
#define MAG_OUT_Z_L 0x06

// Accelerometer register addresses
#define ACC_CTRL_REG1 0x20 // Control register 1 for accelerometer
#define ACC_OUT_X_L 0x28 // Output register for X-axis accelerometer
#define ACC_OUT_X_H 0x29
#define ACC_OUT_Y_L 0x2A
#define ACC_OUT_Y_H 0x2B
#define ACC_OUT_Z_L 0x2C
#define ACC_OUT_Z_H 0x2D

#define MAG_SCALE_FACTOR 0.92 // Scale factor for converting magnetometer data to uT
#define ACC_SCALE_FACTOR 0.001 // Scale factor for converting accelerometer data to g (gravity)

// Define magnetometer pins
#define MAG_SDA_PIN 20
#define MAG_SCL_PIN 21



int16_t read16(uint8_t addr, uint8_t deviceAddr) {
    uint8_t data[2];
    i2c_write_blocking(i2c0, deviceAddr, &addr, 1, true);
    i2c_read_blocking(i2c0, deviceAddr, data, 2, false);
    return (data[0] << 8) | data[1];
}

double getMagHeading() {
    int16_t magX = 0;
    int16_t magY = 0;
    int16_t magZ = 0;

    magX = read16(MAG_OUT_X_H, LSM303DLHC_MAG_ADDR);
    magY = read16(MAG_OUT_Y_H, LSM303DLHC_MAG_ADDR);
    magZ = read16(MAG_OUT_Z_H, LSM303DLHC_MAG_ADDR);

    double magXuT = magX * MAG_SCALE_FACTOR;
    double magYuT = magY * MAG_SCALE_FACTOR;
    double magZuT = magZ * MAG_SCALE_FACTOR;

    double magHeading = atan2(magYuT, magXuT) * 180.0 / M_PI;

    if (magHeading < 0) {
        magHeading += 360.0;
    }

    return magHeading;
}


// MOTORS
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
#define MOTOR4_IN1_PIN 10 // 20
#define MOTOR4_IN2_PIN 11 // 21

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
#define CRAB_LEFT 8
#define CRAB_RIGHT 9
#define TOPRIGHT 10 
#define TOPLEFT 11
#define BTMLEFT 12
#define BTMRIGHT 13

// Define constants for motor control
#define WHEEL_CIRCUMFERENCE_CM 18.85 // need to update
#define PULSES_PER_REVOLUTION 360  // Change with actual value

static uint32_t last_edge_time; 
static uint32_t pulse_count; // Count of pulses or notches detected 
static double wheel_distance; // Distance traveled in centimeters 
static uint32_t speed; 

// Function for wheel encoder to calculate distance and speed
void gpio_callback(uint gpio, uint32_t events) 
{ 
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
void setPWM(uint m,int s) 
{
    pwm_set_clkdiv(m, 100);
    pwm_set_wrap(m, 12500);
    pwm_set_chan_level(m, PWM_CHAN_A, (12500/100)*s);
    pwm_set_chan_level(m, PWM_CHAN_B, (12500/100)*s);
}

// Function to move the car based on received instruction
void move(int ins)
{
    if(ins==BACKWARD)
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
    else if (ins==FORWARD)
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
    else if (ins==CRAB_LEFT)
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
    else if (ins==CRAB_RIGHT)
    {
        gpio_put(MOTOR1_IN1_PIN, 0);
        gpio_put(MOTOR1_IN2_PIN, 1);
        gpio_put(MOTOR2_IN1_PIN, 1);
        gpio_put(MOTOR2_IN2_PIN, 0);
        gpio_put(MOTOR3_IN1_PIN, 1);
        gpio_put(MOTOR3_IN2_PIN, 0);
        gpio_put(MOTOR4_IN1_PIN, 0);
        gpio_put(MOTOR4_IN2_PIN, 1);
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
    else if (ins==RIGHT)
    {
        gpio_put(MOTOR1_IN1_PIN, 0);
        gpio_put(MOTOR1_IN2_PIN, 1);
        gpio_put(MOTOR2_IN1_PIN, 1);
        gpio_put(MOTOR2_IN2_PIN, 0);
        gpio_put(MOTOR3_IN1_PIN, 0);
        gpio_put(MOTOR3_IN2_PIN, 1);
        gpio_put(MOTOR4_IN1_PIN, 1);
        gpio_put(MOTOR4_IN2_PIN, 0);
        // sleep_ms(1000);
    }
    else if (ins==LEFT)
    {
        gpio_put(MOTOR1_IN1_PIN, 1);
        gpio_put(MOTOR1_IN2_PIN, 0);
        gpio_put(MOTOR2_IN1_PIN, 0);
        gpio_put(MOTOR2_IN2_PIN, 1);
        gpio_put(MOTOR3_IN1_PIN, 1);
        gpio_put(MOTOR3_IN2_PIN, 0);
        gpio_put(MOTOR4_IN1_PIN, 0);
        gpio_put(MOTOR4_IN2_PIN, 1);
        // sleep_ms(1000);
    }
    else if (ins==TOPRIGHT)
    {
        gpio_put(MOTOR1_IN1_PIN, 0);
        gpio_put(MOTOR1_IN2_PIN, 0);
        gpio_put(MOTOR2_IN1_PIN, 0);
        gpio_put(MOTOR2_IN2_PIN, 1);
        gpio_put(MOTOR3_IN1_PIN, 0);
        gpio_put(MOTOR3_IN2_PIN, 1);
        gpio_put(MOTOR4_IN1_PIN, 0);
        gpio_put(MOTOR4_IN2_PIN, 0);
        // sleep_ms(1000);
    }
    else if (ins==TOPLEFT)
    {
        gpio_put(MOTOR1_IN1_PIN, 0);
        gpio_put(MOTOR1_IN2_PIN, 1);
        gpio_put(MOTOR2_IN1_PIN, 0);
        gpio_put(MOTOR2_IN2_PIN, 0);
        gpio_put(MOTOR3_IN1_PIN, 0);
        gpio_put(MOTOR3_IN2_PIN, 0);
        gpio_put(MOTOR4_IN1_PIN, 0);
        gpio_put(MOTOR4_IN2_PIN, 1);
        // sleep_ms(1000);
    }
    else if (ins==BTMLEFT)
    {
        gpio_put(MOTOR1_IN1_PIN, 0);
        gpio_put(MOTOR1_IN2_PIN, 0);
        gpio_put(MOTOR2_IN1_PIN, 1);
        gpio_put(MOTOR2_IN2_PIN, 0);
        gpio_put(MOTOR3_IN1_PIN, 1);
        gpio_put(MOTOR3_IN2_PIN, 0);
        gpio_put(MOTOR4_IN1_PIN, 0);
        gpio_put(MOTOR4_IN2_PIN, 0);
        // sleep_ms(1000);
    }else if (ins==BTMRIGHT)
    {
        gpio_put(MOTOR1_IN1_PIN, 1);
        gpio_put(MOTOR1_IN2_PIN, 0);
        gpio_put(MOTOR2_IN1_PIN, 0);
        gpio_put(MOTOR2_IN2_PIN, 0);
        gpio_put(MOTOR3_IN1_PIN, 0);
        gpio_put(MOTOR3_IN2_PIN, 0);
        gpio_put(MOTOR4_IN1_PIN, 1);
        gpio_put(MOTOR4_IN2_PIN, 0);
        // sleep_ms(1000);
    }
}

// Function to move by fixed distance
void moveForwardDistance(int distance_to_move)
{
    while (wheel_distance < wheel_distance + distance_to_move)
    {
        move(FORWARD);
    }
}

// Function to turn by fixed angle
void turnToHeading(double saved_heading, int turn_direction)

{
    i2c_init(i2c0, 100000);
    gpio_set_function(MAG_SDA_PIN, GPIO_FUNC_I2C); // Use GPIO 20 for SDA
    gpio_set_function(MAG_SCL_PIN, GPIO_FUNC_I2C); // Use GPIO 21 for SCL
    gpio_pull_up(MAG_SDA_PIN);
    gpio_pull_up(MAG_SCL_PIN);

    double dest_lower_buffer = saved_heading - 2;
    double dest_upper_buffer = saved_heading + 2;
    double current_heading;
    if (dest_lower_buffer < 0) 
    {
        dest_lower_buffer += 360.0;
    }
    else if (dest_lower_buffer > 360) {
        dest_lower_buffer -= 360.0;
    }

    if (dest_upper_buffer < 0) 
    {
        dest_upper_buffer += 360.0;
    }
    else if (dest_upper_buffer > 360) {
        dest_upper_buffer -= 360.0;
    }

    // Turn until reach final heading
    while(1)
    {

        int16_t magX = read16(MAG_OUT_X_H, LSM303DLHC_MAG_ADDR);
        int16_t magY = read16(MAG_OUT_Y_H, LSM303DLHC_MAG_ADDR);
        int16_t magZ = read16(MAG_OUT_Z_H, LSM303DLHC_MAG_ADDR);

        // magX = read16(MAG_OUT_X_H, LSM303DLHC_MAG_ADDR);
        // magY = read16(MAG_OUT_Y_H, LSM303DLHC_MAG_ADDR);
        // magZ = read16(MAG_OUT_Z_H, LSM303DLHC_MAG_ADDR);

        double magXuT = magX * MAG_SCALE_FACTOR;
        double magYuT = magY * MAG_SCALE_FACTOR;
        double magZuT = magZ * MAG_SCALE_FACTOR;

        double magHeading = atan2(magYuT, magXuT) * 180.0 / M_PI;

        if (magHeading < 0) {
            magHeading += 360.0;
        }

        current_heading = magHeading;
        printf("", current_heading);
    }
}


// ULTRASONIC SENSOR
// Define constants for ultrasonic sensor
int timeout = 30000; // Increase the timeout to handle longer distances
int car_angle_buffer = 32;
int car_length = 28;
int max_dist_to_object = 30;
int wall_length_threshold = 50;

const uint trigPin = 13; 
const uint echoPin = 12;

int carturnfinish;

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

// Define a struct to hold the return values of travel_distance()
struct TravelDistanceResult {
    float distance_to_travel;
    float straight_line;
};

struct TravelDistanceResult2 {
    float distance_to_travel;
    float straight_line;
};

// Function to calculate travel distance around an obstacle
struct TravelDistanceResult travel_distance(float dist_to_object, uint trigPin, uint echoPin, int turn_towards)
{
    struct TravelDistanceResult TDresult;

    carturnfinish = 0;

    float saved_distance = dist_to_object;
    float latest_distance = 0;
    float distance_difference = 0;
    float straight_line = 0;

    // Keep measuring the ultrasonic sensor until the difference is too large (corner found) or the straight line distance is too large (assume wall)
    // while ((distance_difference < 5) && (straight_line < wall_length_threshold))
    while (1)
    {

        if ((distance_difference < 5) && (straight_line < wall_length_threshold))
        {
            // Turn in the direction of the turn_towards
            move(turn_towards);

            // Measure current distance, calculate difference
            latest_distance = getCm(trigPin, echoPin);
            distance_difference = latest_distance - saved_distance;

            // Calculate straight line distance along obstacle
            straight_line = sqrt(pow(latest_distance,2) - pow(dist_to_object,2));

            // Update saved distance to latest distance
            saved_distance = latest_distance;

        }

        else
        {
            // Stop turning
            move(STOP);
            carturnfinish = 1;
            sleep_ms(1000);
            break;
        }
      
        
    }

    
    
    TDresult.distance_to_travel = latest_distance;
    TDresult.straight_line = straight_line;

    // // printf("distance to travel: %2f\n", TDresult.distance_to_travel);
    // // printf("straight line: %2f\n", TDresult.straight_line);

    return TDresult;
}


struct TravelDistanceResult2 travel_distance2(float dist_to_object, uint trigPin, uint echoPin, int turn_towards, int carturnright)
{
    struct TravelDistanceResult2 TDresult;

    //carturnfinish = 0;

    float saved_distance = dist_to_object;
    float latest_distance = 0;
    float distance_difference = 0;
    float straight_line = 0;

    // Keep measuring the ultrasonic sensor until the difference is too large (corner found) or the straight line distance is too large (assume wall)
    // while ((distance_difference < 5) && (straight_line < wall_length_threshold))
    while (1)
    {

        if (carturnright == 2)
        {
            // Turn in the direction of the turn_towards
                move(turn_towards);
                sleep_ms(50);

            if (distance_difference < 3)
            {
                 move(turn_towards);
                // Measure current distance, calculate difference
                latest_distance = getCm(trigPin, echoPin);
                distance_difference = latest_distance - saved_distance;

                // // Calculate straight line distance along obstacle
                // straight_line = sqrt(pow(latest_distance,2) - pow(dist_to_object,2));

                // Update saved distance to latest distance
                saved_distance = latest_distance;

            }
             else
            {
                // Stop turning
                move(STOP);
                carturnfinish = 1;
                sleep_ms(1000);
                break;
            }
        }

       
      
        
    }

    
    
    TDresult.distance_to_travel = latest_distance;
    TDresult.straight_line = straight_line;

    // // printf("distance to travel: %2f\n", TDresult.distance_to_travel);
    // // printf("straight line: %2f\n", TDresult.straight_line);

    return TDresult;
}


bool checkcarhasturn()
{
    return carturnfinish;
}

// Define a struct to hold the return values of firstObjectAvoidance()
struct ObjectAvoidanceResult {
    float leftward_distance;
    float left_straight_line;
    double left_heading;
    float rightward_distance;
    float right_straight_line;
    double right_heading;
};

// Function to check left and right corners of obstacle and return the results
struct ObjectAvoidanceResult firstObjectAvoidance(float object_distance, uint trigPin, uint echoPin, double saved_heading) {
    struct ObjectAvoidanceResult OAresult;

    OAresult.leftward_distance = 0;
    OAresult.rightward_distance = 0;

    int car_state = 1;

    
    while(car_state == 1)
    {

        //turnToHeading(saved_heading,RIGHT);
        // printf("whileloop working");
        sleep_ms(3000);
        struct TravelDistanceResult left_result = travel_distance(object_distance, trigPin, echoPin, LEFT);
        OAresult.leftward_distance = left_result.distance_to_travel;
        OAresult.left_straight_line = left_result.straight_line;
        OAresult.left_heading = getMagHeading();
        
        printf("left heading: %2f\n", OAresult.left_heading);
        printf("left distance: %2f\n", OAresult.leftward_distance);
        printf("left straight line: %2f\n", OAresult.left_straight_line);

        sleep_ms(2000);
        bool carHasTurned = checkcarhasturn();
        if(carHasTurned == 1)
        {
            // car_state = RIGHT;
            
            break;
            
        }
    }
    move(STOP);
    sleep_ms(1000);
    printf("saved heading: %2f\n", saved_heading);
    return OAresult;
}




struct ObjectAvoidanceResult secondObjectAvoidance(float object_distance, uint trigPin, uint echoPin) {
    struct ObjectAvoidanceResult OAresult;

    //OAresult.leftward_distance = 0;
    OAresult.rightward_distance = 0;

    int car_state = 2;
    
    while(car_state == 2)
    {
        sleep_ms(3000);
       struct TravelDistanceResult2 right_result = travel_distance2(object_distance, trigPin, echoPin, RIGHT,car_state);
        OAresult.rightward_distance = right_result.distance_to_travel;
        OAresult.right_straight_line = right_result.straight_line;
        OAresult.right_heading = getMagHeading();
        
        printf("right heading: %2f\n", OAresult.right_heading);
        printf("right distance: %2f\n", OAresult.rightward_distance);
        printf("right straight line: %2f\n", OAresult.right_straight_line);

        sleep_ms(2000);
        bool carHasTurned = checkcarhasturn();
        if(carHasTurned == 1)
        {
            // car_state = RIGHT;
            break;
        }
    }

    move(STOP);
    sleep_ms(1000);

    return OAresult;
}




// ORTHOGONAL PROJECTION
// Function to caluculate dot product of two vectors
double dotProduct(double x1, double y1, double x2, double y2) 
{
    return x1 * x2 + y1 * y2;
}

// Function to normalize a vector
void normalizeVector(double x, double y, double* u, double* v) 
{
    double magnitude = sqrt(x * x + y * y);
    *u = x / magnitude;
    *v = y / magnitude;
}

// Function to project a point onto a line
double calculateOrthoDistance(double x0, double y0, double heading, double x1, double y1) 
{
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
    char charData;
    stdio_init_all();

    uart_init(UART_ID, BAUD_RATE);
    
    int carmovingstate = 0;
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    stdio_init_all();

    // MAGNETOMETER SET UP
    i2c_init(i2c0, 100000);
    gpio_set_function(MAG_SDA_PIN, GPIO_FUNC_I2C); // Use GPIO 20 for SDA
    gpio_set_function(MAG_SCL_PIN, GPIO_FUNC_I2C); // Use GPIO 21 for SCL
    gpio_pull_up(MAG_SDA_PIN);
    gpio_pull_up(MAG_SCL_PIN);

      // ULTRASONIC SENSOR SET UP
    setupUltrasonicPins(trigPin, echoPin);

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

    
    // MOTORS SET UP
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
    int z = 0;
    while (true)
    
    {
        // // Measure distance to object
        float object_distance = getCm(trigPin, echoPin);

         double magHeading = getMagHeading();

        int16_t magX = read16(MAG_OUT_X_H, LSM303DLHC_MAG_ADDR);
        int16_t magY = read16(MAG_OUT_Y_H, LSM303DLHC_MAG_ADDR);
        int16_t magZ = read16(MAG_OUT_Z_H, LSM303DLHC_MAG_ADDR);

        int16_t accX = read16(ACC_OUT_X_L, LSM303DLHC_ACC_ADDR);
        int16_t accY = read16(ACC_OUT_Y_L, LSM303DLHC_ACC_ADDR);
        int16_t accZ = read16(ACC_OUT_Z_L, LSM303DLHC_ACC_ADDR);

        double magXuT = magX * MAG_SCALE_FACTOR;
        double magYuT = magY * MAG_SCALE_FACTOR;
        double magZuT = magZ * MAG_SCALE_FACTOR;

        double accXg = accX * ACC_SCALE_FACTOR;
        double accYg = accY * ACC_SCALE_FACTOR;
        double accZg = accZ * ACC_SCALE_FACTOR;

        printf("Magnetometer - X: %.2f uT, Y: %.2f uT, Z: %.2f uT, Heading: %.2f°\n", magXuT, magYuT, magZuT, magHeading);
        printf("Accelerometer - X: %.2f g, Y: %.2f g, Z: %.2f g\n", accXg, accYg, accZg);

        move(FORWARD);
        printf("Distance: %.2lf cm\n", wheel_distance); 
        // printf("Speed: %d pulses per second\n", speed); 
        printf("heading: %.2f\n", getMagHeading());
        sleep_ms(100);

        if (object_distance <= 30)
        {
                car_mode = AUTO;
                printf("object detected\n");
                move(STOP);
                sleep_ms(1000);

                // // Save current position and heading
                double og_heading = getMagHeading();
                printf("og heading: %2f\n", og_heading);

                // struct ObjectAvoidanceResult result = 
                firstObjectAvoidance(object_distance, trigPin, echoPin, og_heading);
        }

        if (uart_is_readable(UART_ID))
        {   
            // printf("UART detected\n");
            char charData = uart_getc(UART_ID);
            // printf("%c\n", charData);

            if(charData == 'r')
            {
                // printf("UART RECEIVE 1 \n");
                move(RIGHT);
                printf("CAR MOVING RIGHT TURNING\n");
                
                if(z==1){
                    move(STOP);
                    z=0;
                }else{
                z=1;}
            }

            if(charData == 'l')
            {
                // printf("UART RECEIVE 1 \n");
                move(LEFT);
                printf("CAR MOVING LEFT TURNING\n");
                if(z==1){
                    move(STOP);
                    z=0;
                }else{
                z=1;}
            }

            if ((charData == 'a' || charData == 's')&&z==0)
            {
                //move(RIGHT);
                move(STOP);
                printf("CAR STOP\n");
               
            }
            if(charData == 'b'&&z==0)
            {
                // printf("UART RECEIVE 1 \n");
                move(FORWARD);
                printf("CAR MOVING FORWARD\n");
            }
            
            if(charData == 'd'&&z==0)
            {
                // printf("UART RECEIVE 1 \n");
                move(CRAB_LEFT);
                printf("CAR MOVING LEFT\n");
            }
            if(charData == 'e'&&z==0)
            {
                // printf("UART RECEIVE 1 \n");
                move(CRAB_RIGHT);
                printf("CAR MOVING RIGHT\n");
            }
            if(charData == 'h'&&z==0)
            {
                // printf("UART RECEIVE 1 \n");
                move(BTMLEFT);
                printf("CAR MOVING BOTTOM LEFT\n");
            }
            if(charData == 'j'&&z==0)
            {
                // printf("UART RECEIVE 1 \n");
                move(BTMRIGHT);
                printf("CAR MOVING BOTTOM RIGHT\n");
            }
            if(charData == 'f'&&z==0)
            {
                // printf("UART RECEIVE 1 \n");
                move(TOPLEFT);
                printf("CAR MOVING TOP LEFT\n");
            }
            if(charData == 'g'&&z==0)
            {
                // printf("UART RECEIVE 1 \n");
                move(TOPRIGHT);
                printf("CAR MOVING TOP RIGHT\n");
            }

            if(charData == 'z'&&z==0)
            {
                // printf("UART RECEIVE 1 \n");
                move(BACKWARD);
                printf("CAR MOVING BACKWARD\n");
            }
 
            sleep_ms(22);
        }
    }
    
    return 0;
    
}