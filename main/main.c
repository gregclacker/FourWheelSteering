#include <stdio.h>
#include <math.h>     //The C math library that is used for some of the complex math functions needed
#include "driver/gpio.h"    //The header file needed to use GPIO Pins and functions
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"      //A header needed to use serial communication
#include "freertos/task.h"      //A header needed to use delays
#include "driver/adc.h"     //The header file needed to use ADC pins and functions
#include "sdkconfig.h"     //Will have the pin setups and variable names
#include "TurnRadiusCalc.h"     //The header file for turn radius calc has the coefficient and power values for the function of each wheel
#include "pwm.h"        //The header file where the PWM Values will be stored
#include "RearAngle.h"

double FS_SteeringAngle; //Input Front Steering Angle, set to -91 for testing the angles, whenever actually implemented this will not have a value
double RS_SteeringAngle;        //Input Rear Steering Angle
double LW_Angle;    //Left Wheel Angle
double RW_Angle;    //Right Wheel Angle
double TR_Left;     //Turn Radius Left Wheel Right Turn
double TR_Right;    //Turn Radius Right Wheel Left Turn
double IDEAL_RS_ANGLE;
double RS_TR_RIGHT;
double RS_WA_RIGHT;
double RS_TR_LEFT;
double RS_WA_LEFT;
double RT_Percentage;
double LT_Percentage;
double RS_Deg;
double deadband = 0;
//double Gain_Input;    //Not sure if this will be used
//The amount of 'clicks' you want to be able to turn the rear steering adjustment knob
int ADJUSTMENT_KNOB_VALUE;      //The value of the adjustment knob that will be used on calculations


#define LED_PIN 2       //This is for the onboard LED (Status LED)
#define FAN_PIN 23      //This is for the constant fan to cool the controller
#define FRONT_STEERING_POT_PIN ADC2_CHANNEL_7 //This is the front steering pot       ADC1_CHANNEL_7 Corresponds to GPIO35 on the pinout diagram
#define REAR_STEERING_POT_PIN ADC2_CHANNEL_5 //This is the rear steering pot        ADC1_CHANNEL_6 Corresponds to GPIO34 on the pinout diagram
#define ADJUSTMENT_POT_PIN ADC1_CHANNEL_4 //This is the adjustment pot      ADC1_CHANNEL_4 Corresponds to GPIO32 on the pinout diagram
//#define MOTOR_DRIVER_1PIN  25      //This is the Motor Driver 1 pin correspodning to GPIO27
//#define MOTOR_DRIVER_2PIN 14        //This is the Motor Driver 2 pin corresponding to GPIO14
#define MOTOR_PWM_PIN 18        //This is the PWM pin corresponding to GPIO12
#define DIR_PIN 19
#define ADJUSTMENT_AMOUNT 7     //The amount of clicks the potentiometer will have, this only needs to be adjusted right here

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double previous_error;
    double integral;
} PID;

double Kp = 0.16;        //The Kp value of the PID
double Ki = 0.000;       //The Ki value of the PID
double Kd = 0.016;        //The Kd value of the PID

PID pid;

void setup_pid(PID *pid) {
pid->Kp = Kp;
pid->Ki = Ki;
pid->Kd = Kd;
pid->previous_error = 0;
pid->integral = 0;
}

#define INTEGRAL_LIMIT 100 // Adjust based on testing

double compute_pid(PID *pid, double error)
{
    double P_out = pid->Kp * error;

    // **Reset integral when error direction changes**
    if ((pid->previous_error > 0 && error < 0) || (pid->previous_error < 0 && error > 0)) {
        pid->integral = 0;
    } else {
        pid->integral += error;
    }

    // **Clamp integral to prevent windup**
    if (pid->integral > INTEGRAL_LIMIT) pid->integral = INTEGRAL_LIMIT;
    if (pid->integral < -INTEGRAL_LIMIT) pid->integral = -INTEGRAL_LIMIT;

    double I_out = pid->Ki * pid->integral;
    
    double derivative = (error - pid->previous_error); 
    double D_out = pid->Kd * derivative;

    pid->previous_error = error;

    double output = P_out + I_out + D_out;

    // Debug Output
    /*printf("PID Debug - Error: %lf, P: %lf, I: %lf, D: %lf, PID Output: %lf\n", 
           error, P_out, I_out, D_out, output);
    */
    return output;
}

int duty_cycle_convert(double pid_output)
{
    int duty = (int)(fabs(pid_output) * (PWM_MAX_DUTY / 15)); // 🔹 Adjusted scaling factor

    // Ensure duty cycle is within valid range
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    
    if (duty > 0 && duty < 300) duty = 400;  // Minimum force to actually move

    return duty;
}

void setup_pwm()
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num = MOTOR_PWM_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);
}

double READ_FS_POT()
{
    int raw_fs_val = 0;
    if (adc2_get_raw(FRONT_STEERING_POT_PIN, ADC_WIDTH_BIT_12, &raw_fs_val) == ESP_OK) {
        double front_voltage = (raw_fs_val / 4095.0) * 3.3;
        double fs_angle = (front_voltage / 3.3) * 270.0 - 131.0;
        return fs_angle;
    } else {
        // Handle error (return 0, NAN, or a special value)
        return 0.0;
    }
}

double READ_RS_POT()
{
    int raw_rs_val = 0;
    if (adc2_get_raw(REAR_STEERING_POT_PIN, ADC_WIDTH_BIT_12, &raw_rs_val) == ESP_OK) {
        double rear_voltage = (raw_rs_val / 4095.0) * 3.3;
        double rs_angle = (rear_voltage / 3.3) * 270 - 135;
        return rs_angle;
    } else {
        return 0.0;
    }
}

int POT_ADJUSTMENT()
{
    int ADJUSTMENT_RAW_VAL = adc1_get_raw(ADJUSTMENT_POT_PIN);  // Get raw ADC value
    int ADJUSTMENT_NUMBER = (int)((ADJUSTMENT_RAW_VAL / 4095.0) * ADJUSTMENT_AMOUNT); 
    return ADJUSTMENT_NUMBER;
}   

void TurnRadiusLeft(double FS_SteeringAngle)        //The Equation for the left wheel turn radius
{   
    TR_Left = TR_LW_Coefficient*pow(FS_SteeringAngle,TR_LW_Power);
}

void TurnRadiusRight(double FS_SteeringAngle)       //The Equation for the right wheel turn radius
{
    TR_Right = TR_RW_Coefficient*pow((-1*FS_SteeringAngle),TR_RW_Power);
}

void WheelAngleLeft(double FS_SteeringAngle)        //The equation for the left wheel angle
{
    LW_Angle = WA_LW_Slope*(FS_SteeringAngle)+WA_LW_Intercept;
}

void WheelAngleRight(double FS_SteeringAngle)       //The equation for the right wheel angle
{
    RW_Angle = WA_RW_Slope*((-1*FS_SteeringAngle))+WA_RW_Intercept;
}

double IdealRearAngle(double FS_SteeringAngle)
{
    if (FS_SteeringAngle > deadband)    //Left Turn Steering Percentage Calc
    {
        LT_Percentage = -50 * tanh(0.1 * (TR_RW_Coefficient * pow(FS_SteeringAngle, TR_RW_Power)) - 4.5) + 50;
        double RearAngle = (LT_Percentage / 100.0) * 135.0;
        RS_Deg = RearAngle;
        return RS_Deg;
    }
    else if (FS_SteeringAngle < -deadband)  // Right Turn Steering Percentage Calc
    {
        RT_Percentage = -50 * tanh(0.1 * (TR_LW_Coefficient * pow(-FS_SteeringAngle, TR_LW_Power)) - 4.5) + 50;
        double RearAngle = (RT_Percentage / 100.0) * 135.0;
        RS_Deg = RearAngle * -1 ;
        return RS_Deg;
    }
    {
        RS_Deg = 0.0;
        return RS_Deg;
    }
        
}

  void app_main(void)
{
    // Set up GPIOs
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(FAN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT); // Direction control
    setup_pwm(); // Set up PWM control

    // Set up ADC for potentiometers
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc2_config_channel_atten(FRONT_STEERING_POT_PIN, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(REAR_STEERING_POT_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADJUSTMENT_POT_PIN, ADC_ATTEN_DB_11);

    // Initialize PID
    setup_pid(&pid);

    while (1)
    {
        // Read potentiometer values
        FS_SteeringAngle = READ_FS_POT();  // Front steering input
        RS_SteeringAngle = READ_RS_POT();  // Rear steering feedback
        ADJUSTMENT_KNOB_VALUE = POT_ADJUSTMENT(); // Additional tuning input

        // Calculate ideal rear steering angle
        IDEAL_RS_ANGLE = IdealRearAngle(FS_SteeringAngle);

        // Compute PID error (Difference between actual and ideal rear steering)
        double error = RS_SteeringAngle - IDEAL_RS_ANGLE ;
        // Apply deadband
        if (fabs(error) < 1.2) {
        error = 0;
        }
 
        double pid_output = compute_pid(&pid, error);

        // Convert PID output into a PWM duty cycle
        int duty_cycle = duty_cycle_convert(pid_output);

        // Ensure duty cycle stays within bounds
        if (duty_cycle > PWM_MAX_DUTY) duty_cycle = PWM_MAX_DUTY;
        if (duty_cycle < 0) duty_cycle = 0;

        // Determine motor direction based on PID output
        if (pid_output > 0) {
            gpio_set_level(DIR_PIN, 1); // Move forward
        } else {
            gpio_set_level(DIR_PIN, 0); // Move backward
        }

        // Apply PWM to control motor speed
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

        // Debugging output to verify PID response
        /*printf("FS: %.2lf, RS: %.2lf, IdealRS: %.2lf, Error: %.2lf, PID: %.2lf\n",
            FS_SteeringAngle, RS_SteeringAngle, IDEAL_RS_ANGLE, error, pid_output);
     
        */
        printf("%.2lf\t %.2lf\n", IDEAL_RS_ANGLE, RS_SteeringAngle);
        // LED Blink for Status (indicates loop is running)
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}





