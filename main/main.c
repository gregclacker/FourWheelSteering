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
//double Gain_Input;    //Not sure if this will be used
//The amount of 'clicks' you want to be able to turn the rear steering adjustment knob
int ADJUSTMENT_KNOB_VALUE;      //The value of the adjustment knob that will be used on calculations


#define LED_PIN 2       //This is for the onboard LED (Status LED)
#define FAN_PIN 23      //This is for the constant fan to cool the controller
#define FRONT_STEERING_POT_PIN ADC1_CHANNEL_7 //This is the front steering pot       ADC1_CHANNEL_7 Corresponds to GPIO35 on the pinout diagram
#define REAR_STEERING_POT_PIN ADC1_CHANNEL_6 //This is the rear steering pot        ADC1_CHANNEL_6 Corresponds to GPIO34 on the pinout diagram
#define ADJUSTMENT_POT_PIN ADC1_CHANNEL_4 //This is the adjustment pot      ADC1_CHANNEL_4 Corresponds to GPIO32 on the pinout diagram
#define MOTOR_DRIVER_1PIN 27        //This is the Motor Driver 1 pin correspodning to GPIO27
#define MOTOR_DRIVER_2PIN 14        //This is the Motor Driver 2 pin corresponding to GPIO14
#define MOTOR_PWM_PIN 12        //This is the PWM pin corresponding to GPIO12
#define ADJUSTMENT_AMOUNT 7     //The amount of clicks the potentiometer will have, this only needs to be adjusted right here

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double previous_error;
    double integral;
} PID;

double Kp = 0.5;        //The Kp value of the PID
double Ki = 0.01;       //The Ki value of the PID
double Kd = 0.3;        //The Kd value of the PID

/*
double Kp_min = #;
double Kp_max = #;

*/

PID pid;

void setup_pid(PID *pid) {
pid->Kp = Kp;
pid->Ki = Ki;
pid->Kd = Kd;
pid->previous_error = 0;
pid->integral = 0;
}

double compute_pid(PID *pid, double error){
    double P_out = pid->Kp*error;
    pid->integral += error;
    double I_out = pid->Ki * pid->integral;
    double derivative = error - pid ->previous_error;
    double D_out = pid ->Kd * derivative;
    pid->previous_error = error;
    return P_out + I_out + D_out;

}

void move_forward(){
    gpio_set_level(MOTOR_DRIVER_1PIN, 1);
    gpio_set_level(MOTOR_DRIVER_2PIN,0);
}

void move_backward(){
    gpio_set_level(MOTOR_DRIVER_1PIN, 0);
    gpio_set_level(MOTOR_DRIVER_2PIN, 1);
}

int duty_cycle_convert(double gain_error){
    int convert_error = (int)(gain_error * PWM_MAX_DUTY);
    if (convert_error > PWM_MAX_DUTY)
    {
        return PWM_MAX_DUTY;
    }
    else if (convert_error < -PWM_MAX_DUTY)
    {
        return -PWM_MAX_DUTY;
    }
    else
    {
        return convert_error;
    }
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

double READ_FS_POT()       //Reads potentiometer voltage, converts it from analog to digital 
{
    int RAW_FS_VAL = adc1_get_raw(FRONT_STEERING_POT_PIN);        //Gets the raw value of the voltage
    double FRONT_VOLTAGE = ((RAW_FS_VAL / 4095.0) *3.3);     //Converts the bits to a corresponding voltage
    int FS_ANGLE = (FRONT_VOLTAGE/3.3)* 180-90;       //Maps the voltage to the angle from -90 to 90
    return FS_ANGLE;       //The return angle is then mapped to FS_SteeringAngle when the function is called
}

double READ_RS_POT()
{
    int RAW_RS_VAL = adc1_get_raw(REAR_STEERING_POT_PIN);
    double REAR_VOLTAGE = ((RAW_RS_VAL/4095.0)*3.3);
    double RS_ANGLE = (REAR_VOLTAGE/3.3)*180-90;        //The Mapping of this will need to be changed at some point because we wont have that much rear steer
    return RS_ANGLE;
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
    if (FS_SteeringAngle > 0)
    {
        RS_TR_RIGHT = RS_TR_RW_COEFFICIENT * pow((-1 * FS_SteeringAngle), RS_TR_RW_POWER);
        RS_WA_RIGHT = RS_WA_RW_SLOPE*(-1*FS_SteeringAngle)+RS_WA_RW_INTERCEPT;
        return RS_TR_RIGHT;
    }
    if (FS_SteeringAngle < 0)
    {
        RS_TR_LEFT = RS_TR_LW_COEFFICIENT*pow((-1 * FS_SteeringAngle), RS_TR_LW_POWER);
        RS_WA_LEFT = RS_WA_LW_SLOPE*(-1 * FS_SteeringAngle)+RS_WA_LW_INTERCEPT;
        return RS_TR_LEFT;
    }
    else
    {
        return 0;
    }
    /*
    Double Calculated_RS = f(x);
    return Calculated_RS;
    */
}

void app_main(void)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(FAN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_DRIVER_1PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_DRIVER_2PIN, GPIO_MODE_OUTPUT);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(FRONT_STEERING_POT_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(REAR_STEERING_POT_PIN, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADJUSTMENT_POT_PIN, ADC_ATTEN_DB_11);

    //gpio_set_direction(FAN_PIN, GPIO_MODE_OUTPUT);
    while(1)        //This means while the system is running to repeat this loop
    {
        
        setup_pwm();
        FS_SteeringAngle = READ_FS_POT();       //This calls the function above and gets the front steering angle based on pot position
        RS_SteeringAngle = READ_RS_POT();       //This calls the function above and gets the rear steering angle based on pot position
        ADJUSTMENT_KNOB_VALUE = POT_ADJUSTMENT();       //This calls the function above and gets the adjustment value based on pot position
        gpio_set_level(LED_PIN, 1);     //This turns the Status LED On
        //gpio_set_level(FAN_PIN, 1);       //This means the fan will be running as long as the system is on
        IDEAL_RS_ANGLE = IdealRearAngle(FS_SteeringAngle);
        double error = RS_SteeringAngle - IDEAL_RS_ANGLE;
        double pid_output = compute_pid(&pid, error);
        int duty_cycle = duty_cycle_convert(pid_output);
    
    if (duty_cycle > -100 && duty_cycle < 100){
        duty_cycle = 0;
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, abs(duty_cycle));
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

        if (duty_cycle > 0)
            {
                move_backward();
            }
        else
            {
                move_forward();
            }
    }
        /*
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        FS_SteeringAngle += 1;
        */
        
    if(FS_SteeringAngle > 0 && FS_SteeringAngle <= 90)      //This says with 0 being TDC if the angle is above that it calculates for left wheel
    {
        TurnRadiusLeft(FS_SteeringAngle);
        WheelAngleLeft(FS_SteeringAngle);
        printf("Turn Radius Left \t%lf\n", TR_Left);    //Prints to the terminal, mainly used for testing
        printf("Wheel Angle Left \t%lf\n", LW_Angle);     //See above
    }
    if(FS_SteeringAngle < 0 && FS_SteeringAngle >= -90)     //This says with 0 being TDC if the angle is above that it calculates for right wheel
    {
        TurnRadiusRight(FS_SteeringAngle);
        WheelAngleRight(FS_SteeringAngle);
        printf("Turn Radius Right \t%lf\n", TR_Right);      //Prints to the terminal, mainly used for testing
        printf("Wheel Angle Right \t%lf\n", RW_Angle);    //See Above
    }
    if(FS_SteeringAngle==0)
    {
        printf("Turn Radius \t0\n");
        printf("Wheel Angle \t0\n");
    }
    printf("Knob Value \t%i\n\n",ADJUSTMENT_KNOB_VALUE);

        vTaskDelay(10/portTICK_PERIOD_MS);        //This is the time at which the terminal will be updated in miliseconds

    }
}
