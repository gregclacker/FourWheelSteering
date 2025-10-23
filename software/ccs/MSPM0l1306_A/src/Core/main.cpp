/*
 * main.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *      https://cataas.com/cat/says/accumulating
 */
// RTLinux when? idk how to work with that :(


#include <stdio.h>
#include <stdint.h>
#include <cstdio>


#include "system.hpp"

void thing( void * ){
    auto &led = System::GPIO::PA27;
    DL_GPIO_initDigitalOutputFeatures(
            led.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(GPIOPINPUX(led));
    DL_GPIO_enableOutput(GPIOPINPUX(led));

    for(;;){
        for(int i = 0; i < 1000; i++){
            led.set();
            delay_cycles(10);
            led.clear();
            delay_cycles(20);
        }

        led.clear();
        delay_cycles(10e6);
    }

}

/*************************************************************/
GPTIMER_Regs * PWMTimer = TIMG1;
constexpr uint32_t PWMMAX = 0xFFFF;

void setPWM(uint32_t val){
    if(val >= PWMMAX)
        val = PWMMAX - 1;

    if(val == 0)
        val = PWMMAX;

    DL_Timer_setCaptureCompareValue(PWMTimer, val, DL_TIMER_CC_INDEX::DL_TIMER_CC_0_INDEX);
}

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

#define ESP_OK 0
//PA1 - PA9
System::GPIO::GPIO ADC2_CHANNEL_1 = System::GPIO::PA1;
System::GPIO::GPIO ADC2_CHANNEL_2 = System::GPIO::PA2;
System::GPIO::GPIO ADC1_CHANNEL_3 = System::GPIO::PA3;

#define ADC12_0_INST                                                        ADC0
#define ADC12_0_INST_IRQHandler                                  ADC0_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC0_INT_IRQn)

#define LED_PIN 2       //This is for the onboard LED (Status LED)
#define FAN_PIN 23      //This is for the constant fan to cool the controller
#define FRONT_STEERING_POT_PIN ADC2_CHANNEL_1 //This is the front steering pot       ADC1_CHANNEL_1 Corresponds to GPIO35 on the pinout diagram
#define REAR_STEERING_POT_PIN ADC2_CHANNEL_2 //This is the rear steering pot        ADC1_CHANNEL_2 Corresponds to GPIO34 on the pinout diagram
#define ADJUSTMENT_POT_PIN ADC1_CHANNEL_3 //This is the adjustment pot      ADC1_CHANNEL_3 Corresponds to GPIO32 on the pinout diagram
//#define MOTOR_DRIVER_1PIN  25      //This is the Motor Driver 1 pin correspodning to GPIO27
//#define MOTOR_DRIVER_2PIN 14        //This is the Motor Driver 2 pin corresponding to GPIO14
#define MOTOR_PWM_PIN 18        //This is the PWM pin corresponding to GPIO12
#define DIR_PIN 19
#define ADJUSTMENT_AMOUNT 7     //The amount of clicks the potentiometer will have, this only needs to be adjusted right here

#define TURNRADIUSCALC_H

#define WA_LW_Slope 0.3117f     //This is the slope of the left wheel, wheel angle vs turn radius
#define WA_LW_Intercept -1.6651f     //This is the Y intercept of the left wheel, wheel angle vs turn radius
#define WA_RW_Slope 0.2286f     //This is the slope of the right wheel, wheel angle vs turn radius
#define WA_RW_Intercept -0.194f     //This is the Y intercept of the right wheel, wheel angle vs turn radius
#define TR_LW_Coefficient 2677.7f  //This is the left wheel coefficient of the power function for Turn Radius vs Steering Angle
#define TR_LW_Power -1.147f    //This is the left wheel power of the power function for Turn Radius vs Steering Angle
#define TR_RW_Coefficient 2760.0f  //This is the right wheel coefficient of the power function for Turn Radius vs Steering Angle
#define TR_RW_Power -1.222f    //This is the right wheel power of the power function for Turn Radius vs Steering Angle

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
//PA0 - Digital Output | A7 - Analog Input
//Go forward when A7 <= 180 or A7 >= 0
//Go backward when A7 > 180 or A7 < 0
int setDirection(System::GPIO::GPIO p_pin, uint8_t p_dir) {
    //p_pin
    //p_dir
}

/** THIS IS FOR REFRENCE
 *
 * ADC Methods:
    __STATIC_INLINE DL_ADC12_getMemResult(
        const ADC12_Regs *adc12, DL_ADC12_MEM_IDX idx)                          // Read Digial output from the ADC

    __STATIC_INLINE void DL_GPIO_writePinsVal(
        GPIO_Regs* gpio, uint32_t pinsMask, uint32_t pinsVal)                   //

    Pin Methods:
    __STATIC_INLINE uint32_t DL_GPIO_readPins(GPIO_Regs* gpio, uint32_t pins)   // Reads GPIO Pin
    __STATIC_INLINE void DL_GPIO_setPins(GPIO_Regs* gpio, uint32_t pins)        // Sets GPIO to Input
    __STATIC_INLINE void DL_GPIO_clearPins(GPIO_Regs* gpio, uint32_t pins)      // Sets GPIO to an Output
    __STATIC_INLINE void DL_GPIO_togglePins(GPIO_Regs* gpio, uint32_t pins)     // Flips pins in that registor mask
 *
 * @brief Returns the conversion result for the selected memory index
 *
 * @param[in] adc12     Pointer to the register overlay for the peripheral
 * @param[in] idx       Memory conversion index. @ref DL_ADC12_MEM_IDX.
 *
 * @return Conversion result
 *
__STATIC_INLINE uint16_t DL_ADC12_getMemResult(
    const ADC12_Regs *adc12, DL_ADC12_MEM_IDX idx)
{
    volatile const uint32_t *pReg = &adc12->ULLMEM.MEMRES[idx];

    return (uint16_t)(*(pReg + DL_ADC12_SVT_OFFSET));
}
*/

void getADCOut(System::GPIO::GPIO p_pin, int *p_volt) {
    *p_volt = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
}

//TODO: Make sure to fill this in
double READ_FS_POT() {
    int raw_fs_val = 0;
    getADCOut(FRONT_STEERING_POT_PIN, &raw_fs_val);
    double front_voltage = (raw_fs_val / 4095.0) * 3.3;
    double fs_angle = (front_voltage / 3.3) * 270.0 - 131.0;
    return fs_angle;
}

double READ_RS_POT() {
    int raw_rs_val = 0;
    getADCOut(REAR_STEERING_POT_PIN, &raw_rs_val);
    double rear_voltage = (raw_rs_val / 4095.0) * 3.3;
    double rs_angle = (rear_voltage / 3.3) * 270.0 - 131.0;
    return rs_angle;
}

#define INTEGRAL_LIMIT 100 // Adjust based on testing
double compute_pid(PID *pid, double error) {
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

double IdealRearAngle(double FS_SteeringAngle) {
    if (FS_SteeringAngle > deadband) { //Left Turn Steering Percentage Calc
        LT_Percentage = -50 * tanh(0.1 * (TR_RW_Coefficient * pow(FS_SteeringAngle, TR_RW_Power)) - 4.5) + 50;
        double RearAngle = (LT_Percentage / 100.0) * 135.0;
        RS_Deg = RearAngle;
        return RS_Deg;
    } else if (FS_SteeringAngle < -deadband) { // Right Turn Steering Percentage Calc
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

volatile bool gCheckADC;

int main() {
    System::init();

    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    /*** PWM config *******************************************/
    /* PA26 used as PWM output. driven by TIMER-1 C0
     */


    // setup PA26 as PWM output
    DL_GPIO_initPeripheralOutputFunctionFeatures(
            IOMUX_PINCM27,
            IOMUX_PINCM27_PF_TIMG1_CCP0,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_enableOutput(GPIOPINPUX(System::GPIO::PA26));

    // setup Timer-1 for PWM
    DL_Timer_enablePower(PWMTimer);
    delay_cycles(POWER_STARTUP_DELAY);
    {
        constexpr DL_Timer_ClockConfig clkConfig = {
                .clockSel      = DL_TIMER_CLOCK::DL_TIMER_CLOCK_BUSCLK,
                .divideRatio   = DL_TIMER_CLOCK_DIVIDE::DL_TIMER_CLOCK_DIVIDE_1,
                .prescale      = 0,
            };
        DL_Timer_setClockConfig(PWMTimer, &clkConfig);
    }
    {
        constexpr DL_Timer_PWMConfig pwmConfig = {
                .period     = PWMMAX,
                .pwmMode    = DL_TIMER_PWM_MODE::DL_TIMER_PWM_MODE_EDGE_ALIGN,
                .isTimerWithFourCC = false,
                .startTimer = DL_TIMER::DL_TIMER_START,
            };
        DL_Timer_initPWMMode(PWMTimer, &pwmConfig);
    }

    // PWM level triggers
    DL_Timer_setCounterControl(
            PWMTimer,
            DL_TIMER_CZC::DL_TIMER_CZC_CCCTL0_ZCOND,
            DL_TIMER_CAC::DL_TIMER_CAC_CCCTL0_ACOND,
            DL_TIMER_CLC::DL_TIMER_CLC_CCCTL0_LCOND
        );
    DL_Timer_setCaptureCompareOutCtl(
            PWMTimer,
            DL_TIMER_CC_OCTL_INIT_VAL_LOW,
            DL_TIMER_CC_OCTL_INV_OUT_ENABLED,
            DL_TIMER_CC_OCTL_SRC_FUNCVAL,
            DL_TIMER_CC_INDEX::DL_TIMER_CC_0_INDEX
        );
    DL_Timer_setCaptCompUpdateMethod(
            PWMTimer,
            DL_TIMER_CC_UPDATE_METHOD::DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
            DL_TIMER_CC_INDEX::DL_TIMER_CC_0_INDEX
        );

    setPWM(0);
    DL_Timer_enableClock(PWMTimer);
    DL_Timer_setCCPDirection(PWMTimer, DL_TIMER_CC0_OUTPUT);
    DL_Timer_startCounter(PWMTimer);

    //this gpio is going to be binded to the motor that steer the rear wheels
    auto &motor_dir = System::GPIO::PA23;
    DL_GPIO_initDigitalOutputFeatures(
        motor_dir.iomux,
        DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
    );
    DL_GPIO_clearPins(GPIOPINPUX(motor_dir));
    DL_GPIO_enableOutput(GPIOPINPUX(motor_dir));

    //TODO: add pin later for ac/dc for the something that is analog to digitl cause its on the car
    //Analog pins are PA1 - PA9
    auto &adc_o_A = System::GPIO::PA6; //accounts for the MA button
    DL_GPIO_initDigitalOutputFeatures(
        adc_o_A.iomux,
       DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
       DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
       DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
       DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
    );
    DL_GPIO_clearPins(GPIOPINPUX(adc_o_A));
    DL_GPIO_enableOutput(GPIOPINPUX(adc_o_A));

    auto &adc_o_B = System::GPIO::PA7; //accounts for the MB button
    DL_GPIO_initDigitalOutputFeatures(
        adc_o_B.iomux,
        DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
    );
    DL_GPIO_clearPins(GPIOPINPUX(adc_o_B));
    DL_GPIO_enableOutput(GPIOPINPUX(adc_o_B));

    setup_pid(&pid);


    auto &adc_fs_pot_pin = FRONT_STEERING_POT_PIN;
    DL_GPIO_initDigitalOutputFeatures(
            adc_fs_pot_pin.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(GPIOPINPUX(adc_fs_pot_pin));
    DL_GPIO_enableOutput(GPIOPINPUX(adc_fs_pot_pin));

    auto &adc_rs_pot_pin = REAR_STEERING_POT_PIN;
    DL_GPIO_initDigitalOutputFeatures(
            adc_rs_pot_pin.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(GPIOPINPUX(adc_rs_pot_pin));
    DL_GPIO_enableOutput(GPIOPINPUX(adc_rs_pot_pin));

    auto &adc_adjust_pot_pin = ADJUSTMENT_POT_PIN;
    DL_GPIO_initDigitalOutputFeatures(
            adc_adjust_pot_pin.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(GPIOPINPUX(adc_adjust_pot_pin));
    DL_GPIO_enableOutput(GPIOPINPUX(adc_adjust_pot_pin));

    double IDEAL_RS_ANGLE;
    double FS_SteeringAngle; //Input Front Steering Angle, set to -91 for testing the angles, whenever actually implemented this will not have a value
    double RS_SteeringAngle;        //Input Rear Steering Angle
    // defines values for PID and calculus

    DL_ADC12_configConversionMem(ADC12_0_INST,
                                 DL_ADC12_MEM_IDX_0,
                                 DL_ADC12_INPUT_CHAN_0,
                                 DL_ADC12_REFERENCE_VOLTAGE_VDDA,
                                 DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
                                 DL_ADC12_AVERAGING_MODE_DISABLED,
                                 DL_ADC12_BURN_OUT_SOURCE_DISABLED,
                                 DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT,
                                 DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    DL_ADC12_enableConversions(ADC12_0_INST);
    DL_ADC12_startConversion(ADC12_0_INST);
    delay_cycles(32e6);
    //while (!DL_ADC12_getStatus(ADC12_0_INST) & DL_ADC12_STATUS_CONVERSION_ACTIVE) {}
    uint16_t adcValue = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
    char str1[100];
    snprintf(ARRANDN(str1), "First: %6d\n", adcValue);
    System::uart_ui.nputs(ARRANDN(str1));
    gCheckADC = false;
    /**********************************************************/
/*
    while(1){
        DL_ADC12_startConversion(ADC12_0_INST);

        FS_SteeringAngle = READ_FS_POT();  // Front steering input
        RS_SteeringAngle = READ_RS_POT();  // Rear steering feedback

        // Calculate ideal rear steering angle
       IDEAL_RS_ANGLE = IdealRearAngle(FS_SteeringAngle);
       // Compute PID error (Difference between actual and ideal rear steering)
        double error = RS_SteeringAngle - IDEAL_RS_ANGLE;
        // Apply deadband
        if (fabs(error) < 1.2) {
            error = 0;
        }

        double pid_output = compute_pid(&pid, error);
        // Determine motor direction based on PID output
        if (pid_output > 0) {
            DL_GPIO_writePinsVal(motor_dir.port, DL_GPIO_PIN_0, 1); // Move forward
        } else {
            DL_GPIO_writePinsVal(motor_dir.port, DL_GPIO_PIN_0, 0); // Move backward
        }

        //if turns left "press" MA "Button"
        //NEED TO KNOW IF IDEAL_RS_ANGLE RETURNS NEGATIVE VALUES WHEN TURNING LEFT
        //IDEAL_RS_ANGLE < 0 or IDEAL_RS_ANGLE < 180 such and such...
        //inputting true for now so the code still runs
        // Determine motor direction based on PID output
        static double duty = 0;
        setPWM(PWMMAX * duty);

        delay_cycles(System::CLK::CPUCLK/20);

        // Convert PID output into a PWM duty cycle
        //int duty_cycle = duty_cycle_convert(pid_output); TODO: Consider defining this for LED (dunno what LED does)
        /*
        static double duty = 0;

        if(duty == 1){
            duty = 0;
        } else {'';
            duty = 1;
        }*/
    //}

    char str[100];
    int _value = 0;
    getADCOut(FRONT_STEERING_POT_PIN, &_value);
    snprintf(ARRANDN(str), "%6d\n", DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0));
    while(true) {
        System::uart_ui.nputs(ARRANDN(str));
        delay_cycles(32e6);
    }

    while(true) {
        System::FailHard("reached end of main" NEWLINE);
        delay_cycles(20e6);
    }
}

void ADC12_0_INST_IRQHandler(void) {
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            gCheckADC = true;
            break;
        default:
            break;
    }
}
