/*
 * old_main.cpp
 *
 *  Created on: Nov 25, 2025
 *      Author: FSAE
 */

/*
 * PWM Signal will use the digital output to drive the anolog voltage. This voltage is scaled by the pwm timer
 * The H Bridge has 4 fets. These 4 fets are meant to be managed by 4 GPIO pins
 * Fets that are diagonally oppositly active to one another are in sync while the nearby fets are inversely active
 * Ensure the fets with the same terminal are not active with one another. If they are it will short
 * Once a pair of fets are disabled the other pair must be activated within some x amount of nanoseconds or the motor destroys itself
 */

#include <cstdio>

#include "system.hpp"

/*************************************************************/
GPTIMER_Regs * PWMTimer = TIMG1;
constexpr uint32_t PWMMAX = 0xFFFF;
/*
void setPWM(uint32_t val){
    if(val >= PWMMAX)
        val = PWMMAX - 1;

    if(val == 0)
        val = PWMMAX;

    DL_Timer_setCaptureCompareValue(PWMTimer, val, DL_TIMER_CC_INDEX::DL_TIMER_CC_0_INDEX);
}
uint32_t getPWM(){
    return DL_Timer_getCaptureCompareValue(PWMTimer, DL_TIMER_CC_INDEX::DL_TIMER_CC_0_INDEX);
}
*/
/*
 * Number of bytes to send from Controller to target.
 *  This example uses FIFO with polling, and the maximum FIFO size is 8.
 *  Refer to interrupt examples to handle larger packets
 */
#define I2C_TX_PACKET_SIZE (8)

/*
 * Number of bytes to received from target.
 *  This example uses FIFO with polling, and the maximum FIFO size is 8.
 *  Refer to interrupt examples to handle larger packets
 */
#define I2C_RX_PACKET_SIZE (5)

/* Data sent to the Target */
uint8_t gTxPacket[I2C_TX_PACKET_SIZE] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

/* Data received from Target */
volatile uint8_t gRxPacket[I2C_RX_PACKET_SIZE];

/* I2C Target address */
#define I2C_TARGET_ADDRESS (0x48)

System::GPIO::GPIO GPIO_LEDS_PORT = System::GPIO::PA1;
System::GPIO::GPIO GPIO_LEDS_USER_LED_1_PIN = System::GPIO::PA2;
//1st write: 10001000


/*************************************************************/

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
//int ADJUSTMENT_KNOB_VALUE;      //The value of the adjustment knob that will be used on calculations

#define ESP_OK 0
//PA1 - PA9
System::GPIO::GPIO GateDriver_1 = System::GPIO::PA6;
System::GPIO::GPIO GateDriver_2 = System::GPIO::PA7;

//System::GPIO::GPIO Dir_Pin = System::GPIO::PA0;
System::GPIO::GPIO PWM = System::GPIO::PA26;

#define ADC_1_ADDR            0x49
#define ADC_2_ADDR            0x50

//#define LED_PIN 2       //This is for the onboard LED (Status LED)
//#define FAN_PIN 23      //This is for the constant fan to cool the controller
//#define FRONT_STEERING_POT_PIN ADC2_CHANNEL_1 //This is the front steering pot       ADC1_CHANNEL_1 Corresponds to GPIO35 on the pinout diagram
//#define REAR_STEERING_POT_PIN ADC2_CHANNEL_2 //This is the rear steering pot        ADC1_CHANNEL_2 Corresponds to GPIO34 on the pinout diagram
//#define ADJUSTMENT_POT_PIN ADC1_CHANNEL_3 //This is the adjustment pot      ADC1_CHANNEL_3 Corresponds to GPIO32 on the pinout diagram
//#define MOTOR_DRIVER_1PIN  25      //This is the Motor Driver 1 pin correspodning to GPIO27
//#define MOTOR_DRIVER_2PIN 14        //This is the Motor Driver 2 pin corresponding to GPIO14
//#define MOTOR_PWM_PIN 18        //This is the PWM pin corresponding to GPIO12
//#define DIR_PIN 19
//#define ADJUSTMENT_AMOUNT 7     //The amount of clicks the potentiometer will have, this only needs to be adjusted right here

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

double Kp = 0.16;        //The Kp value of the PID  | How hard motor should push
double Ki = 0.000;       //The Ki value of the PID  | Helps push motor into error range (kinda like a counter ofrce to friction)
double Kd = 0.016;        //The Kd value of the PID | Dampens movements (kinda like shock absorber)

/*
 * How to tune Ki
 * Start with:
 * Set Ki = 0
 * Tune Kp until response is fast but not unstable
 * Slowly increase Ki until the system just removes remaining error without oscillating
 * Kp reacts to now. Ki reacts to the past. Kd reacts to the future.
 */

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

/* ADC Information
 * 120 SPS Minimum ~ 12 Bit Resolution
 * 3.3 Vref
*/
constexpr float REFVOLTAGE = 3.3f;
constexpr float MAXBITS = 2048.0f; // Currently 12 bits unsigned
void getADCOut(int8_t p_adcAddr, int16_t*_raw) {
    //ADC_1_ADDR
    uint8_t config = 0x80;       // example config byte | 1000000 Ready
    System::i2c1.tx(p_adcAddr, &config, 1);
    uint8_t _dataBuffer[2];
    System::i2c1.rx(p_adcAddr, _dataBuffer, 2);            // _dataBuffer contains 12 bits of data. It stores 7 bits in the first byte and 5 bits in the second
    if(!(_dataBuffer[1] & 0x80)) {
//        Explination for my smol brain
//        uint8_t _dataByte1 = buf[0] & 0x7F;       // Remove ready bit for data there are 7 bits of data
//        uint8_t _dataByte2 = (buf[1] >> 3);       // 3 bits aren't needed since 3 lowest bits are padding
//        *_raw = (_dataByte1 << 5) | _dataByte2;   // The 5 extra zeros leaves room for lsb to be combined.
        *_raw = (_dataBuffer[0] & 0x7F << 5) | (_dataBuffer[1] >> 3);
    }
}
// Eventually this will replace 'front_voltage'/'rear_voltage' just don't wanna replace it yet
float getADCVoltage(int8_t p_adcAddr) {
    int16_t _raw;
    getADCOut(p_adcAddr, &_raw);
    return (_raw / MAXBITS) * REFVOLTAGE;
}

double READ_FS_POT() {
    int16_t raw_fs_val = 0;
    getADCOut(ADC_1_ADDR, &raw_fs_val);
    double front_voltage = (raw_fs_val / MAXBITS) * REFVOLTAGE;
    double fs_angle = (front_voltage / REFVOLTAGE) * 270.0 - 131.0;
    return fs_angle;
}

double READ_RS_POT() {
    int16_t raw_rs_val = 0;
    getADCOut(ADC_2_ADDR, &raw_rs_val);
    double rear_voltage = (raw_rs_val / MAXBITS) * REFVOLTAGE;
    double rs_angle = (rear_voltage / REFVOLTAGE) * 270.0 - 131.0;
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
        LT_Percentage = -50 * tanh(0.1 * (TR_RW_Coefficient * pow(FS_SteeringAngle, TR_RW_Power)) - 4.5) + 50;  //Steering using hyperbolic tangent curve to get percentage of left turn
        double RearAngle = (LT_Percentage / 100.0) * 135.0;
        RS_Deg = RearAngle;
        return RS_Deg;
    } else if (FS_SteeringAngle < -deadband) { // Right Turn Steering Percentage Calc
        RT_Percentage = -50 * tanh(0.1 * (TR_LW_Coefficient * pow(-FS_SteeringAngle, TR_LW_Power)) - 4.5) + 50;  //Steering using hyperbolic tangent curve to get percentage of right turn
        double RearAngle = (RT_Percentage / 100.0) * 135.0;
        RS_Deg = RearAngle * -1 ;
        return RS_Deg;
    }
    {
        RS_Deg = 0.0;
        return RS_Deg;
    }
}

#include <ti/driverlib/driverlib.h>
int main(void) {
    System::init();

//    System::uart_ui.setBaudTarget(115200);
//    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    // clock
    {
        DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ::DL_SYSCTL_SYSOSC_FREQ_BASE); // SYSOSC 32Mhz
        DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER::DL_SYSCTL_MCLK_DIVIDER_DISABLE);
        DL_SYSCTL_enableMFCLK();
    }

    //BOR
    {
        // levels: 0:1V62, 1:2V23, 2:2V82, 3:2V95 . (see 7.6.1)
        // didn't notice anything about min-voltage for clock speeds
        DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL::DL_SYSCTL_BOR_THRESHOLD_LEVEL_1);
    }

    DL_GPIO_disablePower(GPIOA);
    DL_GPIO_reset(GPIOA);
    DL_GPIO_enablePower(GPIOA);
    delay_cycles(POWER_STARTUP_DELAY);


    DL_GPIO_initDigitalOutput(System::GPIO::PA22.iomux);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM22);
    DL_GPIO_initDigitalOutputFeatures(
                System::GPIO::PA22.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );

    {

        while(1){
            /*
            System::GPIO::PA3.set();
            System::GPIO::PA4.set();
            System::GPIO::PA10.set();
            System::GPIO::PA11.set();
            System::GPIO::PA22.set();
            delay_cycles(System::CLK::CPUCLK * 3);

            System::GPIO::PA3.clear();
            System::GPIO::PA4.clear();
            System::GPIO::PA10.clear();
            System::GPIO::PA11.clear();
            System::GPIO::PA22.clear();
            delay_cycles(System::CLK::CPUCLK * 3);
            */
//            DL_GPIO_togglePins(System::GPIO::PA3.port, System::GPIO::PA3.pin);
//            DL_GPIO_togglePins(System::GPIO::PA4.port, System::GPIO::PA4.pin);
//            DL_GPIO_togglePins(System::GPIO::PA10.port, System::GPIO::PA10.pin);
//            DL_GPIO_togglePins(System::GPIO::PA11.port, System::GPIO::PA11.pin);
            DL_GPIO_togglePins(System::GPIO::PA22.port, System::GPIO::PA22.pin);
            delay_cycles(System::CLK::CPUCLK/100);
        };
    }

    char str[100];
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
       DL_GPIO_enableOutput(GPIOPINPUX(PWM));


       // setup PA6 as
       DL_GPIO_initPeripheralOutputFunctionFeatures(
               IOMUX_PINCM27,
               IOMUX_PINCM27_PF_TIMG1_CCP0,
               DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
               DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
               DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
               DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
           );
       DL_GPIO_enableOutput(GPIOPINPUX(System::GPIO::PA6));
       // setup PA7 as
       DL_GPIO_initPeripheralOutputFunctionFeatures(
               IOMUX_PINCM27,
               IOMUX_PINCM27_PF_TIMG1_CCP0,
               DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
               DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
               DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
               DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
           );
       DL_GPIO_enableOutput(GPIOPINPUX(System::GPIO::PA7));



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
                   .period     = 0xFF,
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

       /**********************************************************/
       double voltage = 3.3;
       while(1){
           if(voltage > 0x7FF) {
               setPWM(voltage);
           }
           setPWM(PWMMAX);
           snprintf(ARRANDN(str), "PWM Out: %u\nVoltage: %f\n", getPWM(), voltage);
           System::uart_ui.nputs(ARRANDN(str));
       }
       delay_cycles(System::CLK::CPUCLK);

       while(true) {
           System::FailHard("reached end of main" NEWLINE);
           delay_cycles(20e6);
       }
   }
/*
while (1) {
    double voltage = 3.3;
    uint32_t pwm = DL_Timer_getCaptureCompareValue(PWMTimer, DL_TIMER_CC_INDEX::DL_TIMER_CC_0_INDEX);
    snprintf(ARRANDN(str), "PWM Out: %d; Voltage: %f\n", pwm, voltage);
    System::uart_ui.nputs(ARRANDN(str));

    if(voltage > 0x7FF) {
        System::uart_ui.nputs(ARRANDN("High\n"));
    } else {
        System::uart_ui.nputs(ARRANDN("Low\n"));
    }
    delay_cycles(32e6);

    //DL_ADC12_enableConversions(ADC12_0_INST);
}*/


/*

void setPWM2(uint32_t val){
    if(val >= 0xFFFF)
        val = 0xFFFF - 1;

    if(val == 0)
        val = 0xFFFF;

    DL_Timer_setCaptureCompareValue(PWM1.timer, val, PWM1.cc_index);
    DL_Timer_setCaptureCompareValue(PWM3.timer, val, PWM3.cc_index);
}

void tmp () {

    DL_GPIO_initPeripheralOutputFunctionFeatures(
                LED1_PIN.iomux,
                PWM1.iomuxes[0],
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
    DL_GPIO_initPeripheralOutputFunctionFeatures(
                LED2_PIN.iomux,
                PWM1.iomuxes[1],
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );

        // setup Timer-1 for PWM
        DL_Timer_enablePower(PWM1.timer);
        delay_cycles(POWER_STARTUP_DELAY);
        {
            constexpr DL_Timer_ClockConfig clkConfig = {
                    .clockSel      = DL_TIMER_CLOCK::DL_TIMER_CLOCK_BUSCLK,
                    .divideRatio   = DL_TIMER_CLOCK_DIVIDE::DL_TIMER_CLOCK_DIVIDE_1,
                    .prescale      = 0,
                };
            DL_Timer_setClockConfig(PWM1.timer, &clkConfig);
        }
        {
            constexpr DL_Timer_PWMConfig pwmConfig = {
                    .period     = 0xFFFF,
                    .pwmMode    = DL_TIMER_PWM_MODE::DL_TIMER_PWM_MODE_EDGE_ALIGN,
                    .isTimerWithFourCC = false,
                    .startTimer = DL_TIMER::DL_TIMER_START,
                };
            DL_Timer_initPWMMode(PWM1.timer, &pwmConfig);
        }

        // PWM level triggers
        DL_Timer_setCounterControl(
                PWM1.timer,
                DL_TIMER_CZC::DL_TIMER_CZC_CCCTL0_ZCOND,
                DL_TIMER_CAC::DL_TIMER_CAC_CCCTL0_ACOND,
                DL_TIMER_CLC::DL_TIMER_CLC_CCCTL0_LCOND
            );
        DL_Timer_setCaptureCompareOutCtl(
                PWM1.timer,
                DL_TIMER_CC_OCTL_INIT_VAL_LOW,
                DL_TIMER_CC_OCTL_INV_OUT_ENABLED,
                DL_TIMER_CC_OCTL_SRC_FUNCVAL,
                PWM1.cc_index
            );
        DL_Timer_setCaptCompUpdateMethod(
                PWM1.timer,
                DL_TIMER_CC_UPDATE_METHOD::DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
                PWM1.cc_index
            );

        setPWM2(0);
        DL_Timer_enableClock(PWM1.timer);
        DL_Timer_setCCPDirection(PWM1.timer, PWM1.cc_output);
        DL_Timer_startCounter(PWM1.timer);
}
*/



/*
 * DL_GPIO_initPeripheralOutputFunctionFeatures(
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
 */
