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


int main(){
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

    double IDEAL_RS_ANGLE;
    double FS_SteeringAngle; //Input Front Steering Angle, set to -91 for testing the angles, whenever actually implemented this will not have a value
    double RS_SteeringAngle;        //Input Rear Steering Angle
    // defines values for PID and calculus
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

    //TODO: Make sure to fill this in
    double READ_FS_POT()
    {

    }
    double READ_RS_POT()
      {

      }
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
            // Math to find the Ideal angle under the intergral math things
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
                //compute pid function for things
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
            // Debug Output
            /*printf("PID Debug - Error: %lf, P: %lf, I: %lf, D: %lf, PID Output: %lf\n",
                   error, P_out, I_out, D_out, output);
            */
            return output;
        }

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
        auto &adc_o = System::GPIO::PA7;
               DL_GPIO_initDigitalOutputFeatures(
                       adc_o.iomux,
                       DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                       DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                       DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                       DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                   );
               DL_GPIO_clearPins(GPIOPINPUX(adc_o));
               DL_GPIO_enableOutput(GPIOPINPUX(adc_o));
               setup_pid(&pid);

    /**********************************************************/

    while(1){
        FS_SteeringAngle = READ_FS_POT();  // Front steering input
        RS_SteeringAngle = READ_RS_POT();  // Rear steering feedback

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
          //int duty_cycle = duty_cycle_convert(pid_output); TODO: Consider defining this for LED (dunno what LED does)
        /*
        static double duty = 0;

        if(duty == 1){
            duty = 0;
        } else {
            duty = 1;
        }*/
        setPWM(PWMMAX * duty);

        delay_cycles(System::CLK::CPUCLK/20);
    }

    while(true) {
        System::FailHard("reached end of main" NEWLINE);
        delay_cycles(20e6);
    }
}
