/*
 * pid_defs.hpp
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

#include <stdint.h>
#include <stdbool.h>

#include "system.hpp"

#ifndef system
#define system

/*************************************************************/

#define ESP_OK 0

//TIMG0_C0, TIMG0_C1, TIMG4_C0, TIMG4_C0
#define GateDriver1_PIN         System::GPIO::PA3
#define GateDriver2_PIN         System::GPIO::PA4

#define GateDriver3_PIN         System::GPIO::PA10
#define GateDriver4_PIN         System::GPIO::PA11

#define LED1_PIN                System::GPIO::PA0
#define LED2_PIN                System::GPIO::PA26

//#define Dir_Pin =             System::GPIO::PA0
#define PWM_Pin                 System::GPIO::PA26

#define PWMTIMER1_REG           TIMG0
#define PWMTIMER2_REG           TIMG2

#define ADC_1_ADDR              0x49 //TODO: NOT ACTUAL ADDRESS PLS FIX OR U WILL BE SAD FOR THE REST OF UR LIFE :( WOMP WOMP DO BETTER L CODE
#define ADC_2_ADDR              0x50

namespace System { namespace GPIO { } }

namespace FWS_Utils {

    namespace FWS {
        constexpr double WA_LW_Slope = 0.3117f;          //This is the slope of the left wheel, wheel angle vs turn radius
        constexpr double WA_LW_Intercept = -1.6651f;     //This is the Y intercept of the left wheel, wheel angle vs turn radius
        constexpr double WA_RW_Slope = 0.2286f;          //This is the slope of the right wheel, wheel angle vs turn radius
        constexpr double WA_RW_Intercept = -0.194f;      //This is the Y intercept of the right wheel, wheel angle vs turn radius
        constexpr double TR_LW_Coefficient = 2677.7f;    //This is the left wheel coefficient of the power function for Turn Radius vs Steering Angle
        constexpr double TR_LW_Power = -1.147f;          //This is the left wheel power of the power function for Turn Radius vs Steering Angle
        constexpr double TR_RW_Coefficient = 2760.0f;    //This is the right wheel coefficient of the power function for Turn Radius vs Steering Angle
        constexpr double TR_RW_Power = -1.222f;          //This is the right wheel power of the power function for Turn Radius vs Steering Angle

        constexpr double KP = 0.16;          //The Kp value of the PID  | How hard motor should push
        constexpr double KI = 0.000;         //The Ki value of the PID  | Helps push motor into error range (kinda like a counter ofrce to friction)
        constexpr double KD = 0.016;         //The Kd value of the PID | Dampens movements (kinda like shock absorber)

        typedef struct {
            double FS_SteeringAngle;        //Input Front Steering Angle, set to -91 for testing the angles, whenever actually implemented this will not have a value
            double RS_SteeringAngle;        //Input Rear Steering Angle
            double LW_Angle;                //Left Wheel Angle
            double RW_Angle;                //Right Wheel Angle
            double TR_Left;                 //Turn Radius Left Wheel Right Turn
            double TR_Right;                //Turn Radius Right Wheel Left Turn
            double IDEAL_RS_ANGLE;
            double RS_TR_RIGHT;
            double RS_WA_RIGHT;
            double RS_TR_LEFT;
            double RS_WA_LEFT;
            double RT_Percentage;
            double LT_Percentage;
            double RS_Deg;
            double deadband;
            //double Gain_Input;    //Not sure if this will be used
            //The amount of 'clicks' you want to be able to turn the rear steering adjustment knob
            int ADJUSTMENT_KNOB_VALUE;      //The value of the adjustment knob that will be used on calculations
        } FWS;

        void INITFWS(FWS*);
    }

    namespace Motor {
        // Steers the motor
        void steer_motor();
        // Turns the motor right through the H-bridge
        void motor_right();
        // Turns the motor left through the H-bridge
        void motor_left();
    }

    namespace ADC {
        // Gets the raw binary voltage data over I2c
        void get_ADC_raw(uint8_t, int16_t*);
        // Gets the adc voltage over I2c
        double get_ADC_voltage(uint8_t);
    }

    namespace PID {
        /*
         * How to tune Ki
         * Start with:
         * Set Ki = 0
         * Tune Kp until response is fast but not unstable
         * Slowly increase Ki until the system just removes remaining error without oscillating
         * Kp reacts to now. Ki reacts to the past. Kd reacts to the future.
         */
        typedef struct {
            double Kp;
            double Ki;
            double Kd;
            double previous_error;
            double integral;
        } PID;

        void INITPID(PID*);

        // Front angle
        double fs_POT();
        // Rear angle
        double rs_POT();
        // Calculates PID
        double caclulate_PID(PID*, double);
        // Gets the target rear angle
        double target_rear_angle(double, FWS::FWS*);
    }

    namespace PWM {
        constexpr uint32_t PWMMAX = 0xFFFF;
        /*
         * Groups all our needed timer functionality
         */
        typedef struct {
            GPTIMER_Regs *timer;            // Timer register
            DL_TIMER_CC_INDEX cc_index;     // CC index
            const uint32_t cc_output;             // CC Output index
            const  System::GPIO::GPIO *pins;       // Timer Pins
            const uint32_t *iomuxes;              // Timer IOMux
            const buffsize_t count;              // Timer IOMux
        } PWM;

        // Inits basic pwm functionality
        void INITPWMTIMER(PWM);

        // Gets PWM output
        uint32_t PWM_output(PWM);
        // Sets get PWM duty
        int PWM_duty(double);
        // Sets PWM duty
        void set_PWM_duty(PWM, uint32_t);
    }
//uint32_t, DL_TIMER_CC_INDEX
    namespace GPIO {
        // Inits basic GPIO functionality
        void INITGPIO(PWM::PWM, buffsize_t);
    }
}

#endif
