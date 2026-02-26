/*
 * fourwheelsteer_defs.hpp
 *
 *  Created on: Nov 25, 2025
 *      Author: FSAE
 *      Drived From: https://github.com/gregclacker/FourWheelSteering/tree/main/original%20VSC%20project
 *      Contributer: Marcus Turley, Abdul-Muizz Abgoola
 */

#include <stdint.h>
#include <stdbool.h>

#include "system.hpp"

#ifndef system
#define system

/*************************************************************/

#define ESP_OK 0

//TIMG0_C0, TIMG0_C1, TIMG4_C0, TIMG4_C0
#define GATE_PIN_1         System::GPIO::PA3
#define GATE_PIN_2         System::GPIO::PA4

#define GATE_PIN_3         System::GPIO::PA10
#define GATE_PIN_4         System::GPIO::PA11

#define Output_1         System::GPIO::PA17
#define Output_2         System::GPIO::PA18
#define Output_3         System::GPIO::PA21
#define Output_4         System::GPIO::PA24

#define TIMERSYNC_REG           TIMG0
#define TIMER_REG_1             TIMG2
#define TIMER_REG_2             TIMG4

#define ADC_1_ADDR              0x49 // TODO: Update this address to match adc
#define ADC_2_ADDR              0x50

namespace System { namespace GPIO { } }

namespace FWS_Utils {

    /* NB - All this stuff is derived from the FourwheelSteering github we forked:
     * The github contains helpful front and rear wheel steer math for angles
     */
    namespace FWS {

        typedef struct {
            double WA_LW_Slope;             //This is the slope of the left wheel, wheel angle vs turn radius
            double WA_LW_Intercept;         //This is the Y intercept of the left wheel, wheel angle vs turn radius
            double WA_RW_Slope;             //This is the slope of the right wheel, wheel angle vs turn radius
            double WA_RW_Intercept;         //This is the Y intercept of the right wheel, wheel angle vs turn radius
            double TR_LW_Coefficient;       //This is the left wheel coefficient of the power function for Turn Radius vs Steering Angle
            double TR_LW_Power;             //This is the left wheel power of the power function for Turn Radius vs Steering Angle
            double TR_RW_Coefficient;       //This is the right wheel coefficient of the power function for Turn Radius vs Steering Angle
            double TR_RW_Power;             //This is the right wheel power of the power function for Turn Radius vs Steering Angle

            double KP;                      //The Kp value of the PID  | How hard motor should push
            double KI;                      //The Ki value of the PID  | Helps push motor into error range (kinda like a counter ofrce to friction)
            double KD;                      //The Kd value of the PID | Dampens movements (kinda like shock absorber)

            double sensor_max_angle;
            double sensor_angle_offset;

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

        // Basic init for FWS struct
        void INIT_FWS(FWS*);
    }

    /* NB - All this stuff is derived from the FourwheelSteering github we forked:
     * The github contains helpful front and rear wheel steer math for angles
     */
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
            double kp;
            double ki;
            double kd;
            double previous_error;
            double integral;
        } PID;

        void INIT_PID(PID*, FWS::FWS*);                 // Basic init for PID struct
        double fs_pot(FWS::FWS*);                       // Front angle
        double rs_pot(FWS::FWS*);                       // Rear angle
        double caclulate_pid(PID*, double);             // Calculates PID
        double target_rear_angle(double, FWS::FWS*);    // Gets the target rear angle
    }

    /* Namespace containing general timer/pwm information
     * The general idea is to have a group of pwms contained within the 'pwms' variable for access
     * Make sure you properly set the 'pwm_count' so functions execute properly
     */
    namespace PWM {
        constexpr uint32_t PWMMAX = 0xFFFF;
        /*
         * Groups all our needed timer functionality
         */
        typedef struct {
            GPTIMER_Regs *timer;                // Timer register
            DL_TIMER_CC_INDEX cc_index;         // CC index
            const uint32_t cc_output;           // CC Output index
            const System::GPIO::GPIO pin;       // Timer Pin
            const uint32_t iomux;               // Timer IOmux
            uint32_t phase;
        } PWM;
        extern const PWM PWM_GATE_1;
        extern const PWM PWM_GATE_2;
        extern const PWM PWM_GATE_3;
        extern const PWM PWM_GATE_4;
        extern const PWM PWM_GATE_5;            // Remove this or remove it (probably forgot to remove)
        typedef struct {
            GPTIMER_Regs *timer;                // Timer register
            const PWM *pwms;                    // pwms
            const buffsize_t pwm_count;         // PWM Count
        } TIMER;
        extern const TIMER PWM_TIMER_1;
        extern const TIMER PWM_TIMER_2;

        void INIT_TIMER_BASIC(TIMER,uint32_t);  // Inits basic individual timer functionality
        void INIT_TIMER_MASTER(GPTIMER_Regs*);  // Inits basic master timer functionality through cross trigger
        void INIT_TIMER_SLAVE(TIMER,uint32_t);  // Inits basic slave timer functionality through cross trigger

        void INIT_PWM_OUTPUTS(TIMER,bool,bool);
        void INIT_PWM_OUTPUT_SLAVE(PWM,bool);
        void INIT_PWM_OUTPUT_BASIC(PWM,bool);

        void start_timers(GPTIMER_Regs**, buffsize_t, bool);
        void start_timer(GPTIMER_Regs*, bool);
        void stop_timers(GPTIMER_Regs**, buffsize_t);
        void stop_timer(GPTIMER_Regs*);

        uint32_t get_counter_output(GPTIMER_Regs *);
        uint32_t get_counter_value(const PWM*);

        uint32_t get_ccv(const PWM*);
        double get_duty(const PWM*);
        uint32_t get_ccv_phase(const PWM *PWM);

        uint32_t get_ccv_max(const PWM*);
        uint32_t get_ccr_output(const PWM*);    // Gets PWM output raw

        uint32_t pwm_duty_to_ccv(double);
        double pwm_ccv_to_ratio(uint32_t);

        void set_duty(const PWM*, double);      // Sets PWM duty
        void set_phase(PWM *PWM, double);
    }

    /* Namespace containing motor functionality
     */
    namespace Motor {
        void steer_motor(const PWM::PWM* const*, buffsize_t, FWS::FWS*);        // Steers the motor
        void motor_forward(const PWM::PWM* const*, buffsize_t, FWS::FWS*);      // Turns the motor right through the H-bridge
        void motor_backward(const PWM::PWM* const*, buffsize_t, FWS::FWS*);     // Turns the motor left through the H-bridge
        double steer_error(FWS::FWS*);                                          // Error move
    }

    /* Namespace containing adc functionality
     */
    namespace ADC {
        const bool use_external = false;        // Gets the raw binary voltage data over I2c
        void get_adc_raw(uint8_t, int16_t*);    // Gets raw the adc voltage data over I2c
        double get_adc_voltage(uint8_t);        // Gets the adc voltage over I2c
    }

    /* Namespace containing any gpio functionality
     */
    namespace GPIO {
        void INIT_GPIO(System::GPIO::GPIO);     // Inits basic GPIO functionality
    }
}

#endif
