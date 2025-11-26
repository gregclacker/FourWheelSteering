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

#ifndef SRC_CORE_PID_DEFS_HPP_
#define SRC_CORE_PID_DEFS_HPP_

/*************************************************************/

#define ESP_OK 0

//PA1 - PA9
#define GateDriver_1 System::GPIO::PA6
#define GateDriver_2 System::GPIO::PA7

//#define Dir_Pin = System::GPIO::PA0
#define PWM System::GPIO::PA26

#define ADC_1_ADDR            0x49
#define ADC_2_ADDR            0x50

#define TURNRADIUSCALC_H

#define WA_LW_Slope 0.3117f     //This is the slope of the left wheel, wheel angle vs turn radius
#define WA_LW_Intercept -1.6651f     //This is the Y intercept of the left wheel, wheel angle vs turn radius
#define WA_RW_Slope 0.2286f     //This is the slope of the right wheel, wheel angle vs turn radius
#define WA_RW_Intercept -0.194f     //This is the Y intercept of the right wheel, wheel angle vs turn radius
#define TR_LW_Coefficient 2677.7f  //This is the left wheel coefficient of the power function for Turn Radius vs Steering Angle
#define TR_LW_Power -1.147f    //This is the left wheel power of the power function for Turn Radius vs Steering Angle
#define TR_RW_Coefficient 2760.0f  //This is the right wheel coefficient of the power function for Turn Radius vs Steering Angle
#define TR_RW_Power -1.222f    //This is the right wheel power of the power function for Turn Radius vs Steering Angle

#define KP 0.16        //The Kp value of the PID  | How hard motor should push
#define KI 0.000       //The Ki value of the PID  | Helps push motor into error range (kinda like a counter ofrce to friction)
#define KD 0.016        //The Kd value of the PID | Dampens movements (kinda like shock absorber)

namespace FWS_Utils {

    typedef struct {
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
        double deadband;
        //double Gain_Input;    //Not sure if this will be used
        //The amount of 'clicks' you want to be able to turn the rear steering adjustment knob
        int ADJUSTMENT_KNOB_VALUE;      //The value of the adjustment knob that will be used on calculations
    } FWS;

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

    void SteerMotor();
    void MotorForward();
    void MotorReverse();

    void getADCOut(uint8_t, int16_t*);
    float getADCVoltage(uint8_t);

    double READ_FS_POT();
    double READ_RS_POT();
    double compute_pid(PID*, double);
    double IdealRearAngle(double,FWS*);

    void SetupFWS(FWS*);
    void SetupPID(PID*);

    #endif /* SRC_CORE_PID_DEFS_HPP_ */
}
