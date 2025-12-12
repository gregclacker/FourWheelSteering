/*
 * FWS_Utils.cpp
 *
 *  Created on: Nov 25, 2025
 *      Author: FSAE
 */

#include "system.hpp"
#include "fourwheelsteer_defs.hpp"

/* Four Wheel Steering Information
 * fs - Front Steer ADC Angle
 * Go forward when fs <= 180 or fs >= 0
 * Go backward when fs > 180 or fs < 0
 * */
void FWS_Utils::Motor::steer_motor() {
    double _fsangle = FWS_Utils::PID::fs_POT();
    if(_fsangle <= 180 || _fsangle >= 0) {
        motor_right();
    }
    if(_fsangle < 180 || _fsangle > 0) {
        motor_left();
    }
}

// 6 \ 10 >>> 9 / 7
//
//  10  7
//    \/
//    /\
//   9  6
//
void FWS_Utils::Motor::motor_right() {
    //Set Pins
    GateDriver1_PIN.set();
    GateDriver4_PIN.set();

}
void FWS_Utils::Motor::motor_left() {
    //Set Pins
    GateDriver2_PIN.set();
    GateDriver3_PIN.set();
}

/* ADC Information
 * 120 SPS Minimum ~ 12 Bit Resolution
 * 3.3 Vref
*/
constexpr double REFVOLTAGE = 3.3f;
constexpr double MAXBITS = 2048.0f; // Currently 12 bits unsigned
void FWS_Utils::ADC::get_ADC_raw(uint8_t p_adcAddr, int16_t *_raw) {
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
double FWS_Utils::ADC::get_ADC_voltage(uint8_t p_adcAddr) {
    int16_t _raw;
    FWS_Utils::ADC::get_ADC_raw(p_adcAddr, &_raw);
    return (_raw / MAXBITS) * REFVOLTAGE;
}

double FWS_Utils::PID::fs_POT() {
    int16_t raw_fs_val = 0;
    FWS_Utils::ADC::get_ADC_raw(ADC_1_ADDR, &raw_fs_val);
    double front_voltage = (raw_fs_val / MAXBITS) * REFVOLTAGE;
    double fs_angle = (front_voltage / REFVOLTAGE) * 270.0 - 131.0;
    return fs_angle;
}
double FWS_Utils::PID::rs_POT() {
    int16_t raw_rs_val = 0;
    FWS_Utils::ADC::get_ADC_raw(ADC_2_ADDR, &raw_rs_val);
    double rear_voltage = (raw_rs_val / MAXBITS) * REFVOLTAGE;
    double rs_angle = (rear_voltage / REFVOLTAGE) * 270.0 - 131.0;
    return rs_angle;
}

void FWS_Utils::PID::INIT_PID(PID *pid) {
    pid->Kp = FWS::KP;
    pid->Ki = FWS::KI;
    pid->Kd = FWS::KD;
    pid->previous_error = 0;
    pid->integral = 0;
}
#define INTEGRAL_LIMIT 100 // Adjust based on testing
double FWS_Utils::PID::caclulate_PID(PID *p_pid, double p_err) {
    double P_out = p_pid->Kp * p_err;

    // **Reset integral when error direction changes**
    if ((p_pid->previous_error > 0 && p_err < 0) || (p_pid->previous_error < 0 && p_err > 0)) {
        p_pid->integral = 0;
    } else {
        p_pid->integral += p_err;
    }

    // **Clamp integral to prevent windup**
    if (p_pid->integral > INTEGRAL_LIMIT) p_pid->integral = INTEGRAL_LIMIT;
    if (p_pid->integral < -INTEGRAL_LIMIT) p_pid->integral = -INTEGRAL_LIMIT;

    double I_out = p_pid->Ki * p_pid->integral;

    double derivative = (p_err - p_pid->previous_error);
    double D_out = p_pid->Kd * derivative;

    p_pid->previous_error = p_err;

    double output = P_out + I_out + D_out;

    // Debug Output
    /*printf("PID Debug - Error: %lf, P: %lf, I: %lf, D: %lf, PID Output: %lf\n",
           error, P_out, I_out, D_out, output);
    */
    return output;
}
double FWS_Utils::PID::target_rear_angle(double FS_SteeringAngle, FWS::FWS *fws) {
    if (fws->FS_SteeringAngle > fws->deadband) { //Left Turn Steering Percentage Calc
        fws->LT_Percentage = -50 * tanh(0.1 * (FWS::TR_RW_Coefficient * pow(fws->FS_SteeringAngle, FWS::TR_RW_Power)) - 4.5) + 50;  //Steering using hyperbolic tangent curve to get percentage of left turn
        double RearAngle = (fws->LT_Percentage / 100.0) * 135.0;
        fws->RS_Deg = RearAngle;
        return fws->RS_Deg;
    } else if (fws->FS_SteeringAngle < -fws->deadband) { // Right Turn Steering Percentage Calc
        fws->RT_Percentage = -50 * tanh(0.1 * (FWS::TR_LW_Coefficient * pow(-fws->FS_SteeringAngle, FWS::TR_LW_Power)) - 4.5) + 50;  //Steering using hyperbolic tangent curve to get percentage of right turn
        double RearAngle = (fws->RT_Percentage / 100.0) * 135.0;
        fws->RS_Deg = RearAngle * -1;
        return fws->RS_Deg;
    }
    fws->RS_Deg = 0.0;
    return fws->RS_Deg;
}

void FWS_Utils::PWM::INIT_TIMER(GPTIMER_Regs* p_timer) {
    DL_Timer_enablePower(p_timer);

    // setup Timer_x for PWM
    DL_Timer_enablePower(p_timer);

    delay_cycles(POWER_STARTUP_DELAY);
    constexpr DL_Timer_ClockConfig _clkConfig = {
            .clockSel      = DL_TIMER_CLOCK_BUSCLK,
            .divideRatio   = DL_TIMER_CLOCK_DIVIDE_1,
            .prescale      = 0,
        };
    DL_Timer_setClockConfig(p_timer, &_clkConfig);
    constexpr DL_Timer_PWMConfig _pwmConfig = {
            .period     = PWMMAX,
            .pwmMode    = DL_TIMER_PWM_MODE::DL_TIMER_PWM_MODE_EDGE_ALIGN,
            .isTimerWithFourCC = false,
            .startTimer = DL_TIMER::DL_TIMER_START,
        };

    DL_Timer_setCounterControl(
            p_timer,
            DL_TIMER_CZC::DL_TIMER_CZC_CCCTL0_ZCOND,
            DL_TIMER_CAC::DL_TIMER_CAC_CCCTL0_ACOND,
            DL_TIMER_CLC::DL_TIMER_CLC_CCCTL0_LCOND
        );
    DL_Timer_setClockConfig(p_timer, &_clkConfig);
    DL_Timer_initPWMMode(p_timer, &_pwmConfig);
    DL_Timer_enableClock(p_timer);
    DL_Timer_startCounter(p_timer);
}
void FWS_Utils::PWM::INIT_PWM_TIMER(PWM p_pwm) {

    for(int i = 0; i < p_pwm.count; i++)
        DL_GPIO_initPeripheralOutputFunctionFeatures(
                p_pwm.pins[i].iomux,
                p_pwm.iomuxes[i],
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
    DL_Timer_setCaptureCompareOutCtl(
            p_pwm.timer,
            DL_TIMER_CC_OCTL_INIT_VAL_LOW,
            DL_TIMER_CC_OCTL_INV_OUT_ENABLED,
            DL_TIMER_CC_OCTL_SRC_FUNCVAL,
            p_pwm.cc_index
        );
    DL_Timer_setCaptCompUpdateMethod(
            p_pwm.timer,
            DL_TIMER_CC_UPDATE_METHOD::DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
            p_pwm.cc_index
        );

    //FWS_Utils::PWM::set_PWM_duty(p_pwm, 0);
    //DL_Timer_setCCPDirection(p_pwm.timer, p_pwm.cc_output);
    uint32_t ccpd = DL_Timer_getCCPDirection(p_pwm.timer); // read current
    ccpd |= p_pwm.cc_output;                              // set only the desired bit
    DL_Timer_setCCPDirection(p_pwm.timer, ccpd);
}
uint32_t FWS_Utils::PWM::PWM_output(PWM p_pwm) {
    return DL_Timer_getCaptureCompareValue(p_pwm.timer, p_pwm.cc_index);
}
int FWS_Utils::PWM::PWM_duty(double pid_output) {
    int duty = (int)(fabs(pid_output) * (PWMMAX / 15)); // Adjusted scaling factor
    // Ensure duty cycle is within valid range
    if (duty > PWMMAX) duty = PWMMAX;
    if (duty > 0 && duty < 300) duty = 400;  // Minimum force to actually move

    return duty;
}
void FWS_Utils::PWM::set_PWM_duty(PWM p_pwm, double p_duty) {
    uint32_t _PWM_duty = PWMMAX * p_duty;
    if(_PWM_duty >= PWMMAX) _PWM_duty = PWMMAX - 1;
    if(_PWM_duty == 0) _PWM_duty = PWMMAX;
    DL_Timer_setCaptureCompareValue(p_pwm.timer, _PWM_duty, p_pwm.cc_index);
}
/*
 *
 */
void FWS_Utils::GPIO::INIT_GPIO(System::GPIO::GPIO p_gpio) {
    DL_GPIO_initDigitalOutput(p_gpio.iomux);
    DL_GPIO_enableOutput(GPIOPINPUX(p_gpio));
}

void FWS_Utils::FWS::INIT_FWS(FWS *fws) {
    fws->deadband = 0;
}
