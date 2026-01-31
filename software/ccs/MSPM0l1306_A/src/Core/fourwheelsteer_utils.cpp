/*
 * FWS_Utils.cpp
 *
 *  Created on: Nov 25, 2025
 *      Author: FSAE
 */

#include "system.hpp"
#include "fourwheelsteer_defs.hpp"
#include <cmath>
/* ADC Information
 * 120 SPS Minimum ~ 12 Bit Resolution
 * 3.3 Vref
*/
constexpr double REFVOLTAGE = 3.3f;
constexpr double MAXBITS = 4095.0f; // Currently 12 bits unsigned
void FWS_Utils::ADC::get_adc_raw(uint8_t p_adcAddr, int16_t *_raw) {
//    *_raw = MAXBITS;
    if(use_external) {
       uint8_t config = 0x80;   // example: start conversion
       System::i2c1.tx(p_adcAddr, &config, 1);

       uint8_t buf[2];
       System::i2c1.rx(p_adcAddr, buf, 2);

       // READY bit = bit7 of byte1, 0 = ready
       if ((buf[1] & 0x80) == 0) {
           int16_t value =
               ((int16_t)(buf[0] & 0x7F) << 5) |
               ((int16_t)(buf[1] >> 3));

           // If signed 12-bit ADC, uncomment:
           // if (value & 0x0800) value |= 0xF000;

           *_raw = value;
       }
    }
    /*
    if(use_external) {
        uint8_t config = 0x80;       // example config byte | 1000000 Ready
        System::i2c1.tx(p_adcAddr, &config, 1);
        uint8_t _dataBuffer[2];
        System::i2c1.rx(p_adcAddr, _dataBuffer, 2);            // _dataBuffer contains 12 bits of data. It stores 7 bits in the first byte and 5 bits in the second
        if (value & 0x0800) {   // bit 11 set?
            value |= 0xF000;    // sign extend
        }
        if(!(_dataBuffer[1] & 0x80)) {
//            Explination for my smol brain
//            uint8_t _dataByte1 = buf[0] & 0x7F;       // Remove ready bit for data there are 7 bits of data
//            uint8_t _dataByte2 = (buf[1] >> 3);       // 3 bits aren't needed since 3 lowest bits are padding
//            *_raw = (_dataByte1 << 5) | _dataByte2;   // The 5 extra zeros leaves room for lsb to be combined.
            *_raw = ((int16_t)_dataBuffer[0] & 0x7F << 5)
                    | ((int16_t)_dataBuffer[1] >> 3);
        }
    } else {

    }*/
}
// Eventually this will replace 'front_voltage'/'rear_voltage' just don't wanna replace it yet
double FWS_Utils::ADC::get_adc_voltage(uint8_t p_adcAddr) {
    int16_t _raw;
    FWS_Utils::ADC::get_adc_raw(p_adcAddr, &_raw);
    return (_raw / MAXBITS) * REFVOLTAGE;
}

double FWS_Utils::PID::fs_POT(FWS::FWS* p_fws) {
    int16_t raw_fs_val = 0;
    FWS_Utils::ADC::get_adc_raw(ADC_1_ADDR, &raw_fs_val);
    double front_voltage = ((double)(raw_fs_val) / MAXBITS) * REFVOLTAGE;
    double fs_angle = (front_voltage / REFVOLTAGE) * p_fws->sensor_max_angle - p_fws->sensor_angle_offset;
    return fs_angle;
}
double FWS_Utils::PID::rs_POT(FWS::FWS* p_fws) {
    int16_t raw_rs_val = 0;
    FWS_Utils::ADC::get_adc_raw(ADC_2_ADDR, &raw_rs_val);
    double rear_voltage = (raw_rs_val / MAXBITS) * REFVOLTAGE;
    double rs_angle = (rear_voltage / REFVOLTAGE) * p_fws->sensor_max_angle - p_fws->sensor_angle_offset;
    return rs_angle;
}

void FWS_Utils::PID::INIT_PID(PID *p_pid, FWS::FWS *p_fws) {
    p_pid->Kp = p_fws->KP;
    p_pid->Ki = p_fws->KI;
    p_pid->Kd = p_fws->KD;
    p_pid->previous_error = 0;
    p_pid->integral = 0;
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
double FWS_Utils::PID::target_rear_angle(double FS_SteeringAngle, FWS::FWS *p_fws) {
    if (p_fws->FS_SteeringAngle > p_fws->deadband) { //Left Turn Steering Percentage Calc
        p_fws->LT_Percentage = -50 * tanh(0.1 * (p_fws->TR_RW_Coefficient * pow(p_fws->FS_SteeringAngle, p_fws->TR_RW_Power)) - 4.5) + 50;  //Steering using hyperbolic tangent curve to get percentage of left turn
        double RearAngle = (p_fws->LT_Percentage / 100.0) * 135.0;
        p_fws->RS_Deg = RearAngle;
        return p_fws->RS_Deg;
    } else if (p_fws->FS_SteeringAngle < -p_fws->deadband) { // Right Turn Steering Percentage Calc
        p_fws->RT_Percentage = -50 * tanh(0.1 * (p_fws->TR_LW_Coefficient * pow(-p_fws->FS_SteeringAngle, p_fws->TR_LW_Power)) - 4.5) + 50;  //Steering using hyperbolic tangent curve to get percentage of right turn
        double RearAngle = (p_fws->RT_Percentage / 100.0) * 135.0;
        p_fws->RS_Deg = RearAngle * -1;
        return p_fws->RS_Deg;
    }
    p_fws->RS_Deg = 0.0;
    return p_fws->RS_Deg;
}

void FWS_Utils::PWM::INIT_TIMER_DSYNC(GPTIMER_Regs *p_timer) {
    DL_Timer_enablePower(p_timer);
    delay_cycles(POWER_STARTUP_DELAY);

    constexpr DL_Timer_ClockConfig _clkConfig = {
            .clockSel           = DL_TIMER_CLOCK_BUSCLK,
            .divideRatio        = DL_TIMER_CLOCK_DIVIDE_1,
            .prescale            = 0,
        };
    constexpr DL_Timer_PWMConfig _pwmConfig = {
            .period             = PWMMAX,
            .pwmMode            = DL_TIMER_PWM_MODE::DL_TIMER_PWM_MODE_EDGE_ALIGN,
            .isTimerWithFourCC  = true,
            .startTimer         = DL_TIMER::DL_TIMER_STOP,
        };
    // Load refers to when the preiod is hit
    constexpr DL_Timer_CaptureConfig _captureConfig = {
           .captureMode         = DL_TIMER_CAPTURE_MODE_EDGE_TIME,
           .period              = PWMMAX,
           .startTimer          = DL_TIMER_START,
           .edgeCaptMode        = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_RISING,
           .inputChan           = DL_TIMER_INPUT_CHAN_0,
           .inputInvMode        = DL_TIMER_CC_INPUT_INV_NOINVERT,
        };

    DL_Timer_setClockConfig(p_timer, &_clkConfig);
    DL_Timer_initCaptureMode(p_timer, &_captureConfig);
    DL_Timer_setClockConfig(p_timer, &_clkConfig);
    DL_Timer_initPWMMode(p_timer, &_pwmConfig);
    DL_Timer_enableClock(p_timer);
}

constexpr DL_Timer_ClockConfig clkCfg = {
            .clockSel    = DL_TIMER_CLOCK_BUSCLK,
            .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
            .prescale    = 0,
        };
constexpr DL_Timer_PWMConfig pwmCfg = {
        .period            = FWS_Utils::PWM::PWMMAX,
        .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN, //DL_TIMER_PWM_MODE_CENTER_ALIGN
        .isTimerWithFourCC = true,
        .startTimer        = DL_TIMER_STOP,
    };
void FWS_Utils::PWM::INIT_TIMER_SYNC(GPTIMER_Regs *p_timer) {
    DL_Timer_enablePower(p_timer);
    DL_Timer_setClockConfig(p_timer, &clkCfg);
    DL_Timer_initPWMMode(p_timer, &pwmCfg);
    DL_Timer_enableClock(p_timer);

    DL_Timer_configCrossTrigger(
        p_timer,
        DL_TIMER_CROSS_TRIG_SRC_ZERO,           // emit FSUB0 on ZERO
        DL_TIMER_CROSS_TRIGGER_INPUT_DISABLED,
        DL_TIMER_CROSS_TRIGGER_MODE_ENABLED
    );
    DL_Timer_configCrossTriggerEnable(
        p_timer,
        DL_TIMER_CROSS_TRIGGER_MODE_ENABLED);

    DL_Timer_startCounter(p_timer);
    DL_Timer_generateCrossTrigger(p_timer);
}
void FWS_Utils::PWM::INIT_TIMER_PWM(TIMER p_timer, uint32_t p_phase) {
    GPTIMER_Regs* timer = p_timer.timer;
    DL_Timer_enablePower(timer);
    DL_Timer_setClockConfig(timer, &clkCfg);
    DL_Timer_initPWMMode(timer, &pwmCfg);

    for(int i = 0; i < p_timer.pwm_count; i++) {
        for(int j = 0; j < p_timer.pwm_count; j++) {
            DL_GPIO_initPeripheralOutputFunctionFeatures(
                    p_timer.pwms[i].pin.iomux,
                    p_timer.pwms[i].iomux,
                    DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                    DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                    DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                    DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                );
//            DL_Timer_setCaptCompUpdateMethod(timer, DL_TIMER_CC_UPDATE_METHOD_TRIG_EVT, p_timer.pwms[i].cc_index);
//            DL_Timer_setCaptureCompareOutCtl(timer, DL_TIMER_CC_OCTL_INIT_VAL_LOW, DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL, p_timer.pwms[i].cc_index);
//          DL_TimerG_setCaptureCompareValue(timer, 500, p_timer.pwms[i].cc_index);
        }
    }

    DL_Timer_configCrossTrigger(
        timer,
        DL_TIMER_CROSS_TRIG_SRC_FSUB0,
        DL_TIMER_CROSS_TRIGGER_INPUT_ENABLED,
        DL_TIMER_CROSS_TRIGGER_MODE_ENABLED
    );
//    DL_Timer_configCrossTriggerEnable(
//        timer,
//        DL_TIMER_CROSS_TRIGGER_MODE_ENABLED
//        );

//    DL_Timer_setExternalTriggerEvent(timer,
//        DL_TIMER_EXT_TRIG_SEL_TRIG_SUB_0);

    DL_Timer_setCounterControl(
        timer,
        DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND
    );

    DL_Timer_enablePhaseLoad(timer);
    DL_Timer_setPhaseLoadValue(timer, p_phase);   // 0 = symmetric

    DL_Timer_enableClock(timer);

    DL_Timer_startCounter(timer);
}
void FWS_Utils::PWM::INIT_PWM_OUTPUTS(TIMER p_timer, bool p_invert)  {
    for(int i = 0; i < p_timer.pwm_count; i++)
        INIT_PWM_OUTPUT(p_timer.pwms[i], p_invert);
}
void FWS_Utils::PWM::INIT_PWM_OUTPUT(PWM p_pwm, bool p_invert)  {
    GPTIMER_Regs* timer = p_pwm.timer;
    DL_Timer_setCaptureCompareOutCtl(
            timer,
            DL_TIMER_CC_OCTL_INIT_VAL_LOW,
            p_invert ?
            DL_TIMER_CC_OCTL_INV_OUT_ENABLED
            : DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
            DL_TIMER_CC_OCTL_SRC_FUNCVAL,
            p_pwm.cc_index
        );
    DL_Timer_setCCPDirection(timer,
         DL_Timer_getCCPDirection(timer) | p_pwm.cc_output);
//    FWS_Utils::PWM::set_PWM_duty(&p_pwm, 0);
}

void FWS_Utils::PWM::start_timers(GPTIMER_Regs **p_timer, buffsize_t p_size) {
    for(int i = 0; i < p_size; i++)
        start_timer(p_timer[i]);
}
void FWS_Utils::PWM::start_timer(GPTIMER_Regs *p_timer) {
//    DL_Timer_setTimerCount(p_timer, PWMMAX);
    DL_Timer_startCounter(p_timer);
}
void FWS_Utils::PWM::stop_timers(GPTIMER_Regs **p_timer, buffsize_t p_size) {
    for(int i = 0; i < sizeof(p_timer[0])/sizeof(p_timer); i++)
        stop_timer(p_timer[i]);
}
void FWS_Utils::PWM::stop_timer(GPTIMER_Regs *p_timer) {
    DL_Timer_stopCounter(p_timer);
}
uint32_t FWS_Utils::PWM::PWM_output(const PWM *p_pwm) {
    return DL_Timer_getCaptureCompareValue(p_pwm->timer, p_pwm->cc_index);
}
int FWS_Utils::PWM::PWM_duty(double pid_output) {
    int duty = (int)(fabs(pid_output) * (PWMMAX / 15)); // Adjusted scaling factor
    // Ensure duty cycle is within valid range
    if (duty > PWMMAX) duty = PWMMAX;
    if (duty > 0 && duty < 300) duty = 400;  // Minimum force to actually move

    return duty;
}
void FWS_Utils::PWM::set_PWM_duty(const PWM *p_pwm, double p_duty) {
    uint32_t _ccr = PWMMAX * (1 - p_duty);
    if(_ccr >= PWMMAX) _ccr = PWMMAX - 1;
    DL_Timer_setCaptureCompareValue(p_pwm->timer, _ccr, p_pwm->cc_index);
}


// 10 \ 3 (Forward) >>> 11 / 4 (Backward)
//
//  10  4
//    \/
//    /\
//  11  3
//
void FWS_Utils::Motor::steer_motor(const PWM::PWM* const* p_pwms, buffsize_t count, FWS::FWS *p_fws) {
    if(steer_error(p_fws) < 0 && motor_move == 0 || motor_move == 1)
        motor_forward(p_pwms, count, p_fws);
//    else if(steer_error(p_fws) >= 0 && motor_move == 0 || motor_move == 2)
//        motor_backward(p_pwms, count, p_fws);
}
void FWS_Utils::Motor::motor_forward(const PWM::PWM* const* p_pwms, buffsize_t count, FWS::FWS *p_fws) {
//    delay_cycles(System::CLK::MCLK / 1000 / 2);         // Wait 500ns before turning on other pair
    delay_cycles(System::CLK::MCLK / 2);         // Wait 500ns before turning on other pair
//  double _speed = std::abs(steer_error(p_fws))
    double _speed = 0.9;

//    PWM::set_PWM_duty(p_pwms[0], _speed);
//    PWM::set_PWM_duty(p_pwms[2], _speed);
//    PWM::set_PWM_duty(p_pwms[1], 1 - _speed);
//    PWM::set_PWM_duty(p_pwms[3], 1 - _speed);
    double _length = 0.0;
    PWM::set_PWM_duty(p_pwms[0], _speed);
    PWM::set_PWM_duty(p_pwms[2], _speed);
    PWM::set_PWM_duty(p_pwms[1], _speed - _length);
    PWM::set_PWM_duty(p_pwms[3], _speed - _length);
}
void FWS_Utils::Motor::motor_backward(const PWM::PWM* const* p_pwms, buffsize_t count, FWS::FWS *p_fws) {
    PWM::set_PWM_duty(p_pwms[0], 0.0);
    PWM::set_PWM_duty(p_pwms[2], 0.0);
    delay_cycles(System::CLK::MCLK / 2);         // Wait 500ns before turning on other pair

    double speed = std::abs(steer_error(p_fws));

    PWM::set_PWM_duty(p_pwms[1], speed);
    PWM::set_PWM_duty(p_pwms[3], speed);
}
double FWS_Utils::Motor::steer_error(FWS::FWS *p_fws) {
    return (PID::rs_POT(p_fws) - (p_fws->sensor_max_angle - PID::fs_POT(p_fws))) / p_fws->sensor_max_angle;
//    return (PID::rs_POT(p_fws) - (p_fws->sensor_max_angle - PID::fs_POT(p_fws))) / p_fws->sensor_max_angle;
}

void FWS_Utils::GPIO::INIT_GPIO(System::GPIO::GPIO p_gpio) {
    DL_GPIO_initDigitalOutput(p_gpio.iomux);
    DL_GPIO_enableOutput(GPIOPINPUX(p_gpio));
}

void FWS_Utils::FWS::INIT_FWS(FWS *fws) {
    fws->WA_LW_Slope = 0.3117f;
    fws->WA_LW_Intercept = -1.6651f;
    fws->WA_RW_Slope = 0.2286f;
    fws->WA_RW_Intercept = -0.194f;
    fws->TR_LW_Coefficient = 2677.7f;
    fws->TR_LW_Power = -1.147f;
    fws->TR_RW_Coefficient = 2760.0f;
    fws->TR_RW_Power = -1.222f;

    fws->KP = 0.16;
    fws->KI = 0.000;
    fws->KD = 0.016;

    fws->deadband = 0;
    fws->sensor_max_angle = 270.0;
    fws->sensor_angle_offset = 131.0;

    fws->deadband = 0;

    fws->sensor_max_angle = 270.0;
    fws->sensor_angle_offset = 131.0;
}
