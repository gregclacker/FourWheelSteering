/*
 * FWS_Utils.cpp
 *
 *  Created on: Nov 25, 2025
 *      Author: FSAE
 */

#include "system.hpp"
#include "fourwheelsteer_defs.hpp"

//fs - Front Steer ADC Angle
//Go forward when fs <= 180 or fs >= 0
//Go backward when fs > 180 or fs < 0
void FWS_Utils::SteerMotor() {
    float _fsangle = FWS_Utils::READ_FS_POT();
    if(_fsangle <= 180 || _fsangle >= 0) {
        MotorForward();
    }
    if(_fsangle < 180 || _fsangle > 0) {
        MotorReverse();
    }
}

// 6 \ 10 >>> 9 / 7
//
//  10  7
//    \/
//    /\
//   9  6
//
void FWS_Utils::MotorForward() {
    //Set Pins
    System::GPIO::PA6.set();
    System::GPIO::PA10.set();

}
void FWS_Utils::MotorReverse() {
    //Set Pins
    System::GPIO::PA7.set();
    System::GPIO::PA9.set();
}

/* ADC Information
 * 120 SPS Minimum ~ 12 Bit Resolution
 * 3.3 Vref
*/
constexpr float REFVOLTAGE = 3.3f;
constexpr float MAXBITS = 2048.0f; // Currently 12 bits unsigned
void FWS_Utils::getADCOut(uint8_t p_adcAddr, int16_t *_raw) {
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
float FWS_Utils::getADCVoltage(uint8_t p_adcAddr) {
    int16_t _raw;
    getADCOut(p_adcAddr, &_raw);
    return (_raw / MAXBITS) * REFVOLTAGE;
}

double FWS_Utils::READ_FS_POT() {
    int16_t raw_fs_val = 0;
    getADCOut(ADC_1_ADDR, &raw_fs_val);
    double front_voltage = (raw_fs_val / MAXBITS) * REFVOLTAGE;
    double fs_angle = (front_voltage / REFVOLTAGE) * 270.0 - 131.0;
    return fs_angle;
}

double FWS_Utils::READ_RS_POT() {
    int16_t raw_rs_val = 0;
    getADCOut(ADC_2_ADDR, &raw_rs_val);
    double rear_voltage = (raw_rs_val / MAXBITS) * REFVOLTAGE;
    double rs_angle = (rear_voltage / REFVOLTAGE) * 270.0 - 131.0;
    return rs_angle;
}

#define INTEGRAL_LIMIT 100 // Adjust based on testing
double FWS_Utils::compute_pid(PID *pid, double error) {
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

double FWS_Utils::IdealRearAngle(double FS_SteeringAngle, FWS *fws) {
    if (fws->FS_SteeringAngle > fws->deadband) { //Left Turn Steering Percentage Calc
        fws->LT_Percentage = -50 * tanh(0.1 * (TR_RW_Coefficient * pow(fws->FS_SteeringAngle, TR_RW_Power)) - 4.5) + 50;  //Steering using hyperbolic tangent curve to get percentage of left turn
        double RearAngle = (fws->LT_Percentage / 100.0) * 135.0;
        fws->RS_Deg = RearAngle;
        return fws->RS_Deg;
    } else if (fws->FS_SteeringAngle < -fws->deadband) { // Right Turn Steering Percentage Calc
        fws->RT_Percentage = -50 * tanh(0.1 * (TR_LW_Coefficient * pow(-fws->FS_SteeringAngle, TR_LW_Power)) - 4.5) + 50;  //Steering using hyperbolic tangent curve to get percentage of right turn
        double RearAngle = (fws->RT_Percentage / 100.0) * 135.0;
        fws->RS_Deg = RearAngle * -1 ;
        return fws->RS_Deg;
    }
    {
        fws->RS_Deg = 0.0;
        return fws->RS_Deg;
    }
}

void FWS_Utils::SetupFWS(FWS *fws) {
    fws->deadband = 0;
}

void FWS_Utils::SetupPID(PID *pid) {
    pid->Kp = KP;
    pid->Ki = KI;
    pid->Kd = KD;
    pid->previous_error = 0;
    pid->integral = 0;
}
