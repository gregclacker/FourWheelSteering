/*
 * c_I2C.cpp
 *
 *  Created on: Oct 24, 2025
 *      Author: FSAE
 */

#include <cstdio>

#include "fourwheelsteer_defs.hpp"
#include "system.hpp"

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
uint32_t getPWM(){
    return DL_Timer_getCaptureCompareValue(PWMTimer, DL_TIMER_CC_INDEX::DL_TIMER_CC_0_INDEX);
}

int duty_cycle_convert(double pid_output) {
    int duty = (int)(fabs(pid_output) * (PWMMAX / 15)); // Adjusted scaling factor

    // Ensure duty cycle is within valid range
    if (duty > PWMMAX) duty = PWMMAX;

    if (duty > 0 && duty < 300) duty = 400;  // Minimum force to actually move

    return duty;
}

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

FWS_Utils::FWS fws;
FWS_Utils::PID pid;

void FIXGPIONOHIGH();
void Setup();
void Run();
void FailSafe();

int main(void) {
    System::init();

    FIXGPIONOHIGH();
//    Setup();
//    Run();
    FailSafe();
}

void Run() {
    char str[100];
    double voltage = 3.3;
    while(1){
        FWS_Utils::SteerMotor();
        //delay_cycles(System::CLK::CPUCLK);
    }
}

void FIXGPIONOHIGH() {
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

    while(1) {
//                System::GPIO::PA3.set();
//                System::GPIO::PA4.set();
//                System::GPIO::PA10.set();
//                System::GPIO::PA11.set();
//                System::GPIO::PA22.set();
//                delay_cycles(System::CLK::CPUCLK * 3);
//
//                System::GPIO::PA3.clear();
//                System::GPIO::PA4.clear();
//                System::GPIO::PA10.clear();
//                System::GPIO::PA11.clear();
//                System::GPIO::PA22.clear();
//                delay_cycles(System::CLK::CPUCLK * 3);
//
//                DL_GPIO_togglePins(System::GPIO::PA3.port, System::GPIO::PA3.pin);
//                DL_GPIO_togglePins(System::GPIO::PA4.port, System::GPIO::PA4.pin);
//                DL_GPIO_togglePins(System::GPIO::PA10.port, System::GPIO::PA10.pin);
//                DL_GPIO_togglePins(System::GPIO::PA11.port, System::GPIO::PA11.pin);
            DL_GPIO_togglePins(System::GPIO::PA22.port, System::GPIO::PA22.pin);
            delay_cycles(System::CLK::CPUCLK/100);
        };
    }
}

void Setup() {
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

    FWS_Utils::SetupPID(&pid);
    FWS_Utils::SetupFWS(&fws);

    /**********************************************************/
}

void FailSafe() {
    while(true) {
       System::FailHard("reached end of main" NEWLINE);
       delay_cycles(20e6);
    }
}

void VoltageTest() {
    char str[100];
    double voltage = 3.3;
    while(1){
       if(voltage > 0x7FF) {
           setPWM(voltage);
       }
       setPWM(PWMMAX);
       snprintf(ARRANDN(str), "PWM Out: %u\nVoltage: %f\n", getPWM(), voltage);
       System::uart_ui.nputs(ARRANDN(str));
       delay_cycles(System::CLK::CPUCLK);
    }
}
