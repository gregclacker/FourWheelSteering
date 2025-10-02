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

    setPWM(0);
    DL_Timer_enableClock(PWMTimer);
    DL_Timer_setCCPDirection(PWMTimer, DL_TIMER_CC0_OUTPUT);
    DL_Timer_startCounter(PWMTimer);

    /**********************************************************/

    while(1){
        static double duty = 0;
        static double dir = 0.01;

        duty += dir;

        if(duty > 1){
            duty = 1;
            dir = -0.01;
        }
        if(duty < 0){
            duty = 0;
            dir = 0.01;
        }

        setPWM(PWMMAX * duty);

        delay_cycles(System::CLK::CPUCLK/20);
    }

    while(true) {
        System::FailHard("reached end of main" NEWLINE);
        delay_cycles(20e6);
    }
}
