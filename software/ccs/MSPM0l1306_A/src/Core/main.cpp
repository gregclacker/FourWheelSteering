/*
 * main.cpp
 *
 *  Created on: Oct 24, 2025
 *      Author: FSAE
 *      Contributer: Marcus Turley, Abdul-Muizz Abgoola
 */

#include <cstdio>

#include "fourwheelsteer_defs.hpp"
#include "system.hpp"

using namespace System::GPIO;
using namespace FWS_Utils::Motor;
using namespace FWS_Utils::GPIO;
using namespace FWS_Utils::PWM;
using namespace FWS_Utils::FWS;
using namespace FWS_Utils::PID;

/*************************************************************/

/*** GENERAL NOTE
     * Some sudo (derived from what someone else wrote but forgot who)
     * PWM Signal will use the digital output to drive the analog voltage. This voltage is scaled by the pwm timer
     * The H Bridge has 4 fets. These 4 fets are meant to be managed by 4 GPIO pins
     * Fets that are diagonally oppositly active to one another are in sync while the nearby fets are inversely active
     * Ensure the fets with the same terminal are not active with one another. If they are it will short
     * Once a pair of fets are disabled the other pair must be activated within some x amount of nanoseconds or the motor destroys itself
     */
/*** PWM config (BOARD SPECIFIC)
     * PA03 used as PWM output 1. driven by TIMER-2 C0
     * PA04 used as PWM output 2. driven by TIMER-2 C1
     * PA10 used as PWM output 1. driven by TIMER-4 C0
     * PA11 used as PWM output 2. driven by TIMER-4 C1
     * */
/*** ADC Information
     * 120 SPS Minimum ~ 12 Bit Resolution
     * 3.3 Vref
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

FWS fws;PID pid;

// Gate Drivers
const PWM FWS_Utils::PWM::PWM_GATE_1 {
    .timer      = TIMER_REG_1,
    .cc_index   = DL_TIMER_CC_0_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pin        = GATE_PIN_1,                       // PA03
    .iomux      = IOMUX_PINCM4_PF_TIMG2_CCP0,       // PWM_TIMER_1-PWM1
    .phase      = PWMMAX * 0.0
};
const PWM FWS_Utils::PWM::PWM_GATE_2 {
    .timer      = TIMER_REG_1,
    .cc_index   = DL_TIMER_CC_1_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pin        = GATE_PIN_2,                       // PA04
    .iomux      = IOMUX_PINCM5_PF_TIMG2_CCP1,       // PWM_TIMER_1-PWM2
    .phase      = PWMMAX * 0.5
};
const PWM FWS_Utils::PWM::PWM_GATE_3 {
    .timer      = TIMER_REG_2,
    .cc_index   = DL_TIMER_CC_0_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pin        = GATE_PIN_3,                       // PA10
    .iomux      = IOMUX_PINCM11_PF_TIMG4_CCP0,      // PWM_TIMER_2-PWM2
    .phase      = PWMMAX * 0.0
};
const PWM FWS_Utils::PWM::PWM_GATE_4 {
    .timer      = TIMER_REG_2,
    .cc_index   = DL_TIMER_CC_1_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pin        = GATE_PIN_4,                       // PA11
    .iomux      = IOMUX_PINCM12_PF_TIMG4_CCP1,       // PWM_TIMER_2-PWM2
    .phase      = PWMMAX * 0.5
};

static const PWM timer_1_pwms[] = { PWM_GATE_1, PWM_GATE_2 };         // PA3 & PA4
const TIMER FWS_Utils::PWM::PWM_TIMER_1 {
    .timer      = TIMER_REG_1,
    .pwms       = timer_1_pwms,
    .pwm_count  = 2
};
static const PWM timer_2_pwms[] = { PWM_GATE_3, PWM_GATE_4 };         // PA10 & PA11
const TIMER FWS_Utils::PWM::PWM_TIMER_2 {
    .timer      = TIMER_REG_2,
    .pwms       = timer_2_pwms,
    .pwm_count  = 2
};

void init();
void run();

void init_duty(double);

void print_gate_stats();
void print_gate_stat(const GPTIMER_Regs*, const char*);
void print_gate_stat(TIMER, const char*);

void fail_safe();

char uart_buffer[100];


int main(void) {
    init();
    run();
    fail_safe();
}

void init() {
    System::init();

    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    delay_cycles(POWER_STARTUP_DELAY);

    /*** PWM config (BOARD SPECIFIC)
     * PA03 used as PWM output 1. driven by TIMER-2 C0
     * PA04 used as PWM output 2. driven by TIMER-2 C1
     * PA10 used as PWM output 1. driven by TIMER-4 C0
     * PA11 used as PWM output 2. driven by TIMER-4 C1
     * */

    // Test to see if pwm is working properly
    INIT_TIMER_BASIC(PWM_TIMER_1, 0);
    INIT_PWM_OUTPUTS(PWM_TIMER_1, false, false);
    INIT_TIMER_BASIC(PWM_TIMER_2, PWMMAX/2);
    INIT_PWM_OUTPUTS(PWM_TIMER_2, false, false);

    start_timer(PWM_TIMER_1.timer, false);
    start_timer(PWM_TIMER_2.timer, false);

    // This section does not work currently since I could not get timersync work
    // TODO: Trying to get timer sync working but could not. If you can't get it working try shifting output of pwm in code :(
//    INIT_TIMER_MASTER(TIMERSYNC_REG);
//    INIT_TIMER_SLAVE(PWM_TIMER_1, 0);
//    INIT_TIMER_SLAVE(PWM_TIMER_2,PWMMAX/2);  //PWMMAX/2
//
//    INIT_PWM_OUTPUTS(PWM_TIMER_1, true, false);
//    INIT_PWM_OUTPUTS(PWM_TIMER_2, true, false);
//    DL_Timer_generateCrossTrigger(TIMERSYNC_REG);
//    start_timer(TIMERSYNC_REG, true);

    INIT_PID(&pid, &fws);
    INIT_FWS(&fws);
}

void run() {
//    init_duty(0.0);
    GPTIMER_Regs *_timers[] = {PWM_TIMER_1.timer, PWM_TIMER_2.timer};
//    start_timers(_timers, 2, false);

    init_duty(0.8);
//    DL_Timer_setCaptureCompareValue(PWM_TIMER_1.timer, (uint32_t)(PWMMAX * 0), DL_TIMER_CC_0_INDEX);

    while(1) {
        //This should steer the motor but make sure you're pwms work first
        //steer_motor(PWM_GATES, 4, &fws);
        print_gate_stats();

        delay_cycles(System::CLK::CPUCLK);
    }
}

void init_duty(double p_duty = 0) {
    FWS_Utils::PWM::set_duty(&PWM_TIMER_1.pwms[0], p_duty);
    FWS_Utils::PWM::set_duty(&PWM_TIMER_1.pwms[1], p_duty);
    FWS_Utils::PWM::set_duty(&PWM_TIMER_2.pwms[0], p_duty);
    FWS_Utils::PWM::set_duty(&PWM_TIMER_2.pwms[1], p_duty);
}

void print_gate_stats() {
    print_gate_stat(TIMERSYNC_REG, "MASTER");
    print_gate_stat(PWM_TIMER_1, "PWM_TIMER_1");
    print_gate_stat(PWM_TIMER_2, "PWM_TIMER_2");
    snprintf(ARRANDN(uart_buffer), "\n\n\n");
}
void print_gate_stat(const GPTIMER_Regs *p_timer_reg, const char* p_name = nullptr) {
    if(p_name == nullptr)
        p_name = "Timer";
    snprintf(ARRANDN(uart_buffer), "|%s|Count: %u|Phase:%u||Config:%u\n",
             p_name,
             DL_Timer_getTimerCount(p_timer_reg),
             DL_Timer_getPhaseLoadValue(p_timer_reg),
             DL_Timer_getCrossTriggerConfig(p_timer_reg)
             );
    System::uart0.nputs(ARRANDN(uart_buffer));
}
void print_gate_stat(TIMER p_timer, const char* p_name = nullptr) {
    GPTIMER_Regs *_timer_reg;
    if(p_name == nullptr)
        p_name = "Timer";
    snprintf(ARRANDN(uart_buffer), "||%s||Count: %u||Duty_1:%u||Duty_2:%u||Phase:%u||Phase:%u||Config:%u\n||",
            p_name,
            DL_Timer_getTimerCount(_timer_reg),
            FWS_Utils::PWM::get_duty(&p_timer.pwms[0]),
            FWS_Utils::PWM::get_duty(&p_timer.pwms[1]),
//            DL_Timer_getPhaseLoadValue(_timer_reg),
            get_phase(&p_timer.pwms[0]),
            get_phase(&p_timer.pwms[1]),
            DL_Timer_getCrossTriggerConfig(_timer_reg)
        );
    System::uart0.nputs(ARRANDN(uart_buffer));
}

void fail_safe() {
    while(true) {
       System::FailHard("reached end of main" NEWLINE);
       delay_cycles(System::CLK::MCLK * 5/8);
    }
}
