/*
 * main.cpp
 *
 *  Created on: Oct 24, 2025
 *      Author: FSAE
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

FWS fws;
PID pid;

// Leds
static const GPIO pwm_led1_pins[] = { LED2_PIN };
static const uint32_t pwm_led1_iomuxes[] = { IOMUX_PINCM27_PF_TIMG1_CCP0 };
const PWM PWM_LED1 {
    .timer      = TIMG1,
    .cc_index   = DL_TIMER_CC_0_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pins       = pwm_led1_pins,
    .iomuxes    = pwm_led1_iomuxes,
    .count      = 1
};
static const GPIO pwm_led2_pins[] = { LED1_PIN };
static const uint32_t pwm_led2_iomuxes[] = { IOMUX_PINCM1_PF_TIMG1_CCP0 };
const PWM PWM_LED2 {
    .timer      = TIMG1,
    .cc_index   = DL_TIMER_CC_1_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pins       = pwm_led2_pins,
    .iomuxes    = pwm_led2_iomuxes,
    .count      = 1
};

// Gate Drivers
static const GPIO pwm_gate1_pins[] = { GateDriver1_PIN };         // PA03
static const uint32_t pwm_gate1_iomuxes[] = { IOMUX_PINCM4_PF_TIMG2_CCP0 };     // Timer2-PWM1
const PWM PWM_GATE_1 {
    .timer      = PWMTIMER1_REG,
    .cc_index   = DL_TIMER_CC_0_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pins       = pwm_gate1_pins,
    .iomuxes    = pwm_gate1_iomuxes,
    .count      = 1
};
static const GPIO pwm_gate2_pins[] = { GateDriver2_PIN };         // PA04
static const uint32_t pwm_gate2_iomuxes[] = { IOMUX_PINCM5_PF_TIMG2_CCP1 };     // Timer2-PWM2
const PWM PWM_GATE_2 {
    .timer      = PWMTIMER1_REG,
    .cc_index   = DL_TIMER_CC_1_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pins       = pwm_gate2_pins,
    .iomuxes    = pwm_gate2_iomuxes,
    .count      = 1
};
static const GPIO pwm_gate3_pins[] = { GateDriver3_PIN };         // PA10
static const uint32_t pwm_gate3_iomuxes[] = { IOMUX_PINCM11_PF_TIMG4_CCP0 };    // Timer4-PWM2
const PWM PWM_GATE_3 {
    .timer      = PWMTIMER2_REG,
    .cc_index   = DL_TIMER_CC_0_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pins       = pwm_gate3_pins,
    .iomuxes    = pwm_gate3_iomuxes,
    .count      = 1
};
static const GPIO pwm_gate4_pins[] = { GateDriver4_PIN };         // PA11
static const uint32_t pwm_gate4_iomuxes[] = { IOMUX_PINCM12_PF_TIMG4_CCP1 };    // Timer4-PWM2
const PWM PWM_GATE_4 {
    .timer      = PWMTIMER2_REG,
    .cc_index   = DL_TIMER_CC_1_INDEX,
    .cc_output  = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pins       = pwm_gate4_pins,
    .iomuxes    = pwm_gate4_iomuxes,
    .count      = 1
};

void init();
void run();
void pwm_start_timers();
void fail_safe();
void test_pins(const GPIO*, buffsize_t p_size);
void test_pwm_pins();
void turn_on_leds();
void leds_with_pwm();
void killChip();

char uart_buffer[100];


int main(void) {
    init();

//    System::init();
//
//    System::uart_ui.setBaudTarget(115200);
//    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));
//
//    delay_cycles(POWER_STARTUP_DELAY);
//
//    DL_GPIO_initDigitalOutput(PA3.iomux);
//    DL_GPIO_initDigitalOutput(PA4.iomux);
//    DL_GPIO_initDigitalOutput(PA10.iomux);
//    DL_GPIO_initDigitalOutput(PA11.iomux); // 11
//    DL_GPIO_initDigitalOutput(PA12.iomux);
//
//    DL_GPIO_enableOutput(GPIOPINPUX(PA3));
//    DL_GPIO_enableOutput(GPIOPINPUX(PA4));
//    DL_GPIO_enableOutput(GPIOPINPUX(PA10));
//    DL_GPIO_enableOutput(GPIOPINPUX(PA11)); // 11
//    DL_GPIO_enableOutput(GPIOPINPUX(PA12));
//
//    PA3.set();
//    PA4.set();
//    PA10.set();
//    PA11.set(); // 11
//    PA12.set();

//    test_pwm_pins();
//    leds_with_pwm();

    run();
    fail_safe();
}

void init() {
    System::init();

    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    delay_cycles(POWER_STARTUP_DELAY);

    /*** PWM config
     * PA03 used as PWM output 1. driven by TIMER-2 C0
     * PA04 used as PWM output 2. driven by TIMER-2 C1
     * PA10 used as PWM output 1. driven by TIMER-4 C0
     * PA11 used as PWM output 2. driven by TIMER-4 C1
     * */
//    INIT_TIMER_DSYNC(PWM_GATE_1.timer);
//    INIT_TIMER_DSYNC(PWM_GATE_3.timer);

//        INIT_PWM_OUTPUT(PWM_LED1);
//        INIT_PWM_OUTPUT(PWM_LED2);

    INIT_TIMER(PWMTIMERSYNC_REG, true, 0);
    INIT_TIMER(PWM_GATE_1.timer, false, PWMMAX/2);
    INIT_TIMER(PWM_GATE_3.timer, false, PWMMAX);
    INIT_PWM_OUTPUT(PWM_GATE_1, false);
    INIT_PWM_OUTPUT(PWM_GATE_2, true);
    INIT_PWM_OUTPUT(PWM_GATE_3, false);
    INIT_PWM_OUTPUT(PWM_GATE_4, true);

    INIT_PID(&pid, &fws);
    INIT_FWS(&fws);

    pwm_start_timers();
    DL_Timer_generateCrossTrigger(PWMTIMERSYNC_REG);

//    DL_Timer_setPhaseLoadValue(PWM_GATE_1.timer, PWMMAX);
//    DL_Timer_setPhaseLoadValue(PWM_GATE_3.timer, PWMMAX);
//    DL_Timer_setPhaseLoadValue(PWM_GATE_2.timer, PWMMAX/2);
//    DL_Timer_setPhaseLoadValue(PWM_GATE_4.timer, PWMMAX/2);
}

const PWM* PWM_GATES[] {
       &PWM_GATE_1,
       &PWM_GATE_2,
       &PWM_GATE_3,
       &PWM_GATE_4
};
void run() {
    double voltage = 3.3;
    int counter = 0;

    while(1) {
        steer_motor(PWM_GATES, 4, &fws);
        //counter++;
        snprintf(ARRANDN(uart_buffer), "Counter Out: %u\n", counter % 20);
        System::uart0.nputs(ARRANDN(uart_buffer));
        if(counter % 20 < 10)
            motor_move = 1;
        if(counter % 20 >= 10)
            motor_move = 2;
//        delay_cycles(System::CLK::CPUCLK/100);
    }
}

void fail_safe() {
    while(true) {
       System::FailHard("reached end of main" NEWLINE);
       delay_cycles(System::CLK::MCLK * 5/8);
    }
}

void pwm_start_timers() {
    GPTIMER_Regs *_timers[] {
         PWM_GATE_1.timer,
         PWM_GATE_3.timer,
         PWMTIMERSYNC_REG,
    };
    DL_Timer_generateCrossTrigger(PWMTIMERSYNC_REG);
    start_timers(_timers, 3);
}

void test_pwm_pins() {
    const GPIO _pins[] {
        PA3,
        PA4,
        PA10,
        PA11,
    };
    test_pins(_pins, 4);
}

void test_pins(const GPIO* p_pin, buffsize_t p_size) {
    for(int i = 0; i < p_size; i++)
        FWS_Utils::GPIO::INIT_GPIO(p_pin[i]);
    while(true) {
        for(int i = 0; i < p_size; i++)
            DL_GPIO_togglePins(GPIOPINPUX(p_pin[i]));
        delay_cycles(System::CLK::MCLK*3);
    };
}

void turn_on_leds() {
    System::init();
    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    FWS_Utils::GPIO::INIT_GPIO(PA0);
    FWS_Utils::GPIO::INIT_GPIO(PA26);
    while(1) {
        PA1.set();
        PA26.set();
        delay_cycles(System::CLK::CPUCLK/100);
    };
}

void leds_with_pwm() {
    static double duty = 0.0;
    static double dir = 0.1;

//    set_PWM_duty(PWM_GATE_1, 0.1);
//    set_PWM_duty(PWM_GATE_2, 0.1);
//    set_PWM_duty(PWM_GATE_3, 0.1);
//    set_PWM_duty(PWM_GATE_4, 0.9);
//    set_PWM_duty(PWM_GATE_4, 0.9);

    while(1) {
        //int duty_cycle = get_PWM_duty_cycle(FWS_Utils::ADC::get_ADC_voltage(ADC_1_ADDR));
        duty += dir;
        if(duty >= 1) {
            duty = 1;
            dir = -0.1;
        }
        if(duty <= 0) {
            duty = 0;
            dir = 0.1;
        }
        //snprintf(ARRANDN(uart_buffer), "PWM Out: %u\n duty: %f\n dir: %f\n", PWM_output(PWM_GATE_1), duty, dir);
        snprintf(ARRANDN(uart_buffer), "PWM Out: %u\n", PWM_output(&PWM_GATE_1));
        //System::uart_ui.nputs(ARRANDN(uart_buffer));

        set_PWM_duty(&PWM_GATE_1, duty);
        set_PWM_duty(&PWM_GATE_2, duty);
        set_PWM_duty(&PWM_GATE_3, duty);
        set_PWM_duty(&PWM_GATE_4, duty);
        delay_cycles(System::CLK::CPUCLK/20);
    }
}

void voltage_test() {
    char str[100];
    double voltage = 3.3;
    while(1){
       if(voltage > 0x7FF) {
           set_PWM_duty(&PWM_GATE_1, voltage);
       }
       set_PWM_duty(&PWM_GATE_1, PWMMAX);
       snprintf(ARRANDN(str), "PWM Out: %u\nVoltage: %f\n", PWM_output(&PWM_GATE_1), voltage);
       System::uart_ui.nputs(ARRANDN(str));
       delay_cycles(System::CLK::CPUCLK);
    }
}

void killChip() {
    auto &lh = PA4;
    auto &ll = PA3;
    auto &rh = PA11;
    auto &rl = PA10;

//    DL_GPIO_initDigitalOutput(lh.iomux);
//    DL_GPIO_initDigitalOutput(ll.iomux);
//    DL_GPIO_initDigitalOutput(rh.iomux);
//    DL_GPIO_initDigitalOutput(rl.iomux);

    DL_GPIO_initDigitalOutputFeatures(
        lh.iomux,
        DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
    );
    DL_GPIO_initDigitalOutputFeatures(
        ll.iomux,
        DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
    );
    DL_GPIO_initDigitalOutputFeatures(
        rh.iomux,
        DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
    );
    DL_GPIO_initDigitalOutputFeatures(
        rl.iomux,
        DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
        DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
    );

    lh.clear();
    ll.clear();
    rh.clear();
    rl.clear();

//    delay_cycles(32e6*5);
//    lh.set();
//    ll.set();
//    rh.set();
//    rl.set();

    while(true){
        DL_GPIO_togglePins(lh.port,lh.pin);
        DL_GPIO_togglePins(ll.port,ll.pin);
        DL_GPIO_togglePins(rh.port,rh.pin);
        DL_GPIO_togglePins(rl.port,rl.pin);
        delay_cycles(32e6*3);
    }
}
