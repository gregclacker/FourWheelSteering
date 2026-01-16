/*
 * main.cpp
 *
 *  Created on: Oct 24, 2025
 *      Author: FSAE
 */

#include <cstdio>
#include <ti/driverlib/dl_timera.h>.h>

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

const PWM* PWM_GATES[] {
       &PWM_GATE_1,
       &PWM_GATE_2,
       &PWM_GATE_3,
       &PWM_GATE_4
};

void init();
void test_init();
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

//    test_init();

////    INIT_TIMER_DSYNC(PWM_GATE_1.timer);
////    INIT_TIMER_DSYNC(PWM_GATE_3.timer);
////
////    INIT_PWM_OUTPUT(PWM_LED1);
////    INIT_PWM_OUTPUT(PWM_LED2);
//
    INIT_TIMER_SYNC(PWMTIMERSYNC_REG);
    //PWMMAX/2
    INIT_TIMER_PWM(PWM_GATE_1, 0);
    INIT_TIMER_PWM(PWM_GATE_3, PWMMAX/2);

    INIT_PWM_OUTPUT(PWM_GATE_1, false);
    INIT_PWM_OUTPUT(PWM_GATE_2, false);
    INIT_PWM_OUTPUT(PWM_GATE_3, false);
    INIT_PWM_OUTPUT(PWM_GATE_4, false);
//
//    INIT_PWM_OUTPUT(PWM_GATE_1, false);
//    INIT_PWM_OUTPUT(PWM_GATE_2, false);
//    INIT_PWM_OUTPUT(PWM_GATE_3, false);
//    INIT_PWM_OUTPUT(PWM_GATE_4, false);
//
//    pwm_start_timers();
//    start_timer(PWMTIMERSYNC_REG);
//    DL_Timer_generateCrossTrigger(PWMTIMERSYNC_REG);

    INIT_PID(&pid, &fws);
    INIT_FWS(&fws);


//    DL_Timer_setPhaseLoadValue(PWM_GATE_1.timer, PWMMAX);
//    DL_Timer_setPhaseLoadValue(PWM_GATE_3.timer, PWMMAX);
//    DL_Timer_setPhaseLoadValue(PWM_GATE_2.timer, PWMMAX/2);
//    DL_Timer_setPhaseLoadValue(PWM_GATE_4.timer, PWMMAX/2);
}

GPTIMER_Regs *master = PWMTIMERSYNC_REG;
GPTIMER_Regs *slave = PWM_GATE_1.timer;
PWM PWMT = PWM_GATE_1;
uint32_t phase = PWMMAX / 2;
uint16_t rise_ns = 0;
uint16_t fall_ns = 0;
void test_init() {
//////////////////////////////////////////////////////////  MASTER
        // Configure clock
        constexpr DL_Timer_ClockConfig clkCfg = {
                .clockSel    = DL_TIMER_CLOCK_BUSCLK,
                .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
                .prescale    = 0,
            };
        // Configure as basic counter, no PWM outputs needed
        constexpr DL_Timer_PWMConfig pwmCfg = {
                .period            = PWMMAX,
                .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN, //DL_TIMER_PWM_MODE_CENTER_ALIGN
                .isTimerWithFourCC = true,
                .startTimer        = DL_TIMER_STOP,
            };

        DL_Timer_enablePower(master);
        DL_Timer_setClockConfig(master, &clkCfg);
        DL_Timer_initPWMMode(master, &pwmCfg);
        DL_Timer_enableClock(master);

        DL_Timer_configCrossTrigger(
            master,
            DL_TIMER_CROSS_TRIG_SRC_ZERO,           // emit FSUB0 on ZERO
            DL_TIMER_CROSS_TRIGGER_INPUT_DISABLED,
            DL_TIMER_CROSS_TRIGGER_MODE_ENABLED
        );

        DL_Timer_startCounter(master);
//////////////////////////////////////////////////////////  SLAVE-1

        DL_Timer_enablePower(slave);
        DL_Timer_setClockConfig(slave, &clkCfg);
        DL_Timer_initPWMMode(slave, &pwmCfg);
        DL_Timer_enableClock(slave);

        for(int i = 0; i < PWMT.count; i++) {
            DL_GPIO_initPeripheralOutputFunctionFeatures(
                    PWMT.pins[i].iomux,
                    PWMT.iomuxes[i],
                    DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                    DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                    DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                    DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                );
        }

        // 1. Allow FSUB0 in
        DL_Timer_configCrossTrigger(
            slave,
            DL_TIMER_CROSS_TRIG_SRC_FSUB0,
            DL_TIMER_CROSS_TRIGGER_INPUT_ENABLED,
            DL_TIMER_CROSS_TRIGGER_MODE_ENABLED
        );

        // 2. Map external trigger to LOAD/RESET action
        DL_Timer_setExternalTriggerEvent(slave,
            DL_TIMER_EXT_TRIG_SEL_TRIG_SUB_0);

        DL_Timer_setCounterControl(
            slave,
            DL_TIMER_CZC_CCCTL0_ZCOND,   // use ZERO for reload
            DL_TIMER_CAC_CCCTL0_ACOND,   // no active reset
            DL_TIMER_CLC_CCCTL0_LCOND    // LOAD on external trigger (FSUB0)
        );

        // 3. Optional — phase offset
        DL_Timer_enablePhaseLoad(slave);
        DL_Timer_setPhaseLoadValue(slave, phase);   // 0 = symmetric

        // 4. Slaves must run continuously
        DL_Timer_startCounter(slave);

//////////////////////////////////////////////////////////  SLAVE-1-PT2
        DL_Timer_setCaptureCompareOutCtl(slave,
            DL_TIMER_CC_OCTL_INIT_VAL_LOW,
            DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
            DL_TIMER_CC_OCTL_SRC_FUNCVAL,
            DL_TIMER_CC_0_INDEX);

        DL_Timer_setCaptureCompareOutCtl(slave,
            DL_TIMER_CC_OCTL_INIT_VAL_LOW,
            DL_TIMER_CC_OCTL_INV_OUT_ENABLED,
            DL_TIMER_CC_OCTL_SRC_FUNCVAL,
            DL_TIMER_CC_1_INDEX);
        DL_Timer_setCCPDirection(slave,
             DL_Timer_getCCPDirection(slave) | DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
        //DL_Timer_setDeadBand(slave, rise_ns, fall_ns, DL_TIMER_DEAD_BAND_MODE_1);
//////////////////////////////////////////////////////////  TEST
        float duty = 0.6;
        DL_Timer_setCaptureCompareValue(slave, (uint32_t)(PWMMAX * duty), DL_TIMER_CC_0_INDEX);
        DL_Timer_setCaptureCompareValue(slave, (uint32_t)(PWMMAX * duty), DL_TIMER_CC_1_INDEX);
        //DL_Timer_setCounterValueAfterEnable
        uint32_t b = DL_Timer_getTimerCount(slave);
        DL_Timer_generateCrossTrigger(master);
        uint32_t a = DL_Timer_getTimerCount(slave);

        if (a == b) {
            System::uart_ui.nputs(ARRANDN("failed\n"));
            // Slave still ignored the trigger
            // This means CCCTLx LCOND or CLC mapping is wrong
        } else {
            System::uart_ui.nputs(ARRANDN("success\n"));
            // Slave counter incremented cross-trigger working
        }
//////////////////////////////////////////////////////////
}

void run() {
    double voltage = 3.3;
    int counter = 0;

    while(1) {
        //steer_motor(PWM_GATES, 4, &fws);
        FWS_Utils::PWM::set_PWM_duty(&PWM_GATE_1, 0.8);
        FWS_Utils::PWM::set_PWM_duty(&PWM_GATE_2, 0.3);
        FWS_Utils::PWM::set_PWM_duty(&PWM_GATE_3, 0.8);
        FWS_Utils::PWM::set_PWM_duty(&PWM_GATE_4, 0.3);
        //counter++;
        snprintf(ARRANDN(uart_buffer), "|MASTER|Cnt: %u||Phase:%u\n", DL_Timer_getTimerCount(PWMTIMERSYNC_REG), DL_Timer_getPhaseLoadValue(PWMTIMERSYNC_REG));
        System::uart0.nputs(ARRANDN(uart_buffer));
        snprintf(ARRANDN(uart_buffer), "|PWM_GATE_1|Cnt: %u||Phase:%u||PhaseOn:%d\n", DL_Timer_getTimerCount(PWM_GATE_1.timer), DL_Timer_getPhaseLoadValue(PWM_GATE_1.timer), DL_Timer_isPhaseLoadEnabled(PWM_GATE_1.timer));
        System::uart0.nputs(ARRANDN(uart_buffer));
        snprintf(ARRANDN(uart_buffer), "|PWM_GATE_3|Cnt: %u||Phase:%u||PhaseOn:%d\n", DL_Timer_getTimerCount(PWM_GATE_3.timer), DL_Timer_getPhaseLoadValue(PWM_GATE_3.timer), DL_Timer_isPhaseLoadEnabled(PWM_GATE_3.timer));
        System::uart0.nputs(ARRANDN(uart_buffer));

        if(counter % 20 < 10)
            motor_move = 1;
        if(counter % 20 >= 10)
            motor_move = 2;
        delay_cycles(System::CLK::CPUCLK);
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
    start_timers(_timers, 3);
    DL_Timer_generateCrossTrigger(PWMTIMERSYNC_REG);
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
