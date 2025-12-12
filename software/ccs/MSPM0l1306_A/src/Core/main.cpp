/*
 * main.cpp
 *
 *  Created on: Oct 24, 2025
 *      Author: FSAE
 */

#include <cstdio>

#include "fourwheelsteer_defs.hpp"
#include "system.hpp"

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

FWS_Utils::FWS::FWS fws;
FWS_Utils::PID::PID pid;
// Leds
static const System::GPIO::GPIO pwm_led1_pins[] = { LED2_PIN };
static const uint32_t pwm_led1_iomuxes[] = { IOMUX_PINCM27_PF_TIMG1_CCP0 };
const FWS_Utils::PWM::PWM PWM_LED1 {
    .timer     = TIMG1,
    .cc_index  = DL_TIMER_CC_0_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pins      = pwm_led1_pins,
    .iomuxes   = pwm_led1_iomuxes,
    .count     = 1
};
static const System::GPIO::GPIO pwm_led2_pins[] = { LED1_PIN };
static const uint32_t pwm_led2_iomuxes[] = { IOMUX_PINCM1_PF_TIMG1_CCP0 };
const FWS_Utils::PWM::PWM PWM_LED2 {
    .timer     = TIMG1,
    .cc_index  = DL_TIMER_CC_1_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pins      = pwm_led2_pins,
    .iomuxes   = pwm_led2_iomuxes,
    .count     = 1
};

// Gate Drivers
static const System::GPIO::GPIO pwm_gate1_pins[] = { GateDriver1_PIN };     // PA03
static const uint32_t pwm_gate1_iomuxes[] = { IOMUX_PINCM4_PF_TIMG2_CCP0 }; // Timer2-PWM1
const FWS_Utils::PWM::PWM PWM_GATE_1 {
    .timer     = TIMG2,
    .cc_index  = DL_TIMER_CC_0_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pins      = pwm_gate1_pins,
    .iomuxes   = pwm_gate1_iomuxes,
    .count     = 1
};
static const System::GPIO::GPIO pwm_gate2_pins[] = { GateDriver2_PIN };     // PA04
static const uint32_t pwm_gate2_iomuxes[] = { IOMUX_PINCM5_PF_TIMG2_CCP1 }; // Timer2-PWM2
const FWS_Utils::PWM::PWM PWM_GATE_2 {
    .timer     = TIMG2,
    .cc_index  = DL_TIMER_CC_1_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pins      = pwm_gate2_pins,
    .iomuxes   = pwm_gate2_iomuxes,
    .count     = 1
};
static const System::GPIO::GPIO pwm_gate3_pins[] = { GateDriver3_PIN };     // PA10
static const uint32_t pwm_gate3_iomuxes[] = { IOMUX_PINCM11_PF_TIMG4_CCP0 };   // Timer4-PWM2
const FWS_Utils::PWM::PWM PWM_GATE_3 {
    .timer     = TIMG4,
    .cc_index  = DL_TIMER_CC_0_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pins      = pwm_gate3_pins,
    .iomuxes   = pwm_gate3_iomuxes,
    .count     = 1
};
static const System::GPIO::GPIO pwm_gate4_pins[] = { GateDriver4_PIN };     // PA11
static const uint32_t pwm_gate4_iomuxes[] = { IOMUX_PINCM12_PF_TIMG4_CCP1 };   // Timer4-PWM2
const FWS_Utils::PWM::PWM PWM_GATE_4 {
    .timer     = TIMG4,
    .cc_index  = DL_TIMER_CC_1_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pins      = pwm_gate4_pins,
    .iomuxes   = pwm_gate4_iomuxes,
    .count     = 1
};

void init();
void run();
void fail_safe();
void test_pins(const System::GPIO::GPIO*, buffsize_t p_size);
void test_pwm_pins();
void turn_on_leds();
void leds_with_pwm();
void killChip();

char uart_buffer[100];


int main(void) {
    init();
//    test_pwm_pins();
    leds_with_pwm();
//    run();
    fail_safe();
}

void init() {
    /*** PWM config *******************************************/
    /* PA26 used as PWM output. driven by TIMER-1 C0 */
    System::init();

    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    delay_cycles(POWER_STARTUP_DELAY);


    FWS_Utils::PWM::INIT_TIMER(PWM_GATE_1.timer);
    FWS_Utils::PWM::INIT_TIMER(PWM_GATE_3.timer);
    FWS_Utils::PWM::INIT_PWM_TIMER(PWM_GATE_1);
    FWS_Utils::PWM::INIT_PWM_TIMER(PWM_GATE_2);
    FWS_Utils::PWM::INIT_PWM_TIMER(PWM_GATE_3);
    FWS_Utils::PWM::INIT_PWM_TIMER(PWM_GATE_4);
//    FWS_Utils::PWM::INIT_PWM_TIMER(PWM_GATE_4);
//    FWS_Utils::PWM::INIT_PWM_TIMER(PWM_GATE_3);
//    FWS_Utils::PWM::INIT_PWM_TIMER(PWM_LED1);
//    FWS_Utils::PWM::INIT_PWM_TIMER(PWM_LED2);

    FWS_Utils::PID::INIT_PID(&pid);
    //FWS_Utils::FWS::INIT_FWS(&fws);

    /**********************************************************/
}

void run() {
    double voltage = 3.3;

    while(1) {
        FWS_Utils::Motor::steer_motor();
        delay_cycles(System::CLK::CPUCLK);
    }
}

void fail_safe() {
    while(true) {
       System::FailHard("reached end of main" NEWLINE);
       delay_cycles(20e6);
    }
}

void test_pwm_pins() {
    const System::GPIO::GPIO _pins[] {
        System::GPIO::PA3,
        System::GPIO::PA4,
        System::GPIO::PA10,
        System::GPIO::PA12,
    };
    test_pins(_pins, 4);
}

void test_pins(const System::GPIO::GPIO* p_pin, buffsize_t p_size) {
    for(int i = 0; i < p_size; i++)
        FWS_Utils::GPIO::INIT_GPIO(p_pin[i]);
    while(true) {
        for(int i = 0; i < p_size; i++)
            DL_GPIO_togglePins(GPIOPINPUX(p_pin[i]));
        delay_cycles(32e6*3);
    };
}

void turn_on_leds() {
    System::init();
    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    FWS_Utils::GPIO::INIT_GPIO(System::GPIO::PA0);
    FWS_Utils::GPIO::INIT_GPIO(System::GPIO::PA26);
    while(1) {
        System::GPIO::PA1.set();
        System::GPIO::PA26.set();
        delay_cycles(System::CLK::CPUCLK/100);
    };
}

void leds_with_pwm() {
    static double duty = 0.0;
    static double dir = 0.1;

    FWS_Utils::PWM::set_PWM_duty(PWM_GATE_1, 0.1);
    FWS_Utils::PWM::set_PWM_duty(PWM_GATE_2, 0.1);
    FWS_Utils::PWM::set_PWM_duty(PWM_GATE_3, 0.1);
    FWS_Utils::PWM::set_PWM_duty(PWM_GATE_4, 0.9);
//    FWS_Utils::PWM::set_PWM_duty(PWM_GATE_4, 0.9);
//    FWS_Utils::PWM::set_PWM_duty(PWM_LED1, 0.1);
//    FWS_Utils::PWM::set_PWM_duty(PWM_LED2, 0.9);
//    tmp();


    while(1) {
        //int duty_cycle = FWS_Utils::PWM::get_PWM_duty_cycle(FWS_Utils::ADC::get_ADC_voltage(ADC_1_ADDR));
        duty += dir;
        if(duty >= 1) {
            duty = 1;
            dir = -0.1;
        }
        if(duty <= 0) {
            duty = 0;
            dir = 0.1;
        }
        // getPWM()
        //snprintf(ARRANDN(uart_buffer), "PWM Out: %u\n duty: %f\n dir: %f\n", FWS_Utils::PWM::PWM_output(PWM1), duty, dir);
        snprintf(ARRANDN(uart_buffer), "PWM Out: %u\n", FWS_Utils::PWM::PWM_output(PWM_LED2));
        //System::uart_ui.nputs(ARRANDN(uart_buffer));


        //setPWM2(duty * 0xFFFF);


        //FWS_Utils::PWM::set_PWM_duty(PWM1, duty);
        //FWS_Utils::PWM::set_PWM_duty(PWM3, duty);
        delay_cycles(System::CLK::CPUCLK/20);
    }
}

void voltage_test() {
    char str[100];
    double voltage = 3.3;
    while(1){
       if(voltage > 0x7FF) {
           FWS_Utils::PWM::set_PWM_duty(PWM_GATE_1, voltage);
       }
       FWS_Utils::PWM::set_PWM_duty(PWM_GATE_1, FWS_Utils::PWM::PWMMAX);
       snprintf(ARRANDN(str), "PWM Out: %u\nVoltage: %f\n", FWS_Utils::PWM::PWM_output(PWM_GATE_1), voltage);
       System::uart_ui.nputs(ARRANDN(str));
       delay_cycles(System::CLK::CPUCLK);
    }
}

void killChip() {
    auto &lh = System::GPIO::PA4;
    auto &ll = System::GPIO::PA3;
    auto &rh = System::GPIO::PA11;
    auto &rl = System::GPIO::PA10;

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
