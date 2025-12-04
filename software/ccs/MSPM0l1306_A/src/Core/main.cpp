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

static const System::GPIO::GPIO pwm1_pins[] = { GateDriver1_PIN, LED1_PIN };
static const uint32_t pwm1_iomuxes[] = { IOMUX_PINCM4_PF_TIMG2_CCP0, IOMUX_PINCM1_PF_TIMG1_CCP0 };
const FWS_Utils::PWM::PWM PWM1 {
    .timer     = TIMG1,
    .cc_index  = DL_TIMER_CC_0_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pins      = pwm1_pins,
    .iomuxes   = pwm1_iomuxes,
    .count     = 2
};
static const System::GPIO::GPIO pwm2_pins[] = { GateDriver2_PIN };
static const uint32_t pwm2_iomuxes[] = { IOMUX_PINCM5_PF_TIMG2_CCP1 };
const FWS_Utils::PWM::PWM PWM2 {
    .timer     = TIMG1,
    .cc_index  = DL_TIMER_CC_0_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pins      = pwm2_pins,
    .iomuxes   = pwm2_iomuxes,
    .count     = 1
};
static const System::GPIO::GPIO pwm3_pins[] = { GateDriver3_PIN, LED2_PIN };
static const uint32_t pwm3_iomuxes[] = { IOMUX_PINCM11_PF_TIMG4_CCP0, IOMUX_PINCM27_PF_TIMG1_CCP0 };
const FWS_Utils::PWM::PWM PWM3 {
    .timer     = TIMG2,
    .cc_index  = DL_TIMER_CC_0_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP0_OUTPUT,
    .pins      = pwm3_pins,
    .iomuxes   = pwm3_iomuxes,
    .count     = 2
};
static const System::GPIO::GPIO pwm4_pins[] = { GateDriver4_PIN };
static const uint32_t pwm4_iomuxes[] = { IOMUX_PINCM12_PF_TIMG4_CCP1 };
const FWS_Utils::PWM::PWM PWM4 {
    .timer     = TIMG2,
    .cc_index  = DL_TIMER_CC_0_INDEX,
    .cc_output = GPTIMER_CCPD_C0CCP1_OUTPUT,
    .pins      = pwm4_pins,
    .iomuxes   = pwm4_iomuxes,
    .count     = 2
};

void init();
void run();
void fail_safe();
void turn_on_LEDs();
void LEDs_with_pwm();
void Test_PIN(System::GPIO::GPIO);

void killChip();

char uart_buffer[100];

int main(void) {
    init();
//    Test_PIN(LED2_PIN);
    LEDs_with_pwm();

//    killChip();
//    set_LEDs_with_pwm();
//    turn_on_LEDs();
//    run();
    fail_safe();
}

void Test_LED(System::GPIO::GPIO p_pin) {
    DL_GPIO_initDigitalOutput(p_pin.iomux);
    p_pin.set();
    while(true) {
        DL_GPIO_togglePins(GPIOPINPUX(p_pin));
        delay_cycles(32e6*3);
    };
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

void LEDs_with_pwm() {
    static double duty = 0;
    static double dir = 0.1;


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
        snprintf(ARRANDN(uart_buffer), "PWM Out: %u\n", FWS_Utils::PWM::PWM_output(PWM1));
        System::uart_ui.nputs(ARRANDN(uart_buffer));


        //setPWM2(duty * 0xFFFF);


        FWS_Utils::PWM::set_PWM_duty(PWM1, duty);
        FWS_Utils::PWM::set_PWM_duty(PWM3, duty);
        delay_cycles(System::CLK::CPUCLK * 0.5);
    }
}

void run() {
    double voltage = 3.3;

    while(1) {
        FWS_Utils::Motor::steer_motor();
        delay_cycles(System::CLK::CPUCLK);
    }
}

void init() {
    /*** PWM config *******************************************/
    /* PA26 used as PWM output. driven by TIMER-1 C0 */
    System::init();

    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    delay_cycles(POWER_STARTUP_DELAY);

    FWS_Utils::GPIO::INITGPIO(PWM1, 2);
    FWS_Utils::GPIO::INITGPIO(PWM3, 2);

//    FWS_Utils::GPIO::INITGPIO(LED2_PIN, PWM3);
//    IOMUX_PINCM25_PF_TIMG0_CCP1
//    FWS_Utils::InitGPIO(GateDriver_3,IOMUX_PINCM22_PF_TIMG2_CCP0);
//    FWS_Utils::InitGPIO(GateDriver_4,IOMUX_PINCM23_PF_TIMG2_CCP1);
//    DL_TIMER_CC_INDEX::DL_TIMER_CC_0_INDEX

    FWS_Utils::PWM::INITPWMTIMER(PWM1);
    FWS_Utils::PWM::INITPWMTIMER(PWM3);

    FWS_Utils::PID::INITPID(&pid);
    FWS_Utils::FWS::INITFWS(&fws);

    /**********************************************************/
}

void voltage_test() {
    char str[100];
    double voltage = 3.3;
    while(1){
       if(voltage > 0x7FF) {
           FWS_Utils::PWM::set_PWM_duty(PWM1, voltage);
       }
       FWS_Utils::PWM::set_PWM_duty(PWM1, FWS_Utils::PWM::PWMMAX);
       snprintf(ARRANDN(str), "PWM Out: %u\nVoltage: %f\n", FWS_Utils::PWM::PWM_output(PWM1), voltage);
       System::uart_ui.nputs(ARRANDN(str));
       delay_cycles(System::CLK::CPUCLK);
    }
}

void fail_safe() {
    while(true) {
       System::FailHard("reached end of main" NEWLINE);
       delay_cycles(20e6);
    }
}

void turn_on_LEDs() {
    System::init();
    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    DL_GPIO_initPeripheralOutputFunctionFeatures(
           IOMUX_PINCM1,
           IOMUX_PINCM1_PF_TIMG1_CCP0,
           DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
           DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
           DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
           DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
       );
    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM27);
    DL_GPIO_enableOutput(System::GPIO::PA0.port, System::GPIO::PA0.pin | System::GPIO::PA26.pin);
    while(1) {
        System::GPIO::PA1.set();
        System::GPIO::PA26.set();
        delay_cycles(System::CLK::CPUCLK/100);
    };
}
