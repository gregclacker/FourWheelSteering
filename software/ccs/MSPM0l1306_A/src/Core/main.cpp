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

FWS_Utils::FWS fws;
FWS_Utils::PID::PID pid;

void init();
void run();
void fail_safe();
void turn_on_LEDs();
void set_LEDs_with_pwm();

char uart_buffer[100];

int main(void) {
    init();
//    set_LEDs_with_pwm();
//    turn_on_LEDs();
//    run();
    fail_safe();
}

void set_LEDs_with_pwm() {
    static double duty = 0;
    static double dir = 0.01;
    while(1) {
        //int duty_cycle = FWS_Utils::PWM::get_PWM_duty_cycle(FWS_Utils::ADC::get_ADC_voltage(ADC_1_ADDR));
        duty += dir;
        if(duty > 1) {
            duty = 1;
            dir = -0.01;
        }
        if(duty < 0) {
            duty = 0;
            dir = 0.01;
        }
        FWS_Utils::PWM::set_PWM(PWMTIMER1_REG, duty, DL_TIMER_CC_0_INDEX);
        FWS_Utils::PWM::set_PWM(PWMTIMER2_REG, duty, DL_TIMER_CC_1_INDEX);
        delay_cycles(System::CLK::CPUCLK/20);
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

    FWS_Utils::GPIO::INITGPIO(LED1_PIN, IOMUX_PINCM24_PF_TIMG0_CCP0);
    FWS_Utils::GPIO::INITGPIO(LED2_PIN, IOMUX_PINCM24_PF_TIMG0_CCP1);
//    IOMUX_PINCM25_PF_TIMG0_CCP1
//    FWS_Utils::InitGPIO(GateDriver_3,IOMUX_PINCM22_PF_TIMG2_CCP0);
//    FWS_Utils::InitGPIO(GateDriver_4,IOMUX_PINCM23_PF_TIMG2_CCP1);
//    DL_TIMER_CC_INDEX::DL_TIMER_CC_0_INDEX

    FWS_Utils::PWM::INITPWMTIMER(PWMTIMER1_REG, DL_TIMER_CC_0_INDEX, GPTIMER_CCPD_C0CCP0_OUTPUT);
    FWS_Utils::PWM::INITPWMTIMER(PWMTIMER2_REG, DL_TIMER_CC_1_INDEX, GPTIMER_CCPD_C0CCP1_OUTPUT);

//    FWS_Utils::PID::InitPID(&pid);
//    FWS_Utils::InitFWS(&fws);

    /**********************************************************/
}

void voltage_test() {
    char str[100];
    double voltage = 3.3;
    while(1){
       if(voltage > 0x7FF) {
           FWS_Utils::PWM::set_PWM(PWMTIMER1_REG, voltage, DL_TIMER_CC_0_INDEX);
       }
       FWS_Utils::PWM::set_PWM(PWMTIMER1_REG, PWMMAX, DL_TIMER_CC_0_INDEX);
       snprintf(ARRANDN(str), "PWM Out: %u\nVoltage: %f\n", FWS_Utils::PWM::get_PWM(PWMTIMER1_REG), voltage);
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
