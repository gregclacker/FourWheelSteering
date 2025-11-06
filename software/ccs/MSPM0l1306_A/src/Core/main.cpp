/*
 * c_I2C.cpp
 *
 *  Created on: Oct 24, 2025
 *      Author: FSAE
 */

#include "system.hpp"

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

System::GPIO::GPIO GPIO_LEDS_PORT = System::GPIO::PA1;
System::GPIO::GPIO GPIO_LEDS_USER_LED_1_PIN = System::GPIO::PA2;

uint8_t MCP3421_ADDR = 0b10001100u;
uint8_t configByte = 0x68;  // RDY=1, One-Shot, 18-bit, PGA x1

uint8_t adcData[3];
//1st write: 10001000

int main(void)
{
    System::init();

    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET CLIGOOD PROJECT_NAME "   " CLIRESET CLIHIGHLIGHT PROJECT_VERSION CLIRESET NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE CLIRESET));

//    char _write[30];
//    snprintf(ARRANDN(_write), "A");
    uint8_t write[] = {1,2,3};
    uint8_t read[10];
    System::i2c1.setSCLTarget(100e3);

    // Send the config byte to start conversion
    System::I2C::I2C::tx(MCP3421_ADDR, configByte);
    while (1) {
        System::i2c1.tx_blocking(MCP3421_ADDR, ARRANDN(write));
        System::i2c1.rx(MCP3421_ADDR, ARRANDN(read));
    }
    //System::I2C::I2C::tx(ADC12_0_INST, ARRANDN(_write));
    //System::uart_ui.nputs(ARRANDN(_write));
}
