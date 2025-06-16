/*
 * spi.h
 *
 *  Created on: Jun 13, 2025
 *      Author: VBK computer
 */

#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include <string.h>

#define NSS_Pin GPIO_PIN_4
#define NSS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_6
#define MISO_Pin GPIO_PIN_7

#define SET_NSS_HIGH()    (GPIOA->BSRR = NSS_Pin)
#define SET_NSS_LOW()     (GPIOA->BSRR = NSS_Pin << 16u)

#define SET_SCK_HIGH()    (GPIOA->BSRR = SCK_Pin)
#define SET_SCK_LOW()     (GPIOA->BSRR = SCK_Pin << 16u)

#define SET_MOSI_HIGH()   (GPIOA->BSRR = MOSI_Pin)
#define SET_MOSI_LOW()    (GPIOA->BSRR = MOSI_Pin << 16u)

#define MISO_HIGH ((GPIOA->IDR & MISO_Pin) == MISO_Pin)
#define MISO_LOW ((GPIOA->IDR & MISO_Pin) == 0)

#define buffer_length 256

extern uint8_t TxBuffer[buffer_length];
extern uint8_t RxBuffer[buffer_length];


void start_timers();
void gpio_init(void);
void SPI_DELAY(uint32_t cycles);
void transmit_one_byte (uint8_t data);
void transmit_data (uint8_t* buffer, uint16_t length);
uint8_t receive_one_byte (void);
void receive_data (uint8_t* buffer, uint16_t length);
void transmit_receive_data (uint8_t* buffer1, uint8_t* buffer2, uint16_t length1, uint16_t length2);
void readID (void);
void writeEnable (void);
void writeDisable (void);
char* readData (uint32_t page, uint8_t offset, uint16_t length);
char* fastRead (uint32_t page, uint8_t offset, uint16_t length);
void sectorErase (uint16_t sector);
void writeData (uint32_t page, uint8_t offset, const char* inputString);



#endif /* INC_SPI_H_ */
