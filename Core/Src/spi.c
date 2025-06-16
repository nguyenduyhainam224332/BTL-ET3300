#include "main.h"
#include "spi.h"
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

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

uint8_t TxBuffer[buffer_length];
uint8_t RxBuffer[buffer_length];

void start_timers() {
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim4);
}

void gpio_init (void) {
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();

	  /*Configure GPIO pin : NSS_Pin */
	  GPIO_InitStruct.Pin = NSS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : SCK_Pin PA6 */
	  GPIO_InitStruct.Pin = SCK_Pin|MOSI_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA7 */
	  GPIO_InitStruct.Pin = MISO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  GPIOA->BSRR = NSS_Pin;             // NSS high (idle)
	  GPIOA->BSRR = SCK_Pin << 16u;      // SCK low
	  GPIOA->BSRR = MOSI_Pin << 16u;      //MOSI low

}

void SPI_DELAY(uint32_t cycles) {
    while (cycles--) {
        __NOP();
    }
}

void transmit_one_byte (uint8_t data) {

	    for (int i = 7; i >= 0; --i) {
	        // Set MOSI
	        if (data & (1 << i))
	            GPIOA->BSRR = MOSI_Pin;
	        else
	            GPIOA->BSRR = MOSI_Pin << 16u;

	        SPI_DELAY(1);

	        // SCK rising edge (data sampled by slave here in SPI mode 0)
	        GPIOA->BSRR = SCK_Pin;
	        SPI_DELAY(1);

	        // SCK falling edge
	        GPIOA->BSRR = SCK_Pin << 16u;
	        SPI_DELAY(1);

	    }

}


void transmit_data (uint8_t* buffer, uint16_t length) {
    // Pull NSS low to select the slave
    GPIOA->BSRR = NSS_Pin << 16u;  // NSS = 0
    SPI_DELAY(1);

    for (uint16_t i = 0; i < length; ++i) {
        transmit_one_byte(buffer[i]);
    }

    // Pull NSS high to deselect the slave
    GPIOA->BSRR = NSS_Pin;
    SPI_DELAY(4);
}


uint8_t receive_one_byte (void) {
	uint8_t received = 0;

	for (int i = 7; i>= 0; --i) {
		GPIOA->BSRR = SCK_Pin;
		SPI_DELAY(1);

		if (GPIOA->IDR & MISO_Pin)
						received |= (1 << i);

		GPIOA->BSRR = SCK_Pin << 16u;
		SPI_DELAY(1);

	}

	return received;
}

void receive_data (uint8_t* buffer, uint16_t length) {
	for (uint16_t i = 0; i < length; ++i) {
	       buffer[i] = receive_one_byte();
	}
}

void transmit_receive_data (uint8_t* buffer1, uint8_t* buffer2, uint16_t length1, uint16_t length2) {
    // Pull NSS low to select the slave
    GPIOA->BSRR = NSS_Pin << 16u;  // NSS = 0

    SPI_DELAY(1);

    for (uint16_t i = 0; i < length1; ++i) {
        transmit_one_byte(buffer1[i]);
    }

    for (uint16_t i = 0; i < length2; ++i) {
    	       buffer2[i] = receive_one_byte();
    }

    // Pull NSS high to deselect the slave
    GPIOA->BSRR = NSS_Pin;
    SPI_DELAY(4);
}


void readID (void) {
	TxBuffer[0] = 0x90;
	TxBuffer[1] = 0x00;
	TxBuffer[2] = 0x00;
	TxBuffer[3] = 0x00;
	transmit_receive_data(TxBuffer, RxBuffer, 4, 2);
}

void writeEnable (void) {
	TxBuffer[0] = 0x06;
	transmit_data(TxBuffer, 1);
}

void writeDisable (void) {
	TxBuffer[0] = 0x04;
	transmit_data(TxBuffer, 1);
}

char* readData (uint32_t page, uint8_t offset, uint16_t length) {
	static char outputString[257];
	if (length <= 256) {
	uint32_t readAddress = page*256 + offset;
	TxBuffer[0] = 0x03;
	TxBuffer[1] = (readAddress >> 16)&0xFF;
	TxBuffer[2] = (readAddress >> 8)&0xFF;
	TxBuffer[3] = (readAddress)&0xFF;
	transmit_receive_data (TxBuffer, RxBuffer, 4, length);
	}
	uint8_t i = 0;
	for (i = 0; i < length && RxBuffer[i] != 0xFF; ++i)
	        outputString[i] = RxBuffer[i];
	outputString[i] = '\0';
	return outputString;
}

char* fastRead (uint32_t page, uint8_t offset, uint16_t length) {
	static char outputString[257];
	if (length <= 256) {
	uint32_t readAddress = page*256 + offset;
	TxBuffer[0] = 0x0B;
	TxBuffer[1] = (readAddress >> 16)&0xFF;
	TxBuffer[2] = (readAddress >> 8)&0xFF;
	TxBuffer[3] = (readAddress)&0xFF;
	TxBuffer[4] = 0x00;
	transmit_receive_data (TxBuffer, RxBuffer, 5, length); }
	uint8_t i = 0;
		for (i = 0; i < length && RxBuffer[i] != 0xFF; ++i)
		        outputString[i] = RxBuffer[i];
		outputString[i] = '\0';
	return outputString;
}

void sectorErase (uint16_t sector) {
	writeEnable();
	TxBuffer[0] = 0x20;
	uint32_t sectorAddress = sector*16*256;
	TxBuffer[1] = (sectorAddress >> 16)&0xFF;
	TxBuffer[2] = (sectorAddress >> 8)&0xFF;
	TxBuffer[3] = (sectorAddress)&0xFF;
	transmit_data(TxBuffer, 4);
	SPI_DELAY(15000000);
	writeDisable();
}


void writeData (uint32_t page, uint8_t offset, const char* inputString) {
	uint16_t sector = page/16;
	sectorErase(sector);
	writeEnable();
	TxBuffer[0] = 0x02;
	uint32_t writeAddress = page*256+offset;
	TxBuffer[1] = (writeAddress >> 16)&0xFF;
	TxBuffer[2] = (writeAddress >> 8)&0xFF;
	TxBuffer[3] = (writeAddress)&0xFF;
	uint8_t bytestoWrite = 252 - offset;
	uint8_t len = strlen(inputString);
    if (len > bytestoWrite) len = bytestoWrite; // Limit to available space
    for (uint8_t i = 0; i < len; ++i) {
        TxBuffer[i + 4] = (uint8_t)inputString[i];
    }
    transmit_data(TxBuffer, len + 4);
    SPI_DELAY(250000);
	writeDisable();
}
