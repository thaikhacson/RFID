#ifndef _SPI_H
#define _SPI_H

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"

#define SPIx_RCC				RCC_APB2Periph_SPI1
#define SPIx						SPI1
#define SPI_GPIO_RCC		RCC_APB2Periph_GPIOA
#define SPI_GPIO				GPIOA
#define SPI_PIN_MOSI		GPIO_Pin_7
#define SPI_PIN_MISO		GPIO_Pin_6
#define SPI_PIN_SCK			GPIO_Pin_5
#define SPI_PIN_SS			GPIO_Pin_4

void SPIx_Init(void);
uint8_t SPIx_Transfer(uint8_t data);
void SPIx_EnableSlave(void);
void SPIx_DisableSlave(void);


#endif