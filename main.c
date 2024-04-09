#include "stm32f10x.h"
#include "delay.h"
#include "RFID.h"
#include <stdio.h>

void My_GPIO_Init(void);

uint8_t CardID[5];
char szBuff[100];

int main() {
	DelayInit();
	My_GPIO_Init();
	RFID_Init();
	
	while(1) {
		if (RFID_Check(CardID) == MI_OK) {
			sprintf(szBuff, "ID: 0x%02X%02X%02X%02X%02X", CardID[0], CardID[1], CardID[2], CardID[3], CardID[4]);
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			DelayMs(1000);
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		}
	}
}

void My_GPIO_Init(void) {
	GPIO_InitTypeDef gpioInit;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	gpioInit.GPIO_Mode=GPIO_Mode_Out_PP;
	gpioInit.GPIO_Speed=GPIO_Speed_50MHz;
	gpioInit.GPIO_Pin=GPIO_Pin_13;
	GPIO_Init(GPIOC, &gpioInit);
}