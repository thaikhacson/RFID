#include "RFID.h"
#include "SPI.h"

void RFID_Init(void) {
	SPIx_Init();

	RFID_Reset();

	RFID_WriteRegister(MFRC522_REG_T_MODE, 0x8D);
	RFID_WriteRegister(MFRC522_REG_T_PRESCALER, 0x3E);
	RFID_WriteRegister(MFRC522_REG_T_RELOAD_L, 30);           
	RFID_WriteRegister(MFRC522_REG_T_RELOAD_H, 0);

	/* 48dB gain */
	RFID_WriteRegister(MFRC522_REG_RF_CFG, 0x70);
	
	RFID_WriteRegister(MFRC522_REG_TX_AUTO, 0x40);
	RFID_WriteRegister(MFRC522_REG_MODE, 0x3D);

	RFID_AntennaOn();		//Open the antenna
}

RFID_Status_t RFID_Check(uint8_t* id) {
	RFID_Status_t status;
	//Find cards, return card type
	status = RFID_Request(PICC_REQIDL, id);	
	if (status == MI_OK) {
		//Card detected
		//Anti-collision, return card serial number 4 bytes
		status = RFID_Anticoll(id);	
	}
	RFID_Halt();			//Command card into hibernation 

	return status;
}

RFID_Status_t RFID_Compare(uint8_t* CardID, uint8_t* CompareID) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareID[i]) {
			return MI_ERR;
		}
	}
	return MI_OK;
}

void RFID_WriteRegister(uint8_t addr, uint8_t val) {
	//CS low
	SPIx_EnableSlave();
	//Send address
	SPIx_Transfer((addr << 1) & 0x7E);
	//Send data	
	SPIx_Transfer(val);
	//CS high
	SPIx_DisableSlave();
}

uint8_t RFID_ReadRegister(uint8_t addr) {
	uint8_t val;
	//CS low
	SPIx_EnableSlave();

	SPIx_Transfer(((addr << 1) & 0x7E) | 0x80);	
	val = SPIx_Transfer(MFRC522_DUMMY);
	//CS high
	SPIx_DisableSlave();

	return val;	
}

void RFID_SetBitMask(uint8_t reg, uint8_t mask) {
	RFID_WriteRegister(reg, RFID_ReadRegister(reg) | mask);
}

void RFID_ClearBitMask(uint8_t reg, uint8_t mask){
	RFID_WriteRegister(reg, RFID_ReadRegister(reg) & (~mask));
} 

void RFID_AntennaOn(void) {
	uint8_t temp;

	temp = RFID_ReadRegister(MFRC522_REG_TX_CONTROL);
	if (!(temp & 0x03)) {
		RFID_SetBitMask(MFRC522_REG_TX_CONTROL, 0x03);
	}
}

void RFID_AntennaOff(void) {
	RFID_ClearBitMask(MFRC522_REG_TX_CONTROL, 0x03);
}

void RFID_Reset(void) {
	RFID_WriteRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE);
}

RFID_Status_t RFID_Request(uint8_t reqMode, uint8_t* TagType) {
	RFID_Status_t status;  
	uint16_t backBits;			//The received data bits

	RFID_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = RFID_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	if ((status != MI_OK) || (backBits != 0x10)) {    
		status = MI_ERR;
	}

	return status;
}

RFID_Status_t RFID_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen) {
	RFID_Status_t status = MI_ERR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;

	switch (command) {
		case PCD_AUTHENT: {
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE: {
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
	}

	RFID_WriteRegister(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
	RFID_ClearBitMask(MFRC522_REG_COMM_IRQ, 0x80);
	RFID_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);

	RFID_WriteRegister(MFRC522_REG_COMMAND, PCD_IDLE);

	//Writing data to the FIFO
	for (i = 0; i < sendLen; i++) {   
		RFID_WriteRegister(MFRC522_REG_FIFO_DATA, sendData[i]);    
	}

	//Execute the command
	RFID_WriteRegister(MFRC522_REG_COMMAND, command);
	if (command == PCD_TRANSCEIVE) {    
		RFID_SetBitMask(MFRC522_REG_BIT_FRAMING, 0x80);		//StartSend=1,transmission of data starts  
	}   

	//Waiting to receive data to complete
	i = 2000;	//i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
	do {
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = RFID_ReadRegister(MFRC522_REG_COMM_IRQ);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	RFID_ClearBitMask(MFRC522_REG_BIT_FRAMING, 0x80);			//StartSend=0

	if (i != 0)  {
		if (!(RFID_ReadRegister(MFRC522_REG_ERROR) & 0x1B)) {
			status = MI_OK;
			if (n & irqEn & 0x01) {   
				status = MI_NOTAGERR;			
			}

			if (command == PCD_TRANSCEIVE) {
				n = RFID_ReadRegister(MFRC522_REG_FIFO_LEVEL);
				lastBits = RFID_ReadRegister(MFRC522_REG_CONTROL) & 0x07;
				if (lastBits) {   
					*backLen = (n - 1) * 8 + lastBits;   
				} else {   
					*backLen = n * 8;   
				}

				if (n == 0) {   
					n = 1;    
				}
				if (n > MFRC522_MAX_LEN) {   
					n = MFRC522_MAX_LEN;   
				}

				//Reading the received data in FIFO
				for (i = 0; i < n; i++) {   
					backData[i] = RFID_ReadRegister(MFRC522_REG_FIFO_DATA);    
				}
			}
		} else {   
			status = MI_ERR;  
		}
	}

	return status;
}

RFID_Status_t RFID_Anticoll(uint8_t* serNum) {
	RFID_Status_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	RFID_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00);		//TxLastBists = BitFramingReg[2..0]

	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = RFID_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

	if (status == MI_OK) {
		//Check card serial number
		for (i = 0; i < 4; i++) {   
			serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i]) {   
			status = MI_ERR;    
		}
	}
	return status;
} 

void RFID_CalculateCRC(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData) {
	uint8_t i, n;

	RFID_ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04);			//CRCIrq = 0
	RFID_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);			//Clear the FIFO pointer
	//Write_MFRC522(CommandReg, PCD_IDLE);

	//Writing data to the FIFO	
	for (i = 0; i < len; i++) {   
		RFID_WriteRegister(MFRC522_REG_FIFO_DATA, *(pIndata+i));   
	}
	RFID_WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);

	//Wait CRC calculation is complete
	i = 0xFF;
	do {
		n = RFID_ReadRegister(MFRC522_REG_DIV_IRQ);
		i--;
	} while ((i!=0) && !(n&0x04));			//CRCIrq = 1

	//Read CRC calculation result
	pOutData[0] = RFID_ReadRegister(MFRC522_REG_CRC_RESULT_L);
	pOutData[1] = RFID_ReadRegister(MFRC522_REG_CRC_RESULT_M);
}

uint8_t RFID_SelectTag(uint8_t* serNum) {
	uint8_t i;
	RFID_Status_t status;
	uint8_t size;
	uint16_t recvBits;
	uint8_t buffer[9]; 

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) {
		buffer[i+2] = *(serNum+i);
	}
	RFID_CalculateCRC(buffer, 7, &buffer[7]);		//??
	status = RFID_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

	if ((status == MI_OK) && (recvBits == 0x18)) {   
		size = buffer[0]; 
	} else {   
		size = 0;    
	}

	return size;
}

RFID_Status_t RFID_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum) {
	RFID_Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[12]; 

	//Verify the command block address + sector + password + card serial number
	buff[0] = authMode;
	buff[1] = BlockAddr;
	for (i = 0; i < 6; i++) {    
		buff[i+2] = *(Sectorkey+i);   
	}
	for (i=0; i<4; i++) {    
		buff[i+8] = *(serNum+i);   
	}
	status = RFID_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

	if ((status != MI_OK) || (!(RFID_ReadRegister(MFRC522_REG_STATUS2) & 0x08))) {   
		status = MI_ERR;   
	}

	return status;
}

RFID_Status_t RFID_Read(uint8_t blockAddr, uint8_t* recvData) {
	RFID_Status_t status;
	uint16_t unLen;

	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	RFID_CalculateCRC(recvData,2, &recvData[2]);
	status = RFID_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

	if ((status != MI_OK) || (unLen != 0x90)) {
		status = MI_ERR;
	}

	return status;
}

RFID_Status_t RFID_Write(uint8_t blockAddr, uint8_t* writeData) {
	RFID_Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[18]; 

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	RFID_CalculateCRC(buff, 2, &buff[2]);
	status = RFID_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

	if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {   
		status = MI_ERR;   
	}

	if (status == MI_OK) {
		//Data to the FIFO write 16Byte
		for (i = 0; i < 16; i++) {    
			buff[i] = *(writeData+i);   
		}
		RFID_CalculateCRC(buff, 16, &buff[16]);
		status = RFID_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {   
			status = MI_ERR;   
		}
	}

	return status;
}

void RFID_Halt(void) {
	uint16_t unLen;
	uint8_t buff[4]; 

	buff[0] = PICC_HALT;
	buff[1] = 0;
	RFID_CalculateCRC(buff, 2, &buff[2]);

	RFID_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}