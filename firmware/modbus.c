#include "msp430afe253.h"
#include "modbus.h"
#include "serial.h"
#include "string.h"
#include "system.h"
#include "modbus_dta.h"

#include "modbus.h"

unsigned short ModBusCRC16(unsigned char* p, unsigned short n)
{
	unsigned short w, i;
	unsigned char j;
	for (w = 0xFFFF, i=0; i<n; i++) {
		w ^= p[i];
		for (j=0;j<8; j++) if (w&1) { w >>= 1; w ^= 0xA001; } else w >>= 1;
	}
	return w;
}


//unsigned char g_bThisModbusAddr;	// адрес данного устройства на шине Modbus
unsigned char g_bModbusnInListenOnly;	// нахождение в режиме listen-only
unsigned char g_bThisModbusAddr;
//входной/выходной буфер для ModBus

unsigned short g_wModbusCnt;



//проверка RTU-пакета на корректность
unsigned char fmb_CheckRTU(void)
{
	unsigned short crc;
	if (g_wModbusCnt < 3) return 0;	//недостаточная длина для нормального пакета
	crc = ModBusCRC16(g_ModbusBuf, (unsigned short)(g_wModbusCnt-2));
	if ((LOBYTE(crc)!=g_ModbusBuf[g_wModbusCnt-2])
			|| (HIBYTE(crc)!=g_ModbusBuf[g_wModbusCnt-1]))
		return 0; //плохая CRC
	g_wModbusCnt -= 2; //CRC для обработки не нужна
	return 1;
}

//будет: DP, DPCH, P, T, Tкорп, зав.N, флаги сост-я

typedef unsigned char (* PMBFUNC)(void);	//указатель на выполняемую функцию, результат - код исключения

//Таблица поддерживаемых устройством функций MODBUS
typedef struct {
	unsigned char	bFuncId;		//номер функции
	unsigned char	bUseInBroadcast;//возможность вызова в броадкасте
	PMBFUNC			pProcessBody;	//указатель на тело обработчика
} SMBFunc;

//unsigned short gmbf_InputRegisters[MBF_INPREG_CNT];	//"входные" регистры
unsigned short* gmbf_InputRegisters;// в main установить указатель

//unsigned short gmbf_HoldingRegisters[MBF_HOLDREG_CNT];	//хранимые регистры
unsigned short* gmbf_HoldingRegisters;// в main установить указатель

unsigned short wRegStart, wRegCnt;

//чтение 16-битных регистров
unsigned char fmbk_ReadInputOrHoldRegister(unsigned short* pData, unsigned short wMaxVal)
{
	unsigned short i;
	//валидация
	if (wRegCnt<1 || wRegCnt>0x7D) return mbexc_ILLEGAL_DATA_VALUE;
	if ((wRegStart>=wMaxVal) || (wRegStart+wRegCnt>wMaxVal)) return mbexc_ILLEGAL_DATA_ADDRESS;
	//подготовка корректного ответа - складываем все в буфер
	for (i=0; i<wRegCnt; i++) {
		g_ModbusBuf[3+i*2] = HIBYTE(pData[i+wRegStart]);
		g_ModbusBuf[4+i*2] = LOBYTE(pData[i+wRegStart]);
	}
	g_ModbusBuf[2] = wRegCnt*2;	//кол-во байт данных
	g_wModbusCnt = wRegCnt*2 + 3;	//размер посылки
	return 0;
}

//чтение хранимых регистров
unsigned char fmbk_ReadHoldingRegister(void)
{
	return fmbk_ReadInputOrHoldRegister(gmbf_HoldingRegisters, MBF_HOLDREG_CNT);
}

//запись одного флага
unsigned char fmbk_WriteSingleRegister(void)
{
	//валидация
	if (wRegStart>=MBF_HOLDREG_CNT) return mbexc_ILLEGAL_DATA_ADDRESS;
	//осуществляем запись в регистр, готовим ответ
	gmbf_HoldingRegisters[wRegStart] = wRegCnt;
	g_wModbusCnt = 6;	//размер посылки
	return 0;
}

//запись регистров
unsigned char fmbk_WriteHoldingRegisters(void)
{
	unsigned short i;
	unsigned char Nn;
	//валидация
	Nn = g_ModbusBuf[6];	//количество байт в области данных
	if ((wRegCnt<1) || (wRegCnt>0x7B) || (Nn != g_wModbusCnt-7)) return mbexc_ILLEGAL_DATA_VALUE;
	if ((wRegStart>=MBF_HOLDREG_CNT) || (wRegStart+wRegCnt>MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
	//записываем регистры
	for (i=0; i<wRegCnt; i++) gmbf_HoldingRegisters[wRegStart+i] =
		(g_ModbusBuf[7+i*2]<<8)|g_ModbusBuf[8+i*2];
	g_wModbusCnt = 6;	//размер посылки
	return 0;
}

//запись регистров
unsigned char fmbk_ReadWriteHoldingRegisters(void)
{
	unsigned short i, wWriteStart, wWriteCnt;
	unsigned char Nn;
	//валидация
	wWriteStart = (g_ModbusBuf[6]<<8) | g_ModbusBuf[7];
	wWriteCnt = (g_ModbusBuf[8]<<8) | g_ModbusBuf[9];
	Nn = g_ModbusBuf[10];	//количество байт в области данных

	if ((wRegCnt<1) || (wRegCnt>0x7D)) return mbexc_ILLEGAL_DATA_VALUE;
	if ((wWriteCnt<1) || (wWriteCnt>0x79) || (Nn != g_wModbusCnt-11)) return mbexc_ILLEGAL_DATA_VALUE;
	if ((wRegStart>=MBF_HOLDREG_CNT) || (wRegStart+wRegCnt>MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
	if ((wWriteStart>=MBF_HOLDREG_CNT) || (wWriteStart+wWriteCnt>MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
	//записываем регистры
	for (i=0; i<wWriteCnt; i++)
		gmbf_HoldingRegisters[wWriteStart+i] = (g_ModbusBuf[11+i*2]>>8)|g_ModbusBuf[12+i*2];
	//читаем регистры
	for (i=0; i<wRegCnt; i++) {
		g_ModbusBuf[3+i*2] = HIBYTE(gmbf_HoldingRegisters[i+wRegStart]);
		g_ModbusBuf[4+i*2] = LOBYTE(gmbf_HoldingRegisters[i+wRegStart]);
	}
	g_ModbusBuf[2] = wRegCnt*2;	//кол-во байт данных
	g_wModbusCnt = wRegCnt*2 + 3;	//размер посылки
	return 0;
}

//количество имплементированных MODBUS функций
#define MBF_CNT 4

//список имплементированных MODBUS функций
const SMBFunc MBList[MBF_CNT] = {
	{mbf_Read_Holding_Registers, 0, &fmbk_ReadHoldingRegister},
	{mbf_Write_Single_Register, 0, &fmbk_WriteSingleRegister},
	{mbf_Write_Multiple_Registers, 0, &fmbk_WriteHoldingRegisters},
	{mbf_Read_Write_Multiple_Registers, 0, &fmbk_ReadWriteHoldingRegisters}
};

//обработка PDU, результат - код исключения (либо 0, если все OK)
// внутри: проверка на валидность PDU
// внутри: формировать пакет ответа
unsigned char fmb_ValidateAndProcessPDU(void)
{
	unsigned char i, nf;
	nf = 0xFF;
	for (i=0; i<MBF_CNT; i++) { //ищем функцию в списке
		if (MBList[i].bFuncId==g_ModbusBuf[1]) { nf = i; break;	}
	}
	if (nf==0xFF) return mbexc_ILLEGAL_FUNCTION;	//функция вообще не поддерживается

	wRegStart = (g_ModbusBuf[2]<<8) | g_ModbusBuf[3];	//аргументы для большинства функций
	wRegCnt = (g_ModbusBuf[4]<<8) | g_ModbusBuf[5];

	if ((MBList[nf].bUseInBroadcast==0) && (g_ModbusBuf[0]==0)) return mbexc_ILLEGAL_FUNCTION;	//функция не поддерживается в broadcast

	//если длина данных некорректна, R=mbexc_ILLEGAL_DATA_VALUE
	//если адреса регистров некорректны, R=mbexc_ILLEGAL_DATA_ADDRESS
	//если данные некорректны, R=mbexc_ILLEGAL_DATA_VALUE
	return (MBList[nf].pProcessBody)? (*MBList[nf].pProcessBody)() : mbexc_ILLEGAL_FUNCTION;	//либо выполним, либо мы еще не дописали функцию
}

//обработка небитого RTU
//после вызова - надо будет отослать пакет, если он непустой
void fmb_OnNormalRTU(void)
{
	unsigned char r;
	if ((g_ModbusBuf[0] == MODBUS_ADDRESS) || (g_ModbusBuf[0] == 0xFF)) { //сообщение этому устройству или broadcast
		r = fmb_ValidateAndProcessPDU(); //проверка на валидность и обработка
		if (r!=0) {	// таки исключение
			g_ModbusBuf[1] |= 0x80;	//согласно протоколу - код функции + 0x80
			g_ModbusBuf[2] = r;		//код исключения
			g_wModbusCnt = 3;		//количество байт в ответной посылке
		}
		if (g_ModbusBuf[0]==0) { //если это broadcast
			g_wModbusCnt = 0; //ГАСИМ ответ, т.к.посылать его не надо
		}
	} else g_wModbusCnt=0;
	if (g_wModbusCnt) { //пакет для отправки есть, добавим контрольную сумму
	  	unsigned short crc = ModBusCRC16(g_ModbusBuf, g_wModbusCnt);
		g_ModbusBuf[g_wModbusCnt++] = LOBYTE(crc);
		g_ModbusBuf[g_wModbusCnt++] = HIBYTE(crc);
	}
	//if (g_bModbusnInListenOnly) g_wModbusCnt=0;
}

void fmb_OnCompleteMBPacket(void)	//по завершению приема модбас-пакета,
{
	if (fmb_CheckRTU()){
	  fmb_OnNormalRTU();
	  }
	  else g_wModbusCnt = 0;
}


void ProcessModbus(void)
{
    if (U0.mode == UART_RECEIVE_MODE)
    {
      U0.mode = UART_TRANSMIT_MODE;
      fmb_OnCompleteMBPacket();
      if(g_ModbusBuf){
			  U0.Start_TX(g_ModbusBuf, g_wModbusCnt);
      }
    }
    return;
}
