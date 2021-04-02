#ifndef _modbus_h
#define _modbus_h
#include "Serial.h";





#define MODBUS_ADDRESS 1
#define MODBUS_PACKET_DETECT_TIME 20L


#ifdef __cplusplus
extern "C" unsigned short ModBusCRC16(unsigned char* p, unsigned short n);
extern "C" unsigned char ModBusLRC(unsigned char *p, unsigned short n);
#else
unsigned short ModBusCRC16(unsigned char* p, unsigned short n);
unsigned char ModBusLRC(unsigned char *p, unsigned short n);
#endif


enum CMB_Functions {	
	
	mbf_Read_Holding_Registers	= 3,
	mbf_Read_Input_Register		= 4,
	mbf_Write_Single_Register	= 6,
	mbf_Read_Exception_Status	= 7,
	mbf_Write_Multiple_Registers= 16,
	mbf_Mask_Write_Register		= 22,
	mbf_Read_Write_Multiple_Registers= 23,
};

enum CMB_Exception_Codes {	
	mbexc_ILLEGAL_FUNCTION		= 0x01,
	mbexc_ILLEGAL_DATA_ADDRESS	= 0x02,
	mbexc_ILLEGAL_DATA_VALUE	= 0x03,
	mbexc_SLAVE_DEVICE_FAILURE	= 0x04,
	mbexc_ACKNOWLEDGE			= 0x05,
	mbexc_SLAVE_DEVICE_BUSY		= 0x06,
	mbexc_MEMORY_PARITY_ERROR	= 0x08,
	mbexc_GATEWAY_PATH_UNAVAILABLE= 0x0A,
	mbexc_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND= 0x0B
};


#ifndef ULONG
#define ULONG unsigned long
#endif

#ifndef USHORT
#define USHORT unsigned short
#endif

#ifndef SHORT
#define SHORT short
#endif

#ifndef UCHAR
#define UCHAR unsigned char
#endif


#ifndef DWORD
#define DWORD unsigned long
#endif


#ifndef HIBYTE
#define HIBYTE(arg) ( (unsigned char)(((arg)>>8)&0xFF) )
#define LOBYTE(arg) ( (unsigned char)((arg)&0xFF) )
#endif

#define MBF_HOLDREG_CNT (sizeof(CMODBUS_HR)/2+1)


#ifdef __cplusplus
extern "C" void fmb_OnCharacterOverrrun(void); // потери байт во время приема
extern "C" void fmb_OnReceiveError(void);		// ошибка в пакете
extern "C" void fmb_OnCompleteMBPacket(void);	//по завершению приема модбас-пакета,
extern "C" void ProcessModbus(void);
extern "C" unsigned char g_ModbusBuf[MB_BUF_SIZE];
extern "C" unsigned short g_wModbusCnt;
extern "C" unsigned char g_bThisModbusAddr;	// адрес данного устройства на шине Modbus
extern "C" unsigned short* gmbf_InputRegisters;// в main установить указатель
extern "C" unsigned short* gmbf_HoldingRegisters;// в main установить указатель



#else
void fmb_OnCharacterOverrrun(void); // потери байт во время приема
void fmb_OnReceiveError(void);		// ошибка в пакете
void fmb_OnCompleteMBPacket(void);	//по завершению приема модбас-пакета,
void ProcessModbus(void);
extern unsigned char g_ModbusBuf[MB_BUF_SIZE];
extern unsigned short g_wModbusCnt;
extern unsigned char g_bThisModbusAddr;	// адрес данного устройства на шине Modbus
extern unsigned short* gmbf_InputRegisters;// в main установить указатель
extern unsigned short* gmbf_HoldingRegisters;// в main установить указатель

#endif


#endif
