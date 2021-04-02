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


//unsigned char g_bThisModbusAddr;	// ����� ������� ���������� �� ���� Modbus
unsigned char g_bModbusnInListenOnly;	// ���������� � ������ listen-only
unsigned char g_bThisModbusAddr;
//�������/�������� ����� ��� ModBus

unsigned short g_wModbusCnt;



//�������� RTU-������ �� ������������
unsigned char fmb_CheckRTU(void)
{
	unsigned short crc;
	if (g_wModbusCnt < 3) return 0;	//������������� ����� ��� ����������� ������
	crc = ModBusCRC16(g_ModbusBuf, (unsigned short)(g_wModbusCnt-2));
	if ((LOBYTE(crc)!=g_ModbusBuf[g_wModbusCnt-2])
			|| (HIBYTE(crc)!=g_ModbusBuf[g_wModbusCnt-1]))
		return 0; //������ CRC
	g_wModbusCnt -= 2; //CRC ��� ��������� �� �����
	return 1;
}

//�����: DP, DPCH, P, T, T����, ���.N, ����� ����-�

typedef unsigned char (* PMBFUNC)(void);	//��������� �� ����������� �������, ��������� - ��� ����������

//������� �������������� ����������� ������� MODBUS
typedef struct {
	unsigned char	bFuncId;		//����� �������
	unsigned char	bUseInBroadcast;//����������� ������ � ����������
	PMBFUNC			pProcessBody;	//��������� �� ���� �����������
} SMBFunc;

//unsigned short gmbf_InputRegisters[MBF_INPREG_CNT];	//"�������" ��������
unsigned short* gmbf_InputRegisters;// � main ���������� ���������

//unsigned short gmbf_HoldingRegisters[MBF_HOLDREG_CNT];	//�������� ��������
unsigned short* gmbf_HoldingRegisters;// � main ���������� ���������

unsigned short wRegStart, wRegCnt;

//������ 16-������ ���������
unsigned char fmbk_ReadInputOrHoldRegister(unsigned short* pData, unsigned short wMaxVal)
{
	unsigned short i;
	//���������
	if (wRegCnt<1 || wRegCnt>0x7D) return mbexc_ILLEGAL_DATA_VALUE;
	if ((wRegStart>=wMaxVal) || (wRegStart+wRegCnt>wMaxVal)) return mbexc_ILLEGAL_DATA_ADDRESS;
	//���������� ����������� ������ - ���������� ��� � �����
	for (i=0; i<wRegCnt; i++) {
		g_ModbusBuf[3+i*2] = HIBYTE(pData[i+wRegStart]);
		g_ModbusBuf[4+i*2] = LOBYTE(pData[i+wRegStart]);
	}
	g_ModbusBuf[2] = wRegCnt*2;	//���-�� ���� ������
	g_wModbusCnt = wRegCnt*2 + 3;	//������ �������
	return 0;
}

//������ �������� ���������
unsigned char fmbk_ReadHoldingRegister(void)
{
	return fmbk_ReadInputOrHoldRegister(gmbf_HoldingRegisters, MBF_HOLDREG_CNT);
}

//������ ������ �����
unsigned char fmbk_WriteSingleRegister(void)
{
	//���������
	if (wRegStart>=MBF_HOLDREG_CNT) return mbexc_ILLEGAL_DATA_ADDRESS;
	//������������ ������ � �������, ������� �����
	gmbf_HoldingRegisters[wRegStart] = wRegCnt;
	g_wModbusCnt = 6;	//������ �������
	return 0;
}

//������ ���������
unsigned char fmbk_WriteHoldingRegisters(void)
{
	unsigned short i;
	unsigned char Nn;
	//���������
	Nn = g_ModbusBuf[6];	//���������� ���� � ������� ������
	if ((wRegCnt<1) || (wRegCnt>0x7B) || (Nn != g_wModbusCnt-7)) return mbexc_ILLEGAL_DATA_VALUE;
	if ((wRegStart>=MBF_HOLDREG_CNT) || (wRegStart+wRegCnt>MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
	//���������� ��������
	for (i=0; i<wRegCnt; i++) gmbf_HoldingRegisters[wRegStart+i] =
		(g_ModbusBuf[7+i*2]<<8)|g_ModbusBuf[8+i*2];
	g_wModbusCnt = 6;	//������ �������
	return 0;
}

//������ ���������
unsigned char fmbk_ReadWriteHoldingRegisters(void)
{
	unsigned short i, wWriteStart, wWriteCnt;
	unsigned char Nn;
	//���������
	wWriteStart = (g_ModbusBuf[6]<<8) | g_ModbusBuf[7];
	wWriteCnt = (g_ModbusBuf[8]<<8) | g_ModbusBuf[9];
	Nn = g_ModbusBuf[10];	//���������� ���� � ������� ������

	if ((wRegCnt<1) || (wRegCnt>0x7D)) return mbexc_ILLEGAL_DATA_VALUE;
	if ((wWriteCnt<1) || (wWriteCnt>0x79) || (Nn != g_wModbusCnt-11)) return mbexc_ILLEGAL_DATA_VALUE;
	if ((wRegStart>=MBF_HOLDREG_CNT) || (wRegStart+wRegCnt>MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
	if ((wWriteStart>=MBF_HOLDREG_CNT) || (wWriteStart+wWriteCnt>MBF_HOLDREG_CNT)) return mbexc_ILLEGAL_DATA_ADDRESS;
	//���������� ��������
	for (i=0; i<wWriteCnt; i++)
		gmbf_HoldingRegisters[wWriteStart+i] = (g_ModbusBuf[11+i*2]>>8)|g_ModbusBuf[12+i*2];
	//������ ��������
	for (i=0; i<wRegCnt; i++) {
		g_ModbusBuf[3+i*2] = HIBYTE(gmbf_HoldingRegisters[i+wRegStart]);
		g_ModbusBuf[4+i*2] = LOBYTE(gmbf_HoldingRegisters[i+wRegStart]);
	}
	g_ModbusBuf[2] = wRegCnt*2;	//���-�� ���� ������
	g_wModbusCnt = wRegCnt*2 + 3;	//������ �������
	return 0;
}

//���������� ������������������ MODBUS �������
#define MBF_CNT 4

//������ ������������������ MODBUS �������
const SMBFunc MBList[MBF_CNT] = {
	{mbf_Read_Holding_Registers, 0, &fmbk_ReadHoldingRegister},
	{mbf_Write_Single_Register, 0, &fmbk_WriteSingleRegister},
	{mbf_Write_Multiple_Registers, 0, &fmbk_WriteHoldingRegisters},
	{mbf_Read_Write_Multiple_Registers, 0, &fmbk_ReadWriteHoldingRegisters}
};

//��������� PDU, ��������� - ��� ���������� (���� 0, ���� ��� OK)
// ������: �������� �� ���������� PDU
// ������: ����������� ����� ������
unsigned char fmb_ValidateAndProcessPDU(void)
{
	unsigned char i, nf;
	nf = 0xFF;
	for (i=0; i<MBF_CNT; i++) { //���� ������� � ������
		if (MBList[i].bFuncId==g_ModbusBuf[1]) { nf = i; break;	}
	}
	if (nf==0xFF) return mbexc_ILLEGAL_FUNCTION;	//������� ������ �� ��������������

	wRegStart = (g_ModbusBuf[2]<<8) | g_ModbusBuf[3];	//��������� ��� ����������� �������
	wRegCnt = (g_ModbusBuf[4]<<8) | g_ModbusBuf[5];

	if ((MBList[nf].bUseInBroadcast==0) && (g_ModbusBuf[0]==0)) return mbexc_ILLEGAL_FUNCTION;	//������� �� �������������� � broadcast

	//���� ����� ������ �����������, R=mbexc_ILLEGAL_DATA_VALUE
	//���� ������ ��������� �����������, R=mbexc_ILLEGAL_DATA_ADDRESS
	//���� ������ �����������, R=mbexc_ILLEGAL_DATA_VALUE
	return (MBList[nf].pProcessBody)? (*MBList[nf].pProcessBody)() : mbexc_ILLEGAL_FUNCTION;	//���� ��������, ���� �� ��� �� �������� �������
}

//��������� �������� RTU
//����� ������ - ���� ����� �������� �����, ���� �� ��������
void fmb_OnNormalRTU(void)
{
	unsigned char r;
	if ((g_ModbusBuf[0] == MODBUS_ADDRESS) || (g_ModbusBuf[0] == 0xFF)) { //��������� ����� ���������� ��� broadcast
		r = fmb_ValidateAndProcessPDU(); //�������� �� ���������� � ���������
		if (r!=0) {	// ���� ����������
			g_ModbusBuf[1] |= 0x80;	//�������� ��������� - ��� ������� + 0x80
			g_ModbusBuf[2] = r;		//��� ����������
			g_wModbusCnt = 3;		//���������� ���� � �������� �������
		}
		if (g_ModbusBuf[0]==0) { //���� ��� broadcast
			g_wModbusCnt = 0; //����� �����, �.�.�������� ��� �� ����
		}
	} else g_wModbusCnt=0;
	if (g_wModbusCnt) { //����� ��� �������� ����, ������� ����������� �����
	  	unsigned short crc = ModBusCRC16(g_ModbusBuf, g_wModbusCnt);
		g_ModbusBuf[g_wModbusCnt++] = LOBYTE(crc);
		g_ModbusBuf[g_wModbusCnt++] = HIBYTE(crc);
	}
	//if (g_bModbusnInListenOnly) g_wModbusCnt=0;
}

void fmb_OnCompleteMBPacket(void)	//�� ���������� ������ ������-������,
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
