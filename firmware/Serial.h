#ifndef __serial__h__
#define __serial__h__

enum uart_mode_t{
	UART_RECEIVE_MODE=0, UART_TRANSMIT_MODE=1, UART_BUSY_MODE=2, UART_TURN_MODE=3
};
#define MB_BUF_SIZE 120

typedef void (*SerialEvent)(unsigned char cnt);

typedef void (*SerialEventHandlerPtr)(void);

typedef unsigned char (*SerialEventHandlerPtr1)(void);

typedef struct
{
  unsigned char cnt;
  //unsigned char cnt_out;
  unsigned char ptr;
  //unsigned char ptr_out;
  unsigned char mode;
  unsigned char buf[MB_BUF_SIZE];
  //unsigned char buf_out[MB_BUF_SIZE];
  unsigned long rx_last_time;
  SerialEvent Start_TX;
  SerialEventHandlerPtr Stop_TX;
  SerialEventHandlerPtr1 Check_end_pack;
} CComBuf;

extern unsigned char addr;
extern unsigned char cmd_addr;
extern unsigned char cmd_param;
extern unsigned char cmd_param1;
extern unsigned char cmd;
extern unsigned long cmd_last_time;

extern unsigned char crc_rx;

extern CComBuf U0;

extern unsigned char* p_ser;
void Init_UART0(void);

#endif
