#include "msp430afe253.h"
#include "Serial.h"
#include "string.h"
#include "system.h"

CComBuf U0;
unsigned long cmd_last_time=0;
unsigned char bbrrbb;
unsigned char addr=0;
unsigned char cmd=0;
unsigned char cmd_addr=0;
unsigned char cmd_param=0;
unsigned char cmd_param1=0;

    
unsigned char crc_rx;
///**************************************************
SerialEventHandlerPtr Rx0EventHandlerPtr;
SerialEventHandlerPtr Tx0EventHandlerPtr;


void Start_TX0(unsigned char cnt)
{
    U0.ptr = 1;
    U0.mode = UART_TRANSMIT_MODE;
    U0.cnt = cnt;
    IFG1 &= ~UTXIFG0;
    TXBUF0 = U0.buf[0];                          // RXBUF0 to TXBUF0
    IE1 |= UTXIE0;	// tx interrupt disable
}

unsigned char  check0 (void)
{
    return (UTCTL0 & TXEPT) == TXEPT;
}


void Stop_TX0(void)
{
    U0.mode = UART_RECEIVE_MODE;
}

unsigned char* p_ser;

void error(void)
{
  P2OUT=BIT0;
}
/// <summary>
/// Обработчик события по приёму UART1.
/// </summary>
void Rx0EventHandler(void)
{
    unsigned char b = U0RXBUF;
    IFG1 &= (~URXIFG0);

    U0.rx_last_time = GlobalTime;
    if ((URCTL0 & 0x01) == 0x01 && (U0CTL & SYNC) != SYNC)
    {
        Init_UART0();
        return; // error
    }


    if (U0.cnt >= MB_BUF_SIZE) U0.cnt=0;
    U0.buf[U0.cnt++] = b;                       // нормальное добавление в буфер
    //if((U0.cnt>0)&&(U0.buf[0]==1))return;
    cmd_last_time=GetTickCount();
/*
    if(U0.cnt==1){
      crc_tx=0;
      crc_rx=0;
      crcr=1;
      if(U0.buf[0]<6)cmd=U0.buf[0]; else ldev=U0.buf[0]-4;
    }

    if(U0.cnt<U0.buf[1])crc_rx +=b;
*/   
/* Обработка команды адресации 
    if((U0.cnt==3)&&(cmd==1)){
      
      b++;addr=b;
      cmd_addr=(addr-1)/8;
      cmd_param1=(addr-1)-cmd_addr*8;
      cmd_last_time=GetTickCount();
      
    }*/
/*END Обработка команды адресации */
/*
    if((U0.cnt<U0.buf[1])||(U0.cnt<3))
      crc_tx +=b;
  */  
/*    if(U0.cnt==U0.buf[1]){
      if((b==crc_rx)&&(cmd!=1)&&(((ldev)==(addr))||(addr==1))){  
        if((cmd==2)&&(ldev==0))cmd_param=temp_cmd;
        if((cmd==4))cmd_param=U0.buf[1];
        cmd|=0x80;
      }
      temp_cmd=0;
      U0.cnt=0;
      ldev=0;
      crc_tx=0;
      crc_rx=0;
    }*/
      
    //TXBUF0=b;
}


/// <summary>
/// Обработчик события по передаче UART1.
/// </summary>
void Tx0EventHandler(void)
{
    U0.rx_last_time = GlobalTime;
    if ((U0.ptr == U0.cnt) || (U0.mode != UART_TRANSMIT_MODE))
    {
        //отправной пакет кончился или зашли по ошибке
        //if( U0.mode == UART_TRANSMIT_MODE)

        /*if(addr){P2OUT&=~BIT0;U0.mode=UART_BUSY_MODE;}//else P2OUT|=BIT0;
        else*/ 
        U0.mode = UART_RECEIVE_MODE;
        U0.cnt = 0;
        U0.ptr = 0;
        IE1 &= ~UTXIE0;
    }
    else
    {
        TXBUF0 = U0.buf[U0.ptr++];
    }


}

void Init_UART0(void)
{
    Rx0EventHandlerPtr = Rx0EventHandler;
    Tx0EventHandlerPtr = Tx0EventHandler;
    IE1 &= ~UTXIE0;	// tx interrupt disable
    IE1 &= ~URXIE0;	// rx interrupt disable


    P1SEL |= BIT3+BIT4;

    ME1 |= UTXE0 + URXE0;		// module enable register, UTXE0, URXE0

    UCTL0 =  CHAR + SWRST;		// SWRST=1, CHAR = 1: 8-bit data
    __no_operation();
    __no_operation();
    __no_operation();
    UCTL0 =  CHAR;		// SWRST=0, CHAR = 1: 8-bit data
    UTCTL0 = BIT4 + BIT5;	// The receive-start edge-control bit, SMCLK
    U0MCTL = 0x00;
    U0BR0 = 0x40;	//0x180 9600
    U0BR1 = 0x00;       //0x20 115200

    IE1 |= URXIE0;	// interrupt enable
    U0.Start_TX = Start_TX0;
    U0.Stop_TX = Stop_TX0;
    U0.mode = UART_RECEIVE_MODE;
    U0.Check_end_pack = check0;
    bbrrbb = U0RXBUF;	// clr all flags
    U0.cnt = 0;
    //U0.cnt_in = 0;
    U0.ptr = 0;
    //U0.ptr_in = 0;
}

#pragma vector=USART0RX_VECTOR
__interrupt void RX0_interrupt_handler(void)
{
    Rx0EventHandlerPtr();
}

#pragma vector=USART0TX_VECTOR
__interrupt void TX0_interrupt_handler(void)
{
    Tx0EventHandlerPtr();

}
