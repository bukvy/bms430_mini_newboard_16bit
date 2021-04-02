#include "msp430afe253.h"
#include "system.h"
#include "serial.h"
#include "modbus_dta.h"



unsigned long GlobalTime=0;

#define _DINT() __bic_SR_register(GIE); 
#define _EINT() __bis_SR_register(GIE); 

void Sleep(unsigned long ms)
{
  unsigned long dt=GetTickCount();
  while ((GetTickCount()-dt)<ms ) ;
}

void WDT(void)
{
  WDTCTL = WDTPW + WDTCNTCL;
}

unsigned long GetTickCount(void)
{
  unsigned long rez;
  _DINT();// надо оптиимзировать - запрещать не все прерывания, а только таймерное
  rez=GlobalTime;
  _EINT();
  return rez;
}

void Init_SYS(void)
{
  // Stop WDT
  //WDTCTL = WDTPW + WDTHOLD;   
  WDTCTL = WDTPW + WDTCNTCL;
  
  // Start XT2
  unsigned int i;
  BCSCTL1 &= ~XT2OFF;  // XT2 = HF XTAL
  do
  {
    IFG1 &= ~OFIFG;                       // Clear OSCFault flag
    for (i = 0xFF; i > 0; i--);           // Time for flag to set
  } while ((IFG1 & OFIFG) != 0);          // OSCFault flag still set?  //
  BCSCTL2 = SELM1+SELS;
    
  // I/O 
  P1SEL=0;
  P2DIR |= /*BIT2 +*/ BIT0 /*+ BIT0*/;              
  P1DIR |= /*BIT2 +*/ BIT1 + BIT0;              
  //P1SEL |=BIT1; 
  P1OUT=0;
  P2OUT=BIT0;
  
  // Timer A как основная временная сетка
  TACTL = TAIE+TASSEL_2 + TACLR;              // SMCLK, Clear Tar, IE
  CCTL0 = CCIE;                       //
  //CCTL1 = OUTMOD_4 + CCIE;                  // CCR1 toggle, interrupt enabled
  TACTL |= MC_2;                              // mc_2= continuous up to 0xffff Start Timer_A in UP to CCR0
   
}

//3.6864
#pragma vector=TIMERA0_VECTOR
__interrupt void TIMERA0_VECTOR_handler(void)
{
  CCR0 += 36864;
  //CCR0 += 7373;
  //CCR0 += 20000;
  GlobalTime+=10;
  //GlobalTime+=2;
  //TACTL&=(~TAIFG);
}

//#define CCRR 3686400/(2*p_HR->R1)
//#define CCRR 2000000/(2*p_HR->hz)
// Timer_A1 Interrupt Vector (TAIV) handler
#pragma vector=TIMERA1_VECTOR
__interrupt void Timer_A1(void)
{
  switch( TAIV )
  {
  case  2: {
      //CCR1+= CCRR;                    // Add Offset to CCR1
    break;}
  case 10: P1OUT ^= BIT0;                   // Timer_A3 overflow
           break;
 }
}
