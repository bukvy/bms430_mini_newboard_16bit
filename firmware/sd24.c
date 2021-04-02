#include "msp430afe253.h"
#include "sd24.h"
#include "string.h"
#include "system.h"
#include "modbus_dta.h"

unsigned short acnt;
unsigned long aval[3];
  
void Init_SD24(void)
{
  volatile unsigned int i;                  // Use volatile to prevent removal
  SD24CTL = SD24REFON + SD24VMIDON + SD24SSEL0;          // 1.2V ref, SMCLK
  SD24CCTL0 = SD24SNGL + SD24GRP + SD24UNI;              // Group with CH1
  SD24INCTL0= SD24GAIN_1+SD24INCH_0;

  SD24CCTL1 = SD24SNGL + SD24GRP & ~SD24UNI | SD24DF; // Group with CH2  + SD24GRP
   SD24INCTL1= SD24GAIN_16 +SD24INCH_1 |SD24LSBACC ; //&  ~SD24LSBACC;//

//SD24CCTL1 = SD24SNGL + SD24GRP + SD24UNI;              // Group with CH2
//  SD24INCTL1= SD24GAIN_1+SD24INCH_1;
  SD24CCTL2 = SD24SNGL + SD24UNI;               // Enable interrupt
  SD24INCTL2= SD24GAIN_1+SD24INCH_2;
  for ( i = 0; i < 0x3600; i++){
	  WDT();             // Delay for 1.2V ref startup
  }
  SD24CCTL2 |= SD24SC;                      // Set bit to start conversion
}

const float t_data[35][2]={
    {125, 386.0},
    {120, 433.0},
    {115, 488.0},
    {110, 550.0 },
    {105, 622.0},   //5
    {100, 705.0},
    {95, 803.0},
    {90, 917.0 },
    {85, 1049.0 },
    {80, 1204.0 },  //10
    {75, 1387.0 },
    {70, 1603.0 },
    {65, 1856.0 },
    {60, 2159.0 },
    {55, 2525.0 },  //15
    {50, 2961.0 },
    {45, 3485.0 },
    {40, 4115.0 },
    {35, 4871.0 },
    {30, 5787.0 },  //20
    {25, 6899.0},
    {20, 8244.0},
    {15, 9880.0},
    {10, 11871.0},
    {5, 14287.0},  //25
    {0, 17217.0},
    {-5, 20769.0},
    {-10, 25039.0},
    {-15, 30096.0},
    {-20, 36030.0},  //30
    {-25, 42893.0},
    {-30, 50645.0},
    {-35, 59180.0},
    {-40, 65500.0},
    {-50, 65500.0}};   // In a whole 35




float TermoCalc(unsigned long U)
{
  int i;
  float U1,U2;
  float K1,K2,KK;
  if(U>389){
  for(i=0;i<28;i++){
    if(t_data[i][1]>U)break;
  }
  U1=t_data[i-1][1];
  U2=t_data[i][1];
  K2=(float)U2/t_data[i][0];
  K1=(float)U1/t_data[i-1][0];
  KK=K1+(K2-K1)*(U-U1)/(U2-U1);
  }else return 130;
  return (U/KK);
}

void GetADCValue(void)
{
  unsigned long temp=0;
  short temp_sh=0;
  if(((SD24CCTL2 & SD24IFG))&&((SD24CCTL1 & SD24IFG))&&((SD24CCTL0 & SD24IFG))){
    acnt++;
    aval[0] = aval[0] + SD24MEM0;           // Save CH0 results (clears IFG)
// old version    aval[1] = aval[1] + SD24MEM1;           // Save CH1 results (clears IFG)

    temp_sh=SD24MEM1;
    aval[1] = aval[1] + (long)(temp_sh);           // Save CH1 results (clears IFG)


    aval[2] = aval[2] + SD24MEM2;           // Save CH2 results (clears IFG)
    SD24CCTL2 |= SD24SC;                      // Set bit to start conversion
    if(acnt>128){
      p_HR->adc[0]=aval[0]/acnt;
      p_HR->adc[1]=aval[1]/acnt;
      p_HR->aV1 = (unsigned char)(p_HR->adc[0]*p_HR->kV[0].a+p_HR->kV[0].b-200);
      p_HR->aaV1 = (unsigned short)(p_HR->adc[0]*p_HR->kV[0].a+p_HR->kV[0].b);

      p_HR->aI1 = (unsigned char)(p_HR->adc[1]*p_HR->kV[1].a+p_HR->kV[1].b);           // Save CH1 results (clears IFG)
      temp = (aval[2]/acnt);//*p_HR->T1.a+p_HR->T1.b;           // Save CH2 results (clears IFG)
      p_HR->aT1 = (char)TermoCalc(temp);
      aval[0]=0;aval[1]=0;aval[2]=0;
      acnt=0;
    }
  }
}



#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SD24_VECTOR
__interrupt void SD24AISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(SD24_VECTOR))) SD24AISR (void)
#else
#error Compiler not supported!
#endif
{
      switch (SD24IV)
      {
      case 2:                                   // SD24MEM Overflow
        break;
      case 4:                                   // SD24MEM0 IFG
        aval[0] = aval[0] + SD24MEM0;
        break;
      case 6:                                   // SD24MEM1 IFG
        aval[1] = aval[1] + SD24MEM1;           // Save CH1 results (clears IFG)
        break;
      case 8:                                   // SD24MEM2 IFG
        acnt++;
        aval[0] = aval[0] + SD24MEM0;           // Save CH0 results (clears IFG)
        aval[1] = aval[1] + SD24MEM1;           // Save CH1 results (clears IFG)
        aval[2] = aval[2] + SD24MEM2;           // Save CH2 results (clears IFG)
        if(acnt>128){
          p_HR->aV1 = aval[0]/acnt;           // Save CH0 results (clears IFG)
          p_HR->aI1 = aval[1]/acnt;           // Save CH1 results (clears IFG)
          p_HR->aT1 = aval[2]/acnt;           // Save CH2 results (clears IFG)
          aval[0]=0;aval[1]=0;aval[2]=0;
          acnt=0;
        }
        
        break;
      }
}
