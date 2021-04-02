#include "msp430afe253.h"
#include "Serial.h"
#include "string.h"
#include "system.h"
#include "modbus_dta.h"
#include "sd24.h"


#define aU 0.083f
#define bU 24.0f

#define aT 0.101f
#define bT 7.5f

void write_flash(unsigned char *value);

float calc_a(float x1, float y1, float x2, float y2);  // x1-adc y1-real value
float calc_b(float x1, float y1, float coeff_a); // x1 adc  y1 real value
float for_calib_x1;
float for_calib_y1;
float for_calib_y2;


unsigned char ModBusLRC(unsigned char *p, unsigned short n)
{
    unsigned char b = 0;
    while (n--) b += *p++;
    return ((unsigned char)(-((char)b)));
}
/*
 * very old version was commented in 8bit  room420
void send_data(unsigned char cmd,unsigned char *data,unsigned char len){
  cmd_last_time=GetTickCount();
  U0.buf_out[0]=addr+5;
  U0.buf_out[1]=cmd;
  U0.buf_out[2]=len+4;
  memcpy(U0.buf_out+3,data,len);
  U0.buf_out[len+3]=ModBusLRC(U0.buf_out,len+3);
  U0.Start_TX(len+4);
}
// old version 8bit room420
void send_data(unsigned char cmd,unsigned char *data,unsigned char len){
  if(addr==0)return;
  U0.buf[1]=U0.buf[1]+3;
  U0.buf[7+addr*3]=p_HR->aT1;
  U0.buf[8+addr*3]=p_HR->aV1;
  U0.buf[9+addr*3]=p_HR->aI1;
  U0.buf[10+addr*3]=ModBusLRC(U0.buf,U0.cnt+2);
 
  U0.Start_TX(U0.cnt+3);
}
*/
void send_data(unsigned char cmd,unsigned char *data,unsigned char len){  // This sending only for command 2 send data
  if(addr==0)return;
  U0.buf[1]=U0.buf[1]+3;  // room420 =11 + 3 ... may be equal count of bytes used only for calib in all other places is commented
  U0.buf[7+addr*3]=p_HR->aT1;
  U0.buf[8+addr*3]=(unsigned char)(p_HR->aaV1  & 0xFF) ;//   first low byte byte
  U0.buf[9+addr*3]=(unsigned char)((p_HR->aaV1 >>8)  & 0xFF); ;//p_HR->aI1;  Temperature is more important then current
  U0.buf[10+addr*3]=ModBusLRC(U0.buf,U0.cnt+2);

  U0.Start_TX(U0.cnt+3);
}
// That is for another command where 8bits are sending instead of 16 bit Uvolt
void send_data_8bit(unsigned char cmd,unsigned char *data,unsigned char len){  // This sending only for command 2 send data
  if(addr==0)return;
  U0.buf[1]=U0.buf[1]+3;  // room420 =11 + 3 ... may be equal count of bytes used only for calib in all other places is commented
  U0.buf[7+addr*3]=p_HR->aT1;  //
  U0.buf[8+addr*3]=p_HR->aV1;
  U0.buf[9+addr*3]=p_HR->aI1;
  U0.buf[10+addr*3]=ModBusLRC(U0.buf,U0.cnt+2);

  U0.Start_TX(U0.cnt+3);
}

unsigned char parse_data()
{
  static unsigned char ldev = 0;
  unsigned char crcr=ModBusLRC(U0.buf,U0.cnt-1);
  if(U0.buf[U0.cnt-1]!=crcr){
      U0.cnt=0;return 0;
  }
  if(U0.buf[0]==0){
      U0.cnt=0;return 0;
  }
// Room420 for big commands  if(U0.buf[0]>(addr+5)){U0.cnt=0;return 0;}
  //if(U0.buf_in[0]<=5)cmd=U0.buf_in[0];else ldev=U0.buf_in[0];
  cmd=U0.buf[0];
  ldev=(U0.cnt-11)/3;
  if(cmd==0){U0.cnt=0;return 0;}  //||(cmd>5)

  
     //&&((ldev==addr+4)||(addr<=1))){
    switch(cmd){
      case 1:{
        U0.buf[0]=1;
        U0.buf[1]=4;
        U0.buf[2]=U0.buf[2]+1;
        U0.buf[3]=U0.buf[3]-1;
        addr=U0.buf[2];
        cmd_addr=(addr-1)/8;
        cmd_param1=(addr-1)-cmd_addr*8;
        U0.Start_TX(4);
      break;}
      case 2:{
        //if(ldev==0)
          cmd_param=U0.buf[cmd_addr+3];
        /*if((ldev==addr)||(addr==1))
          send_data(0x02,(unsigned char*)&p_HR->aT1,3);*/
          if((ldev+1)==addr)
          send_data(0x02,(unsigned char*)&p_HR->aT1,3);
          
        break;}
      case 3:{
          /* old 8bit version room420
        if(U0.buf[2]==addr+5){
          if(U0.buf[3]==1){
            memcpy(&p_HR->zav_num,&(U0.buf[4]),4);
            write_flash((unsigned char *)&p_HR->kV[0]);
          }
          send_data(0x03,(unsigned char*)&p_HR->zav_num,4);
        }
        */

          if((ldev+1)==addr){
         send_data_8bit(0x03,(unsigned char*)&p_HR->aT1,3);
          }
        break;}
      case 4:{
        if((ldev==addr+4)||(addr==1))
          send_data(0x04,(unsigned char*)&p_HR->adc[0],8);
        break;}
      case 5:{
        if(U0.buf[2]==addr+5){
          if(U0.buf[3]==3){
            memcpy(&p_HR->kV[0],&U0.buf[4],8);
            write_flash((unsigned char *)&p_HR->kV[0]);
          }
          if(U0.buf[3]==4){
            memcpy(&p_HR->kV[1],&U0.buf[4],8);
            write_flash((unsigned char *)&p_HR->kV[0]);
          }
        }
        if(U0.buf[3]&0x01)
          send_data(0x05,(unsigned char*)&(p_HR->kV[0]),8);
        else 
          send_data(0x05,(unsigned char*)&(p_HR->kV[1]),8);
        break;}


      case 6: {  // this sends all invormation available for calib purposes


          U0.buf[0]=6;
          U0.buf[1]=(unsigned char)(p_HR->aaV1  & 0xFF) ;//p_HR->aT1;   first low byte byte
          U0.buf[2]=(unsigned char)((p_HR->aaV1 >>8)  & 0xFF);
          U0.buf[3]=p_HR->aI1;
/*old version 8bit room420
          U0.buf[0]=6;
          U0.buf[1]=p_HR->aT1;
          U0.buf[2]=p_HR->aV1;
          U0.buf[3]=p_HR->aI1;
          */
          memcpy(&U0.buf[4],&p_HR->kV[0].a,32);  // 8 float
          memcpy(&U0.buf[36],&p_HR->adc[0],12);  // 3 long
          U0.buf[49]=0;
          U0.buf[48]=0;
          U0.buf[51]=0;
          U0.buf[50]=0;
          U0.buf[52]=(unsigned char)(p_HR->mspflags);


          U0.buf[53]=ModBusLRC(U0.buf,53);
          U0.cnt=54;
          U0.Start_TX(U0.cnt);

          if ((p_HR->mspflags & BIT1)==0){  // !!! for manual control of balancing
          cmd_param=0;}
          else {
              cmd_param=0xFF;
          }


      break;
      }
      // ========================   New commands for calibration ===============================
             case 30:{   //  ===========  ADC VOLTAGE store point 1 for calibs of voltage x1 -adc y1- real value===================
               for_calib_x1=(float)(p_HR->adc[0]);
               memcpy((unsigned char *)&for_calib_y1,&U0.buf[1],4);
              // for_calib_y1=((float)((long)(U0.buf[1])<<24 +(long)(U0.buf[2])<<16 +(long)(U0.buf[3])<<8  +(long)(U0.buf[1])))/10000;
               asm(" NOP");
               send_data(0x02,(unsigned char*)&p_HR->aT1,3);
               break;
            }

            case 31:{   //  ===========  store point 2 for calibs of voltage x1 -adc y1- real value===================
               memcpy((unsigned char *)&for_calib_y2,&U0.buf[1],4);
              // for_calib_y1=((float)((long)(U0.buf[1])<<24 +(long)(U0.buf[2])<<16 +(long)(U0.buf[3])<<8  +(long)(U0.buf[1])))/10000;
               asm(" NOP");
               _DINT();
               p_HR->kV[0].a=calc_a(for_calib_x1, for_calib_y1, p_HR->adc[0], for_calib_y2);
               p_HR->kV[0].b=calc_b(for_calib_x1, for_calib_y1, p_HR->kV[0].a);
               _EINT();
               send_data(0x02,(unsigned char*)&p_HR->aT1,3);
               asm(" NOP");

               break;
            }


            case 32:{   //  ===========  store coefficients===================
               write_flash((unsigned char *)&p_HR->kV[0].a);
               asm(" NOP");
               send_data(0x02,(unsigned char*)&p_HR->aT1,3);
               break;
            }


            case 33:{   //  =========== ADC CURRENT store point 1 for calibs of voltage x1 -adc y1- real value===================
               for_calib_x1=(float)(p_HR->adc[1]);
               memcpy((unsigned char *)&for_calib_y1,&U0.buf[1],4);
              // for_calib_y1=((float)((long)(U0.buf[1])<<24 +(long)(U0.buf[2])<<16 +(long)(U0.buf[3])<<8  +(long)(U0.buf[1])))/10000;
               asm(" NOP");
               send_data(0x02,(unsigned char*)&p_HR->aT1,3);
               break;
            }

            case 34:{   //  ===========  store point 2 for calibs of current ADC x2 -adc y2- real value===================
               memcpy((unsigned char *)&for_calib_y2,&U0.buf[1],4);
              // for_calib_y1=((float)((long)(U0.buf[1])<<24 +(long)(U0.buf[2])<<16 +(long)(U0.buf[3])<<8  +(long)(U0.buf[1])))/10000;
               asm(" NOP");
               _DINT();
               p_HR->kV[1].a=calc_a(for_calib_x1, for_calib_y1,(float)(p_HR->adc[1]), for_calib_y2);
               p_HR->kV[1].b=calc_b(for_calib_x1, for_calib_y1, p_HR->kV[1].a);
               _EINT();
               asm(" NOP");
               send_data(0x02,(unsigned char*)&p_HR->aT1,3);
               break;
            }


      //================   DAC  VOLTAGE first setup DAC by 16bit short value===========

            case 35:{   //  ===========  setup DAC Voltage by DAC value===================
 // this command is for dac board here is nothing to do
                send_data(0x02,(unsigned char*)&p_HR->aT1,3);
               break;
            }


            case 36:{   //  ===========   SETUP DAC VOLTAGE by Volts float with calibration value===================
                // this command is for dac board here is nothing to do
               send_data(0x02,(unsigned char*)&p_HR->aT1,3);
               break;
            }


            case 37:{   //  ===========  store point 1 for calibs of DAC voltage x1 -dac_voltage y1- real value===================
                // this command is for dac board here is nothing to do
               send_data(0x02,(unsigned char*)&p_HR->aT1,3);
               break;
            }

            case 38:{   //  ===========  store point 2 for calibs of DAC VOLTAGE===================
                // this command is for dac board here is nothing to do
               send_data(0x02,(unsigned char*)&p_HR->aT1,3);
               break;
            }


            //================   DAC Current  first setup DAC by 16bit short value===========

                  case 39:{   //  ===========  setup DAC Current by DAC value===================
                      // this command is for dac board here is nothing to do
                     send_data(0x02,(unsigned char*)&p_HR->aT1,3);
                     break;
                  }


                  case 40:{   //  ===========   setup DAC Current by Amper float with calibration valuee===================
                      // this command is for dac board here is nothing to do
                     send_data(0x02,(unsigned char*)&p_HR->aT1,3);
                     break;
                  }


                  case 41:{   //  ===========  store point 1 for calibs of DAC CURRENT===================
                      // this command is for dac board here is nothing to do
                     send_data(0x02,(unsigned char*)&p_HR->aT1,3);
                     break;
                  }

                  case 42:{   //  ===========  store point 2  for calibs of current DAC CURRENT==================
                      // this command is for dac board here is nothing to do
                     send_data(0x02,(unsigned char*)&p_HR->aT1,3);
                     break;
                  }

                  case 43:{   //  ===========  switch ON power module==================
      // this is in the main                P1OUT |= BIT1;
                     asm(" NOP");
                     p_HR->mspflags |= BIT1;
                     cmd_param=0xFF;
                     send_data(0x02,(unsigned char*)&p_HR->aT1,3);
                     break;
                  }

                  case 44:{   //  ===========  switch OFF power module==================
      // this is in the main               P1OUT &= ~BIT1;
                     p_HR->mspflags &= ~BIT1;
                     asm(" NOP");
                     cmd_param=0;
                     send_data(0x02,(unsigned char*)&p_HR->aT1,3);
                     break;
                  }

                 default:{

                       U0.cnt=0;return 0;
                    break;
                   }

          }




      return 1;
    }



void write_flash(unsigned char *value)
{
  char *Flash_ptr;                          // Flash pointer
  unsigned int i;

  Flash_ptr = (char *)0x1040;               // Initialize Flash pointer
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  *Flash_ptr = 0;                           // Dummy write to erase Flash seg

  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
  unsigned char crc=ModBusLRC(value,20);
  for (i = 0; i < 20; i++)
  {
    *Flash_ptr++ = value[i];                   // Write value to flash
  }
  *Flash_ptr++ =crc;
    
  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}


void read_flash(unsigned char *value)
{
  char *Flash_ptr;                          // Flash pointer
  unsigned int i;

  Flash_ptr = (char *)0x1040;               // Initialize Flash pointer
  for (i = 0; i < 20; i++)
  {
     value[i]=*Flash_ptr++;                   // Write value to flash
  }
  unsigned char crc=*Flash_ptr++;
  if(ModBusLRC(value,20)!=crc){
    p_HR->kV[0].a=0.008157007f;
    p_HR->kV[0].b=0.380803435f;
    p_HR->kV[1].a=0.001374;
    p_HR->kV[1].b=-0.5522;
  }
  //  in passive we do not use this coefficient for DAC room420
  p_HR->kV[2].b=0;
  p_HR->kV[2].a=1;
  p_HR->kV[3].b=0;
  p_HR->kV[3].a=1;

}

int main( void )
{
  Init_SYS();
  Init_UART0();
  p_HR = &g_HR;
  p_ser = (unsigned char *)p_HR;
  WDT();
  Init_SD24();
  WDT();
  unsigned long last_blink=0;
  __bis_SR_register(GIE);                 // Enter LPM0 w/ interrupt
  read_flash((unsigned char *)&p_HR->kV[0]);

  
    
  while(1){
   /*if((U0.rx_last_time+200L<GetTickCount())&&(U0.cnt_in>1)){
      U0.ptr_in=0;
      U0.cnt_in=0;
      //U0.ptr_out=0;
      //U0.cnt_out=0;
      U0.mode=UART_RECEIVE_MODE;
   }*/
   
   if((cmd_last_time+20000>=GetTickCount())){
     WDT();
   }
   
   //WDT();
   if(((U0.rx_last_time+50L)<GetTickCount())&&(U0.mode==UART_RECEIVE_MODE)/*&&(U0TCTL&TXEPT)*/){
       if(U0.cnt>=3){
         if(!parse_data()){
          U0.ptr=0;
          U0.cnt=0;
         }
       }
   }
      
   GetADCValue();
   if(last_blink+300L<GetTickCount()){
       if(cmd_param&(1<<cmd_param1))p_HR->mspflags |= BIT1; else p_HR->mspflags &= ~BIT1;   // room420 this is balancing

       //  =======  Test flags and make action new  room420============
          if((p_HR->mspflags & BIT1)==BIT1 ){
             P1OUT |= BIT1;
             p_HR->mspflags |= BIT2 ; // command on balancing is fulfilled
          }else
          { P1OUT &= ~BIT1;   // room420 this is balancing
          p_HR->mspflags &= ~BIT2 ; // command not balancing command or temperature too high
             }

 // this was before     if(cmd_param&(1<<cmd_param1))P1OUT |= BIT1; else P1OUT &= ~BIT1;
        last_blink=GetTickCount();
   }
  }
  }


//====================  functions for calibration ==============
float calc_a(float x1, float y1, float x2, float y2)  // x1-adc y1-real value
 {
     float x2_dec_x1, ans;

     x2_dec_x1 = x2;
     x2_dec_x1 -= x1;

     if (!x2_dec_x1) x2_dec_x1 = 1;

     ans = y2;
     ans -= y1;
     ans /= x2_dec_x1;
     return ans;
 }

float calc_b(float x1, float y1, float coeff_a) // x1 adc  y1 real value
 {
    float x2_dec_x1, temp, ans;
    ans=x1 * coeff_a ; // real value
    ans = y1 - ans;
     return ans;
 }



