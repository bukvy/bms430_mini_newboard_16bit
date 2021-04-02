#include "msp430afe253.h"
#include "max31855.h"
#include "modbus_dta.h"

const unsigned short sens[138]=
{1,397,798,1203,1612,2023,2436,2851,3267,3682,4096,4509,4920,5328,
5735,6138,6540,6941,7340,7739,8138,8539,8940,9343,9747,10153,10561,
10971,11382,11795,12209,12624,13040,13457,13874,14293,14713,15133,
15554,15975,16397,16820,17243,17667,18091,18516,18941,19366,19792,
20218,20644,21071,21497,21924,22350,22776,23203,23629,24055,24480,
24905,25330,25755,26179,26602,27025,27447,27869,28289,28710,29129,
29548,29965,30382,30798,31213,31628,32041,32453,32865,33275,33685,
34093,34501,34908,35313,35718,36121,36524,36925,37326,37725,38124,
38522,38918,39314,39708,40101,40494,40885,41276,41665,42053,42440,
42826,43211,43595,43978,44359,44740,45119,45497,45873,46249,46623,
46995,47367,47737,48105,48473,48838,49202,49565,49926,50286,50644,
51000,51355,51708,52060,52410,52759,53106,53451,53795,54138,54479};
/*
 *  ThermoInit      set up hardware for using the MAX31855
 *
 *  This routine configures the SPI as a master for exchanging
 *  data with the MAX31855 thermocouple converter.  All pins
 *  and registers for accessing the various port lines are
 *  defined at the top of this code as named literals.
 */
void  ThermoInit(void)
{
    PORT_THERMO_CS |= MASK_THERMO_CS;        // start with CS high
    DDR_THERMO_CS |= MASK_THERMO_CS;         // now make that line an output

    PORT_SPI &= ~MASK_SPI_SCK;               // drive SCK low
    DDR_SPI |=  MASK_SPI_SCK;                 // now make SCK an output
    
    //PORT_SPI &= ~MASK_SPI_MISO;               // drive SCK low
    DDR_SPI  &= ~MASK_SPI_MISO;                 // now make SCK an output

}


/*
 *  ThermoReadRaw      return 32-bit raw value from MAX31855
 *
 *  This routine uses a four-byte SPI exchange to collect a
 *  raw reading from the MAX31855 thermocouple converter.  That
 *  value is returned unprocessed to the calling routine.
 *
 *  Note that this routine does NO processing.  It does not
 *  check for error flags or reasonable data ranges.
 */

int32_t  ThermoReadRaw(void)
{
    unsigned long                d;
    unsigned char                n;

    PORT_THERMO_CS &= ~MASK_THERMO_CS;    // pull thermo CS low
    d = 0;                                // start with nothing
    for (n=0; n<32; n++)
    {
      PORT_SPI |=  MASK_SPI_SCK;
      if((P1IN&MASK_SPI_MISO)==MASK_SPI_MISO)d|=(unsigned long)1<<(31-n);
      PORT_SPI &= ~ MASK_SPI_SCK;
    }
    PORT_THERMO_CS |= MASK_THERMO_CS;     // done, pull CS high

    return  d;
}



unsigned short TermoCalc(unsigned short U)
{
  int i;
  unsigned short U1,U2;
  float K1,K2,KK;
  if(U>397){
  for(i=0;i<137;i++){
    if(sens[i]>U)break;
  }
  U1=sens[i-1];
  U2=sens[i];
  K2=(float)U2/(i*10);
  K1=(float)U1/((i-1)*10);
  KK=K1+(K2-K1)*(U-U1)/(U2-U1);
  }else KK=40;
  return (unsigned short)(U/KK);
}
unsigned char ttt=0;



/*
 *  ThermoReadC      return thermocouple temperature in degrees C
 *
 *  This routine takes a raw reading from the thermocouple converter
 *  and translates that value into a temperature in degrees C.  That
 *  value is returned to the calling routine as an integer value,
 *  rounded.
 *
 *  The thermocouple value is stored in bits 31-18 as a signed 14-bit
 *  value, where the LSB represents 0.25 degC.  To convert to an
 *  integer value with no intermediate float operations, this code
 *  shifts the value 20 places right, rather than 18, effectively
 *  dividing the raw value by 4 and scaling it to unit degrees.
 *
 *  Note that this routine does NOT check the error flags in the
 *  raw value.  This would be a nice thing to add later, when I've
 *  figured out how I want to propagate the error conditions...
 */
int  ThermoReadC(void)
{
    char                        neg;
    int32_t                     d,intT,extT;
    float temp;
    
    neg = 0;                
    d = ThermoReadRaw();    
    
    intT = ((d >> 4) & 0xfff);   
    if (intT & 0x800)             
    {
        intT = -intT & 0xfff;       
        neg = 1;        
    }
    intT = intT + 2;                  
    intT = intT >> 4;               
    if (neg)  intT = -intT;         
    p_IR->T1=  intT;  
    neg = 0;              
    
    extT = ((d >> 18) & 0x3fff);  
    if (extT & 0x2000)            
    {
        extT = -extT & 0x3fff;      
        neg = 1;       
    }
    extT = extT + 2;                
    extT = extT >> 2;              
    if (neg)  extT = -extT;        
    p_IR->T2=  extT;
		if(p_IR->T2>p_IR->T1){
			temp=extT-intT;
			temp=temp*41.276;
			p_IR->T3=  (unsigned short)temp;  
			extT=TermoCalc(p_IR->T3);
			p_IR->T4=extT;
			p_IR->T5=extT+p_IR->T1;
    }else{
			p_IR->T5= p_IR->T2;
		}
    if((d&0x03)>0)
        ttt++;
    else {
        ttt=0;
        p_IR->T6=p_IR->T5;
    }
    
    if(ttt>=5){
      ttt=5;
      extT=20;
      p_IR->T6=p_IR->T5;
    }
    
    return  extT;                  // return as integer
}


