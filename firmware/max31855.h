#ifndef __max31855__H__
#define __max31855__H__

#define int32_t long
#define uint32_t unsigned long
/*
 *  Define literals for the SPI port accesses and the thermocouple chip
 *  select line.
 */
#define  PORT_THERMO_CS           P1OUT
#define  DDR_THERMO_CS            P1DIR
#define  BIT_THERMO_CS            2
#define  MASK_THERMO_CS           (1<<BIT_THERMO_CS)

#define  PORT_SPI                 P1OUT
#define  DDR_SPI                  P1DIR
#define  BIT_SPI_SCK              7
#define  MASK_SPI_SCK             (1<<BIT_SPI_SCK)
#define  BIT_SPI_MISO             6
#define  MASK_SPI_MISO            (1<<BIT_SPI_MISO)





/*
 *  ThermoInit      set up hardware for using the MAX31855
 *
 *  This routine configures the SPI as a master for exchanging
 *  data with the MAX31855 thermocouple converter.  All pins
 *  and registers for accessing the various port lines are
 *  defined at the top of this code as named literals.
 */
void  ThermoInit(void);


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
int32_t  ThermoReadRaw(void);



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
int  ThermoReadC(void);



#endif

