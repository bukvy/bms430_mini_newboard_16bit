#ifndef __modbus_dta__
#define __modbus_dta__


#define PACKED

#ifndef GNU_PACKED
    #if defined(CTK_GNU) || defined(__GNUC__)
      #define GNU_PACKED __attribute__ ((packed))
    #else
      #define GNU_PACKED
    #endif
  #endif


#if defined(_MSC_VER) || defined(__IAR_SYSTEMS_ICC__)
#pragma pack(push,1)
#endif

typedef PACKED struct tagCKAl
{
  float    a,b;
} GNU_PACKED CKal;

// ==========================================================================
// ------------------------------------ ::HR:: ------------------------------
typedef PACKED struct tagMODBUS_HR
{
    unsigned char mspflags;

    char aT1;
    unsigned char aV1;
    unsigned short aaV1;
    char aI1;
   /*old version 8bit room420
  char aT1;
  unsigned char aV1,aI1;
     */
  unsigned long adc[3];
  
  CKal    kV[4];
  unsigned long zav_num;
  



} GNU_PACKED CMODBUS_HR;
// ------------------------------------- END ::HR:: -------------------------

extern CMODBUS_HR *p_HR;

extern CMODBUS_HR g_HR;


#if defined(_MSC_VER) || defined(__IAR_SYSTEMS_ICC__)
#pragma pack(pop)
#endif




#endif
