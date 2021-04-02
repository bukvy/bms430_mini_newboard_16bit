#ifndef __SYSTEM__H__
#define __SYSTEM__H__

extern unsigned long GlobalTime;

unsigned long GetTickCount(void);
void Sleep(unsigned long ms);
void Init_SYS(void);
void WDT(void);
#endif

