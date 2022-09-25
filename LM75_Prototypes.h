#ifndef LM75_PROTOTYPES_H
#define	LM75_PROTOTYPES_H
//------------------------------------------------------------------------------
#define  LM75_ADDRESS             0x90     // Address + Read bit
#define  TEMPERATURE_ADDRESS      0x00
#define  CONFIGURATION_ADDRESS    0x01
#define  THYST_ADDRESS            0x02     
#define  TOS_ADDRESS              0x03
#define  FRACTION_FLAG            0x0080   // 0.5 Bit Flag
//------------------------------------------------------------------------------
void  LM75_init(void);
float LM75_read(char regAddress);
float LM75_read2(char regAddress);
void  LM75_setConfig(char data);
char  LM75_readConfig(void);
void  LM75_set(char regAddress, int data);
float toFloat(signed int tempr);
//------------------------------------------------------------------------------
#endif