#ifndef LM75_H
#define	LM75_H
#define  ACK  0
#define  NACK 1
//------------------------------------------------------------------------------
#include "I2C_librery.h"
#include "LM75_Prototypes.h"
//------------------------------------------------------------------------------
void LM75_init()
{
  I2C_init(100000);   //Start at 100Kb/s
  
  //For Thermostat functions ONLY... can be left out if so wished and device will still read Tempr
  LM75_set(TOS_ADDRESS,0x2380);     //Over Temperature Threshold Activate - I set it to 35.50C
  LM75_set(THYST_ADDRESS,0x1E80);   //Over Temperature Threshold deactivate - I set it to 30.50C
}
//------------------------------------------------------------------------------
float LM75_read(char regAddress)
{
  signed int MSB,LSB;
  
  I2C_start();                      
  
  I2C_write(LM75_ADDRESS + 0);        // 0 = Write             
  I2C_write(regAddress);        
  I2C_restart();                           
  I2C_write(LM75_ADDRESS + 1);        // 1 = Read   
  MSB = I2C_read(ACK);
  LSB = I2C_read(NACK);
  
  I2C_Stop();
  
  return toFloat((MSB << 8) + LSB);
}
float LM75_read2(char regAddress)
{
  signed int MSB,LSB;
  
  I2C_start();                      
  
  I2C_write(LM75_ADDRESS + 0);        // 0 = Write             
  I2C_write(regAddress);        
  I2C_restart();                           
  I2C_write(LM75_ADDRESS + 1);        // 1 = Read   
  MSB = I2C_read(ACK);
  LSB = I2C_read(NACK);
  
  I2C_stop();
  
  return MSB;
}
//------------------------------------------------------------------------------
void LM75_setConfig(char data)
{  
  I2C_Start();                      
  
  I2C_write(LM75_ADDRESS);                       
  I2C_write(CONFIGURATION_ADDRESS);              
  I2C_write(data);       
  
  I2C_stop();
}
//------------------------------------------------------------------------------
char LM75_readConfig()
{  
  I2C_start();                      
  
  I2C_write(LM75_ADDRESS + 0);        // 0 = Write               
  I2C_write(CONFIGURATION_ADDRESS);        
  I2C_restart();                           
  I2C_write(LM75_ADDRESS + 1);        // 1 = Read   
  char config = I2C_read(NACK);
  
  I2C_stop();
  
  return config;
}
//------------------------------------------------------------------------------
void LM75_set(char regAddress, int data)
{  
  I2C_start();                      
  
  I2C_write(LM75_ADDRESS);                    
  I2C_write(regAddress);      //Registers: Config,THYST,OVER TEMP SINK (OS) /*See Datasheet*/                       
  I2C_write(data>>8);         //MSByte       
  I2C_write(data);            //LSByte
  
  I2C_stop();
}
//------------------------------------------------------------------------------
float toFloat(signed int tempr)
{
  float result = (float)(tempr>>8);      //Discard LSByte (Only holds fraction flag bit - see datasheet)
  char isfraction = tempr & FRACTION_FLAG;          
  
  if(isfraction) 
  {
    result = result + 0.5;
  }
  
  return (result);
}
//------------------------------------------------------------------------------
#endif