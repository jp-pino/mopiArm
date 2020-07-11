#ifndef __MCP9600_H__
#define __MCP9600_H__

#define DEVICE_ADDRESS           0x67

#define REGISTER_HOT_JUNCTION    0x00
#define REGISTER_DELTA_TEMP      0x01
#define REGISTER_COLD_JUNCTION   0x02
#define REGISTER_THERM_CFG       0x05
#define REGISTER_VERSION         0x20


typedef enum {
  TYPE_K = 0,
  TYPE_J,
  TYPE_T,
  TYPE_N,
  TYPE_S,
  TYPE_E,
  TYPE_B,
  TYPE_R
} THERM_TYPE;


void MCP9600_Init(THERM_TYPE type, unsigned char filter);
float MCP9600_GetTemperature(unsigned char address);


#endif
