#include <math.h>
#include "MCP9600.h"
#include "tm4c123gh6pm.h"
#include "OS.h"
#include "I2C.h"
#include "ST7735.h"

void temp(void) {
  while (1) {
    ST7735_Message(ST7735_DISPLAY_TOP, 0, "Sensor:", MCP9600_GetTemperature(REGISTER_HOT_JUNCTION));
    ST7735_Message(ST7735_DISPLAY_TOP, 1, "Delta:", MCP9600_GetTemperature(REGISTER_DELTA_TEMP));
    ST7735_Message(ST7735_DISPLAY_TOP, 2, "Ambient:", MCP9600_GetTemperature(REGISTER_COLD_JUNCTION));
    OS_Sleep(200);
  }
}

void MCP9600_Init(THERM_TYPE type, unsigned char filter) {
  unsigned char config;

  // Initialize I2C Bus
  I2C_Init();

  // Configure device
  config = (type << 4) | (filter);
  I2C_write(DEVICE_ADDRESS, REGISTER_THERM_CFG, &config, 1);

  // Start thread
  OS_AddThread("temp", &temp, 128, 3);
}

float MCP9600_GetTemperature(unsigned char address) {
  unsigned short data;
  unsigned char buff[2];
  float temp = 0;

  // Send command and load response
  I2C_read(DEVICE_ADDRESS, address, buff, 2);

  // Read temperature from response
  data = (unsigned short)((buff[0] << 8) | buff[1]);
  temp = data * 0.0625f;
  if ((buff[0] & 0x80) == 0x80) {
    temp -= 4096.0f;
  }
  return temp;
}
