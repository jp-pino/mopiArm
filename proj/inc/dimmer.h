#ifndef __DIMMER_H__
#define __DIMMER_H__

#define DIMMER_PID_PERIOD 100

#define DIMMER_K_P 0.003333f
#define DIMMER_K_I 1.0f
#define DIMMER_K_D -1.0f

typedef struct dimmer_ctrl_data_t {
	float temp;				// Reference
	float temp_p;			// Previous temperature (for differential)
  float integral;			// Integral error
} dimmer_ctrl_data_t;

void Dimmer_Init(float, float);

unsigned char Dimmer_GetState(void);
void Dimmer_Enable(void);
void Dimmer_Disable(void);
unsigned char Dimmer_ToggleState(void);

float Dimmer_GetPower(void);
void Dimmer_SetPower(float);

void Dimmer_SetTemperature(float);

#endif
