#ifndef __DIMMER_H__
#define __DIMMER_H__

void Dimmer_Init(float, float);

void Dimmer_Sync(void);

unsigned char Dimmer_GetState(void);
void Dimmer_Enable(void);
void Dimmer_Disable(void);
unsigned char Dimmer_ToggleState(void);

float Dimmer_GetPower(void);
void Dimmer_SetPower(float);

#endif
