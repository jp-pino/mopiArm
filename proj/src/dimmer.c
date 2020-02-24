#include "dimmer.h"
#include "tm4c123gh6pm.h"
#include "OS.h"

#define DIMMER_PERIOD (((1.0f/120.0f)/800.0f)*1000000000.0f)

unsigned char state;
float min, max, power;

void Dimmer_Init(float min_val, float max_val) {
  // Initialize variables
  state = 0;
  min = min_val;
  max = max_val;
  if (min < 0.0f || min >= 1.0f)
    min = 0.0f;
  if (max <= 0.0f || max > 1.0f)
    max = 1.0f;
  power = min;

  // Initialize PWM Generator PWM0
  SYSCTL_RCGCPWM_R |= 0x01;             // 1) activate PWM0
  SYSCTL_RCGCGPIO_R |= 0x02;            // 2) activate port B
  while((SYSCTL_PRGPIO_R&0x02) == 0);
  GPIO_PORTB_AFSEL_R |= 0x40;           // enable alt funct on PB6
  GPIO_PORTB_PCTL_R &= ~0x0F000000;     // configure PB6 as PWM0
  GPIO_PORTB_PCTL_R |= 0x04000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;          // disable analog functionality on PB6
  GPIO_PORTB_DIR_R |= 0x40;             // PB6 output
  GPIO_PORTB_DEN_R |= 0x40;             // enable digital I/O on PB6
  SYSCTL_RCC_R = 0x001A0000 |           // 3) use PWM divider
      (SYSCTL_RCC_R & (~0x000E0000));   //    configure for /64 divider
  PWM0_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM0_0_GENA_R = 0xC8;                 // low on LOAD, high on CMPA down
  // PB6 goes low on LOAD
  // PB6 goes high on CMPA down
  PWM0_0_LOAD_R = DIMMER_PERIOD - 1;           // 5) cycles needed to count down to 0
  PWM0_0_CMPA_R = (unsigned long)(power * DIMMER_PERIOD) - 1;             // 6) count value when output rises
  PWM0_0_CTL_R |= 0x00000001;           // 7) start PWM0
  PWM0_ENABLE_R &= ~0x00000002;         // disable


  // Configure PB7 as input with interrupt
  // Disable analog functionality on PB7
  GPIO_PORTB_AMSEL_R &= 0x80;
  // Configure as PB7 GPIO
  GPIO_PORTB_PCTL_R &= ~0xF0000000;
  // Configure PB7 as input
  GPIO_PORTB_DIR_R &= ~0x80;
  // Enable digital I/O on PB7
  GPIO_PORTB_DEN_R |= 0x80;
  // Disable alternate functions on PB7
  GPIO_PORTB_AFSEL_R &= ~0x80;
  // Enable interrupts on PB7
  GPIO_PORTB_IM_R |= 0x80;
  // Interrupt priority 3
  NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFF00FF) | (0 << 13);
  NVIC_EN0_R |= 1 << 1;
}

void GPIOPortB_Handler(void) {
  // Check for PB7
  if ((GPIO_PORTB_RIS_R & 0x80) == 0x80) {
    GPIO_PORTB_ICR_R |= 0x80;
    GPIO_PORTB_IM_R &= ~0x80;
    Dimmer_Sync();
    GPIO_PORTB_IM_R |= 0x80;
  }
}

void Dimmer_Sync(void) {
  PWM0_SYNC_R |= 0x01;
}

unsigned char Dimmer_GetState(void) {
  return state;
}

void Dimmer_Enable(void) {
  state = 1;
  PWM0_ENABLE_R |= 0x00000001;
}

void Dimmer_Disable() {
  state = 0;
  PWM0_ENABLE_R &= ~0x00000001;
}

unsigned char Dimmer_ToggleState(void) {
  if (state == 0) {
    Dimmer_Enable();
  } else {
    Dimmer_Disable();
  }
}

float Dimmer_GetPower(void) {
  return power;
}

void Dimmer_SetPower(float power_val) {
  unsigned char last_state = Dimmer_GetState();
  if (power_val >= min && power_val <= max) {
    Dimmer_Disable();
    power = power_val;
    PWM0_0_CMPA_R = (unsigned long)(power * DIMMER_PERIOD) - 1;
    if (last_state) {
      Dimmer_Enable();
    }
  }
}
