#include "Dimmer.h"
#include "tm4c123gh6pm.h"
#include "OS.h"
#include "MCP9600.h"
#include "ST7735.h"


#define PB6 (*((volatile unsigned long *)0x40005100))
#define DIMMER_PERIOD (((1.0f/120.0f)/12.5f)*1000000000.0f)

unsigned char state, thread;
float min, max, power;

dimmer_ctrl_data_t dimmer_ctrl_data;

void task(void) {
  int i = 0;
  if (state != 0) {
    while(i++ < 53)
      PB6 |= 0x40;
    PB6 &= ~0x40;
  }
}

void Dimmer_PID(void) {
	long start;
	float temp_r, temp_c, temp_p, error;
	float power;

	static long last = 0;

  // Initialize variables
	dimmer_ctrl_data.temp = 0.0f;
	dimmer_ctrl_data.temp_p = 0;
	dimmer_ctrl_data.integral = 0;
	while(1) {
		// Store start time
		start = OS_Time();

		// Read Current temperature
		temp_c = MCP9600_GetTemperature(REGISTER_HOT_JUNCTION);
		// Read previous temperature
		temp_p = dimmer_ctrl_data.temp_p;
		// Read Reference temperature
		temp_r = dimmer_ctrl_data.temp;

		// Calculate Error
		error = temp_r - temp_c;


		// Calculate PID
		// P. Proportional
		power = error * DIMMER_K_P;

		// I. Integral
		dimmer_ctrl_data.integral += error * (DIMMER_PID_PERIOD);
		power += dimmer_ctrl_data.integral * DIMMER_K_I;

		// D. Derivative
		power += DIMMER_K_D * (temp_c - temp_p) / (float)(DIMMER_PID_PERIOD);

    if (power > 0.99f) {
      power = 1.00f;
    } else if (power < 0.00f) {
      power = 0.00f;
    }

		// Set new dimmer power
    Dimmer_SetPower(power);

		// Store new previous temperature
		dimmer_ctrl_data.temp_p = temp_c;
		last = OS_Time();

    ST7735_Message(ST7735_DISPLAY_BOTTOM, 0, "Error:", error);
    ST7735_Message(ST7735_DISPLAY_BOTTOM, 1, "Power:", power);

		// Sleep for required time
		OS_Sleep(DIMMER_PID_PERIOD - (OS_Time() - start));
	}
}

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

  // Enable Port B Clock
  SYSCTL_RCGCGPIO_R |= 0x02;
  // Configure PB6 as output 
  // Disable analog functionality on PB6
  GPIO_PORTB_AMSEL_R &= 0x40;
  // Configure as PB6 GPIO
  GPIO_PORTB_PCTL_R &= ~0x0F000000;
  // Configure PB6 as output
  GPIO_PORTB_DIR_R |= 0x40;
  // Enable digital I/O on PB6
  GPIO_PORTB_DEN_R |= 0x40;
  // Disable alternate functions on PB6
  GPIO_PORTB_AFSEL_R &= ~0x40;

  // Enable Port D Clock
  SYSCTL_RCGCGPIO_R |= 0x08;
  // Configure PD0 as input with interrupt
  // Disable analog functionality on PD0
  GPIO_PORTD_AMSEL_R &= 0x01;
  // Configure as PD0 GPIO
  GPIO_PORTD_PCTL_R &= ~0x0000000F;
  // Configure PD0 as input
  GPIO_PORTD_DIR_R &= ~0x01;
  // Enable digital I/O on PD0
  GPIO_PORTD_DEN_R |= 0x01;
  // Disable alternate functions on PD0
  GPIO_PORTD_AFSEL_R &= ~0x01;
  // Enable interrupts on PD0 Both Edges
  GPIO_PORTD_IM_R |= 0x01;
  GPIO_PORTD_IBE_R |= 0x01;
  // Interrupt priority 3
  NVIC_PRI0_R = (NVIC_PRI0_R & 0x00FFFFFF) | (3 << 29);
  NVIC_EN0_R |= 1 << 3;

  thread = OS_AddOneShotThread(&task, (1.0f - power) * DIMMER_PERIOD, 2);

  // Initialize temperature sensor (feedback)
  MCP9600_Init(TYPE_K, 2);

  // Add PID thread
  // OS_AddThread("Dimmer", &Dimmer_PID, 256, 1);
}

void GPIOPortD_Handler(void) {
  // Check for PD0
  if ((GPIO_PORTD_RIS_R & 0x01) == 0x01) {
    GPIO_PORTD_ICR_R |= 0x01;
    GPIO_PORTD_IM_R &= ~0x01;
    OS_TriggerOneShotThread(thread);
    GPIO_PORTD_IM_R |= 0x01;
  }
}

unsigned char Dimmer_GetState(void) {
  return state;
}

void Dimmer_Enable(void) {
  state = 1;
}

void Dimmer_Disable() {
  state = 0;
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
  power = power_val;
  OS_SetOneShotPeriod(thread, (1.0f - power_val) * DIMMER_PERIOD);
}

void Dimmer_SetTemperature(float temperature) {
  dimmer_ctrl_data.temp = temperature;
}

float Dimmer_GetTemperature(void) {
  return dimmer_ctrl_data.temp_p;
}