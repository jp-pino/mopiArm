#include "tm4c123gh6pm.h"
#include "OS.h"
#include "Dimmer.h"
#include "MCP9600.h"

#define TIMESLICE TIME_2MS  //thread switch time in system time units

// OS and modules initialization
int main(void) {        // lab 4 real main

  // Initialize the OS
  OS_Init();

  Dimmer_Init(0.0f, 1.0f);
  Dimmer_SetPower(0.3f);
  Dimmer_Enable();

  MCP9600_Init(TYPE_K, 2);

  // Launch the OS
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}