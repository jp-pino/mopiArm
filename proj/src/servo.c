#include "tm4c123gh6pm.h"
#include "OS.h"
#include "servo.h"

void Servo_Init(char id) {
  switch (id) {
    case 0:
      // Enable PWM0 clock
      SYSCTL_RCGCPWM_R |= (1 << 0);
      // Enable Port B clock
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB;
      // Configure PWM prescaler
      SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
      SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;
      SYSCTL_RCC_R |= SYSCTL_RCC_PWMDIV_64;
      // 4ma drive output on PB6
      GPIO_PORTB_DR8R_R |= (1 << 6);
      // Enable digital output on PB6
      GPIO_PORTB_DEN_R |= (1 << 6);
      // Enable alternate functions on PB6
      GPIO_PORTB_AFSEL_R |= (1 << 6);
      // Configure PB6 as PWM
      GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PB6_M;
      GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB6_M0PWM0;
      // Disable PWM
      PWM0_0_CTL_R &= ~(1 << 0);
      // Set count down mode
      PWM0_0_CTL_R &= ~(1 << 1);
      // Set action
      PWM0_0_GENA_R |= PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
      // Set period to 20ms
      PWM0_0_LOAD_R = 0x000061A8;
      // Set initial servo value
      PWM0_0_CMPA_R = 1250;
      // Start timers
      PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;
      // Enable PWM output
      PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;
      break;
    case 1:
      // Enable PWM0 clock
      SYSCTL_RCGCPWM_R |= (1 << 0);
      // Enable Port B clock
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB;
      // Configure PWM prescaler
      SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
      SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;
      SYSCTL_RCC_R |= SYSCTL_RCC_PWMDIV_64;
      // 4ma drive output on PB7
      GPIO_PORTB_DR8R_R |= (1 << 7);
      // Enable digital output on PB7
      GPIO_PORTB_DEN_R |= (1 << 7);
      // Enable alternate functions on PB7
      GPIO_PORTB_AFSEL_R |= (1 << 7);
      // Configure PB7 as PWM
      GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PB7_M;
      GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB7_M0PWM1;
      // Disable PWM
      PWM0_0_CTL_R &= ~(1 << 0);
      // Set count down mode
      PWM0_0_CTL_R &= ~(1 << 1);
      // Set action
      PWM0_0_GENB_R |= PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;
      // Set period to 20ms
      PWM0_0_LOAD_R = 0x000061A8;
      // Set initial servo value
      PWM0_0_CMPB_R = 1250;
      // Start timers
      PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;
      // Enable PWM output
      PWM0_ENABLE_R |= PWM_ENABLE_PWM1EN;
      break;
    case 2:
      // Enable PWM0 clock
      SYSCTL_RCGCPWM_R |= (1 << 0);
      // Enable Port B clock
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOB;
      // Configure PWM prescaler
      SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
      SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;
      SYSCTL_RCC_R |= SYSCTL_RCC_PWMDIV_64;
      // 4ma drive output on PB4
      GPIO_PORTB_DR8R_R |= (1 << 4);
      // Enable digital output on PB4
      GPIO_PORTB_DEN_R |= (1 << 4);
      // Enable alternate functions on PB4
      GPIO_PORTB_AFSEL_R |= (1 << 4);
      // Configure PB4 as PWM
      GPIO_PORTB_PCTL_R &= ~GPIO_PCTL_PB4_M;
      GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_M0PWM2;
      // Disable PWM
      PWM0_1_CTL_R &= ~(1 << 0);
      // Set count down mode
      PWM0_1_CTL_R &= ~(1 << 1);
      // Set action
      PWM0_1_GENA_R |= PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
      // Set period to 20ms
      PWM0_1_LOAD_R = 0x000061A8;
      // Set initial servo value
      PWM0_1_CMPA_R = 1250;
      // Start timers
      PWM0_1_CTL_R |= PWM_1_CTL_ENABLE;
      // Enable PWM output
      PWM0_ENABLE_R |= PWM_ENABLE_PWM2EN;
      break;
    case 4:
      break;
    default:
      break;
  }
}

// Min: 680 -> .544 ms
// Max: 3000 -> 2.4 ms
void Servo_SetPWM(char id, float pwm) {
  if (pwm < 0 || pwm > 1) {
    return;
  }
  switch (id) {
    case 0:
      PWM0_0_CMPA_R = (2320 * pwm) + 680;
      break;
    case 1:
      PWM0_0_CMPB_R = (2320 * pwm) + 680;
      break;
    case 2:
      PWM0_1_CMPA_R = (2320 * pwm) + 680;
      break;
    case 4:
      break;
    default:
      break;
  }
}

// 0 - 180
void Servo_SetAngle(char id, float angle) {
  if (angle < 0) {
    angle = 0;
  }
  if (angle > 180) {
    angle = 180;
  }
  Servo_SetPWM(id, angle / 180.0f);
}