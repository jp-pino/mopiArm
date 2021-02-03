#ifndef __SERVO_H__
#define __SERVO_H__

void Servo_Init(char id);
void Servo_SetPWM(char id, float pwm);
void Servo_SetAngle(char id, float angle);

#endif
