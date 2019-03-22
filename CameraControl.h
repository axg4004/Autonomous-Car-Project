#ifndef UART_H
#define UART_H
#include <stdint.h>
void Init_Camera(void);
void Smooth_Line(void);
void Diff_Line(void);
void init_FTM2(void);
void init_GPIO(void);
void init_PIT(void);
void init_ADC0(void);
void FTM2_IRQHandler(void);
void PIT1_IRQHandler(void);
void ADC0_IRQHandler(void);
void setMaxMotorSpeed(int speed);
#endif