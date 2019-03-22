#ifndef PWM_H_
#define PWM_H_

void SetRightMotorDutyCycle(float RightDutyCycle, unsigned int Frequency, int dir);
void SetLeftMotorDutyCycle(float LeftDutyCycle, unsigned int Frequency, int dir);
void SetServoDutyCycle(float DutyCycle, unsigned int Frequency);
void Init_Motors();

#endif /* PWM_H_ */
