#ifndef __APP_SHARED_H
#define __APP_SHARED_H

#include "stm32f10x.h"
#include "PID.h"

extern volatile uint8_t RunFlag;
extern volatile uint8_t RunFlagUpdate;
extern volatile uint16_t SpeedLevel;

extern int16_t AX, AY, AZ, GX, GY, GZ;
extern int16_t GY_Offset;
extern int16_t GY_Cali;

extern float AngleAcc;
extern float AngleAcc_Offset;
extern float AngleAcc_Cali;
extern float AngleAcc_Filter;
extern float AngleDelta;
extern float Angle;

extern float SpeedLeft, SpeedRight;
extern float AveSpeed, DifSpeed;

extern float AvePWM, DifPWM;
extern int16_t PWML, PWMR;

extern PID_t AnglePID;
extern PID_t SpeedPID;
extern PID_t TurnPID;

#define ANGLE_T     10
#define SPEED_T     50

void AppShared_EnterCritical(void);
void AppShared_ExitCritical(void);

#endif
