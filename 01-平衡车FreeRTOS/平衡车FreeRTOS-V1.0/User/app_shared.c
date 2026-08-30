#include "app_shared.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint8_t RunFlag;
volatile uint8_t RunFlagUpdate;
volatile uint16_t SpeedLevel = 5;

int16_t AX, AY, AZ, GX, GY, GZ;
int16_t GY_Offset;
int16_t GY_Cali;

float AngleAcc;
float AngleAcc_Offset;
float AngleAcc_Cali;
float AngleAcc_Filter;
float AngleDelta;
float Angle;

float SpeedLeft, SpeedRight;
float AveSpeed, DifSpeed;

float AvePWM, DifPWM;
int16_t PWML, PWMR;

/* PID 固定为默认值，不做在线调参 */
PID_t AnglePID = {
	.Kp = 3,
	.Ki = 0.1f,
	.Kd = 3,
	.OutMax = 100,
	.OutMin = -100,
	.OutOffset = 3,
};

PID_t SpeedPID = {
	.Kp = 2,
	.Ki = 0.05f,
	.Kd = 0,
	.OutMax = 10,
	.OutMin = -10,
};

PID_t TurnPID = {
	.Kp = 2,
	.Ki = 1,
	.Kd = 0,
	.OutMax = 30,
	.OutMin = -30,
};

void AppShared_EnterCritical(void)
{
	taskENTER_CRITICAL();
}

void AppShared_ExitCritical(void)
{
	taskEXIT_CRITICAL();
}
