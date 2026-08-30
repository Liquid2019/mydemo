#include "app_tasks.h"
#include "app_shared.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "MPU6050.h"
#include "PID.h"
#include "BlueSerial.h"
#include "NRF24L01.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PRIO_CONTROL    4
#define PRIO_COMM       3
#define PRIO_UI         2

#define STACK_CONTROL   256
#define STACK_COMM      256
#define STACK_UI        256

static SemaphoreHandle_t semControl;

static void TaskControl(void *arg);
static void TaskComm(void *arg);
static void TaskUI(void *arg);

void AppTasks_Create(void)
{
	semControl = xSemaphoreCreateBinary();
	configASSERT(semControl != NULL);

	xTaskCreate(TaskControl, "ctrl", STACK_CONTROL, NULL, PRIO_CONTROL, NULL);
	xTaskCreate(TaskComm,    "comm", STACK_COMM,    NULL, PRIO_COMM,    NULL);
	xTaskCreate(TaskUI,      "ui",   STACK_UI,      NULL, PRIO_UI,      NULL);
}

void TIM1_UP_IRQHandler(void)
{
	BaseType_t hpTaskWoken = pdFALSE;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		if (semControl != NULL)
		{
			xSemaphoreGiveFromISR(semControl, &hpTaskWoken);
			portYIELD_FROM_ISR(hpTaskWoken);
		}
	}
}

static void TaskControl(void *arg)
{
	static uint16_t SensorCount0, SensorCount1;
	static uint16_t RunCount0, RunCount1;

	(void)arg;

	for (;;)
	{
		xSemaphoreTake(semControl, portMAX_DELAY);

		Key_Tick();

		SensorCount0++;
		if (SensorCount0 >= ANGLE_T)
		{
			SensorCount0 = 0;

			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			GY_Cali = GY + GY_Offset;
			AngleAcc = -atan2(AX, AZ) / 3.1415926535f * 180.0f;
			AngleDelta = GY_Cali / 32768.0f * 2000.0f * (ANGLE_T / 1000.0f);
			AngleAcc_Cali = AngleAcc + AngleAcc_Offset;

			{
				float Alpha0 = 0.8f;
				AngleAcc_Filter = Alpha0 * AngleAcc_Filter + (1.0f - Alpha0) * AngleAcc_Cali;
			}

			Angle += AngleDelta;

			{
				float Alpha1 = fabs(DifSpeed) / 5.0f * 0.02f + 0.005f;
				if (Alpha1 > 0.02f) { Alpha1 = 0.02f; }
				Angle = Alpha1 * AngleAcc_Filter + (1.0f - Alpha1) * Angle;
			}

			if (Angle > 50.0f || Angle < -50.0f)
			{
				if (RunFlag)
				{
					RunFlag = 0;
					RunFlagUpdate = 1;
				}
			}
		}

		SensorCount1++;
		if (SensorCount1 >= SPEED_T)
		{
			SensorCount1 = 0;
			SpeedLeft = Encoder_Get(1) / 408.0f / (SPEED_T / 1000.0f);
			SpeedRight = Encoder_Get(2) / 408.0f / (SPEED_T / 1000.0f);
			AveSpeed = (SpeedLeft + SpeedRight) / 2.0f;
			DifSpeed = SpeedLeft - SpeedRight;
		}

		if (RunFlag)
		{
			RunCount0++;
			if (RunCount0 >= ANGLE_T)
			{
				RunCount0 = 0;

				AnglePID.Actual = Angle;
				PID_Update(&AnglePID);
				AvePWM = -AnglePID.Out;

				PWML = (int16_t)(AvePWM + DifPWM);
				PWMR = (int16_t)(AvePWM - DifPWM);

				if (PWML > 100) { PWML = 100; } else if (PWML < -100) { PWML = -100; }
				if (PWMR > 100) { PWMR = 100; } else if (PWMR < -100) { PWMR = -100; }

				Motor_SetPWM(1, PWML);
				Motor_SetPWM(2, PWMR);
			}

			RunCount1++;
			if (RunCount1 >= SPEED_T)
			{
				RunCount1 = 0;

				AppShared_EnterCritical();
				SpeedPID.Actual = AveSpeed;
				PID_Update(&SpeedPID);
				AnglePID.Target = SpeedPID.Out;

				TurnPID.Actual = DifSpeed;
				PID_Update(&TurnPID);
				DifPWM = TurnPID.Out;
				AppShared_ExitCritical();
			}
		}
		else
		{
			Motor_SetPWM(1, 0);
			Motor_SetPWM(2, 0);
		}
	}
}

static void TaskComm(void *arg)
{
	(void)arg;

	for (;;)
	{
		/* 蓝牙遥控（前进/转向 + 按键映射） */
		if (BlueSerial_ReceiveFlag())
		{
			BlueSerial_Receive();

			if (strcmp(BlueSerial_StringArray[0], "joystick") == 0)
			{
				int8_t LV = (int8_t)atoi(BlueSerial_StringArray[2]);
				int8_t RH = (int8_t)atoi(BlueSerial_StringArray[3]);

				AppShared_EnterCritical();
				SpeedPID.Target = LV / 100.0f * SpeedLevel;
				TurnPID.Target = RH / 100.0f * SpeedLevel;
				AppShared_ExitCritical();
			}
			else if (strcmp(BlueSerial_StringArray[0], "key") == 0)
			{
				if (strcmp(BlueSerial_StringArray[1], "1") == 0
				 && strcmp(BlueSerial_StringArray[2], "up") == 0)
				{
					Key_Flag[KEY_1] |= KEY_SINGLE;
				}
				else if (strcmp(BlueSerial_StringArray[1], "2") == 0
					  && strcmp(BlueSerial_StringArray[2], "up") == 0)
				{
					Key_Flag[KEY_2] |= KEY_SINGLE;
				}
				else if (strcmp(BlueSerial_StringArray[1], "3") == 0
					  && strcmp(BlueSerial_StringArray[2], "up") == 0)
				{
					Key_Flag[KEY_3] |= KEY_SINGLE;
				}
			}
		}

		/* NRF 遥控器 */
		if (NRF24L01_Receive() == 1)
		{
			int8_t LV = (int8_t)NRF24L01_RxPacket[2];
			int8_t RH = (int8_t)NRF24L01_RxPacket[3];
			uint8_t KEY0 = NRF24L01_RxPacket[5];

			AppShared_EnterCritical();
			SpeedPID.Target = LV / 100.0f * SpeedLevel;
			TurnPID.Target = RH / 100.0f * SpeedLevel;
			AppShared_ExitCritical();

			if (KEY0 & 0x01) { Key_Flag[KEY_3] |= KEY_SINGLE; }
			if (KEY0 & 0x02) { Key_Flag[KEY_2] |= KEY_SINGLE; }
			if (KEY0 & 0x04) { Key_Flag[KEY_1] |= KEY_SINGLE; }
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

static void TaskUI(void *arg)
{
	(void)arg;

	for (;;)
	{
		if (RunFlag) { LED_ON(); } else { LED_OFF(); }

		/* K1：启动 / 停止平衡 */
		if (Key_Check(KEY_1, KEY_SINGLE))
		{
			if (RunFlag == 0)
			{
				PID_Init(&AnglePID);
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
				Angle = AngleAcc_Filter;
				RunFlag = 1;
			}
			else
			{
				RunFlag = 0;
			}
		}

		if (RunFlagUpdate)
		{
			RunFlagUpdate = 0;
		}

		/* K2 / K3：速度档位 */
		if (Key_Check(KEY_2, KEY_SINGLE))
		{
			if (SpeedLevel > 1) { SpeedLevel--; }
		}
		if (Key_Check(KEY_3, KEY_SINGLE))
		{
			if (SpeedLevel < 6) { SpeedLevel++; }
		}

		/* 简易运行状态 */
		{
			float angleNow;
			uint8_t running;
			uint16_t level;

			AppShared_EnterCritical();
			angleNow = Angle;
			running = RunFlag;
			level = SpeedLevel;
			AppShared_ExitCritical();

			OLED_Printf(0, 0, OLED_8X16, "Balance RTOS    ");
			OLED_Printf(0, 16, OLED_8X16, "Run:%s     ", running ? "ON " : "OFF");
			OLED_Printf(0, 32, OLED_8X16, "Angle:%+06.1f ", angleNow);
			OLED_Printf(0, 48, OLED_8X16, "SpdLv:%u K1启停 ", (unsigned)level);
			OLED_Update();
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
