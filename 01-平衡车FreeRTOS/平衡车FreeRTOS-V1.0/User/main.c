#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "MPU6050.h"
#include "Timer.h"
#include "BlueSerial.h"
#include "NRF24L01.h"
#include "app_shared.h"
#include "app_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	OLED_Init();
	LED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	MPU6050_Init();
	BlueSerial_Init();
	NRF24L01_Init();
	Timer_Init();

	/* 固定运行参数，不再进入校准/调参流程 */
	GY_Offset = 0;
	AngleAcc_Offset = 0.0f;
	SpeedLevel = 5;

	OLED_Clear();
	OLED_ShowString(0, 0,  "   [江协科技]   ", OLED_8X16);
	OLED_ShowString(0, 16, " 平衡车 FreeRTOS", OLED_8X16);
	OLED_ShowString(0, 32, "   运行精简版   ", OLED_8X16);
	OLED_ShowString(0, 48, "          K4进入", OLED_8X16);
	OLED_Update();

	while (Key_Check(KEY_4, KEY_SINGLE) == 0)
	{
		Key_Tick();
		Delay_ms(1);
	}
	Key_Clear();

	OLED_Clear();
	BlueSerial_ClearBuffer();

	AppTasks_Create();
	vTaskStartScheduler();

	while (1)
	{
	}
}
