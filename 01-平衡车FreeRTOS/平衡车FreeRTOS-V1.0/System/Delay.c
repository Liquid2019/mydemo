#include "stm32f10x.h"
#include "Delay.h"
#include "FreeRTOS.h"
#include "task.h"

/* 不占用 SysTick（已交给 FreeRTOS），用循环近似延时 @72MHz */
void Delay_us(uint32_t xus)
{
	volatile uint32_t n = xus * 8U;
	while (n--)
	{
		__NOP();
	}
}

void Delay_ms(uint32_t xms)
{
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		vTaskDelay(pdMS_TO_TICKS(xms));
	}
	else
	{
		while (xms--)
		{
			Delay_us(1000);
		}
	}
}

void Delay_s(uint32_t xs)
{
	while (xs--)
	{
		Delay_ms(1000);
	}
}
