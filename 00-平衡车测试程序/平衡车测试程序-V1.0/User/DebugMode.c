#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "MPU6050.h"
#include "Timer.h"
#include "PID.h"
#include "BlueSerial.h"
#include "Serial.h"
#include "NRF24L01.h"
#include "Store.h"
#include <math.h>
#include <stdio.h>

extern int16_t AX, AY, AZ, GX, GY, GZ;
extern int16_t GY_Offset;

extern float AngleAcc;
extern float AngleAcc_Offset;

extern uint8_t DebugFlag;
extern uint16_t SpeedLevel; 

void SaveParam(void)
{
	/*保存参数至FLASH*/
	Store_Data[1] = SpeedLevel;
	Store_Data[2] = GY_Offset;
	Store_Data[3] = *((uint16_t *)&AngleAcc_Offset);;
	Store_Data[4] = *((uint16_t *)&AngleAcc_Offset + 1);;
	Store_Save();
}

void LoadParam(void)
{
	/*从FLASH加载参数*/
	uint32_t Temp;
	SpeedLevel = Store_Data[1];
	GY_Offset = Store_Data[2];
	Temp = (Store_Data[4] << 16) | Store_Data[3];
	AngleAcc_Offset = *(float *)&Temp;
}

void HardwareTest(void)			//硬件测试
{
	uint8_t PageSelect = 1;
	int16_t PWML = 0, PWMR = 0;
	int16_t SpdL = 0, SpdR = 0;
	int16_t LocL = 0, LocR = 0;
	
	uint8_t WriteArray[5] = {0};
	uint8_t ReadArray[5] = {0};
	
	uint8_t TX1 = 0, RX1 = 0;
	uint8_t TX2 = 0, RX2 = 0;
	
	OLED_Clear();
	
	while (1)
	{
		if (Key_Check(KEY_4, KEY_SINGLE))	//K4键切换测试项目
		{
			Key_Clear();
			OLED_Clear();
			PageSelect ++;
			if (PageSelect > 4)
			{
				PageSelect = 1;
			}
			PWML = 0;
			PWMR = 0;
			Motor_SetPWM(1, PWML);
			Motor_SetPWM(2, PWMR);
		}
		if (Key_Check(KEY_4, KEY_LONG)) {Key_Clear(); return;}	//K4键长按退出
		
		if (PageSelect == 1)		//编码电机测试
		{
			if (Key_Check(KEY_1, KEY_SINGLE))	//K1键增大PWM
			{
				if (PWML < 100) {PWML += 10;}
				if (PWMR < 100) {PWMR += 10;}
			}
			if (Key_Check(KEY_2, KEY_SINGLE))	//K2键减小PWM
			{
				if (PWML > -100) {PWML -= 10;}
				if (PWMR > -100) {PWMR -= 10;}
			}
			if (Key_Check(KEY_3, KEY_SINGLE))	//K3键PWM归零
			{
				PWML = 0;
				PWMR = 0;
			}
			
			Motor_SetPWM(1, PWML);		//设定PWM
			Motor_SetPWM(2, PWMR);
			
			SpdL = Encoder_Get(1);		//读取编码器速度
			SpdR = Encoder_Get(2);
			
			LocL += SpdL; LocL %= 1000;	//累加得到位置
			LocR += SpdR; LocR %= 1000;
						
			OLED_Printf(0, 0, OLED_8X16,  "编码电机测试 1/4");
			OLED_Printf(0, 16, OLED_8X16, "PWML:%+04d R:%+04d", PWML, PWMR);
			OLED_Printf(0, 32, OLED_8X16, "SpdL:%+04d R:%+04d", SpdL, SpdR);
			OLED_Printf(0, 48, OLED_8X16, "LocL:%+04d R:%+04d", LocL, LocR);
			OLED_Update();
			
			Delay_ms(50);
		}
		else if (PageSelect == 2)	//MPU6050测试
		{
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);		//读取MPU6050原始数据
			
			OLED_Printf(0, 0, OLED_8X16,  "MPU6050测试  2/4");
			OLED_Printf(0, 16, OLED_8X16, "%+06d %+06d", AX, GX);
			OLED_Printf(0, 32, OLED_8X16, "%+06d %+06d", AY, GY);
			OLED_Printf(0, 48, OLED_8X16, "%+06d %+06d", AZ, GZ);
			OLED_Printf(112, 16, OLED_8X16, "%02X", MPU6050_GetID());
			OLED_Update();
			
			Delay_ms(50);
		}
		else if (PageSelect == 3)	//NRF24L01测试
		{
			/*通过读写寄存器进行简单的硬件测试*/
			/*在NRF24L01_TX_ADDR寄存器写入测试数据，随后读取寄存器*/
			/*如果写入寄存器的值和读取寄存器的值一致，则测试通过*/
			
			if (Key_Check(KEY_1, KEY_SINGLE))		//K1键增加写入的测试数据
			{
				WriteArray[0] += 1;
				WriteArray[1] += 2;
				WriteArray[2] += 3;
				WriteArray[3] += 4;
				WriteArray[4] += 5;
				NRF24L01_WriteRegs(NRF24L01_TX_ADDR, WriteArray, 5);
			}
			if (Key_Check(KEY_2, KEY_SINGLE))		//K2键减小写入的测试数据
			{
				WriteArray[0] -= 1;
				WriteArray[1] -= 2;
				WriteArray[2] -= 3;
				WriteArray[3] -= 4;
				WriteArray[4] -= 5;
				NRF24L01_WriteRegs(NRF24L01_TX_ADDR, WriteArray, 5);
			}
			
			NRF24L01_ReadRegs(NRF24L01_TX_ADDR, ReadArray, 5);		//读取寄存器，观察是否与写入一致
			
			OLED_Printf(0, 0, OLED_8X16,  "NRF24L01测试 3/4");
			OLED_Printf(0, 16, OLED_8X16, "W:%02X %02X %02X %02X %02X", 
				WriteArray[0], WriteArray[1], WriteArray[2], WriteArray[3], WriteArray[4]);
			OLED_Printf(0, 32, OLED_8X16, "R:%02X %02X %02X %02X %02X", 
				ReadArray[0], ReadArray[1], ReadArray[2], ReadArray[3], ReadArray[4]);
			OLED_Update();
			
			Delay_ms(50);
		}
		else if (PageSelect == 4)	//串口测试，TX1/RX1为板子右侧串口，TX2/RX2为板子左侧蓝牙串口
		{
			if (Key_Check(KEY_1, KEY_SINGLE))		//K1键增加测试数据
			{
				TX1 += 1;
				TX2 += 2;
			}
			if (Key_Check(KEY_2, KEY_SINGLE))		//K2键减小测试数据
			{
				TX1 -= 1;
				TX2 -= 2;
			}
			if (Key_Check(KEY_3, KEY_SINGLE))		//K3键发送测试数据
			{
				Serial_SendByte(TX1);
				BlueSerial_SendByte(TX2);
				
				OLED_ReverseArea(32, 16, 16, 16);	//发送时反色闪烁
				OLED_ReverseArea(96, 16, 16, 16);
				OLED_Update();
				Delay_ms(100);
				OLED_ReverseArea(32, 16, 16, 16);
				OLED_ReverseArea(96, 16, 16, 16);
				OLED_Update();
			}
			
			if (Serial_GetRxFlag())			//判断串口是否收到数据
			{
				RX1 = Serial_GetRxData();	//收到时显示数据并反色闪烁
				OLED_Printf(0, 32, OLED_8X16,  "RX1:%02X  RX2:%02X", RX1, RX2);
				OLED_ReverseArea(32, 32, 16, 16);
				OLED_Update();
				Delay_ms(100);
				OLED_ReverseArea(32, 32, 16, 16);
				OLED_Update();
			}
			
			if (BlueSerial_Length() > 0)	//判断蓝牙串口是否收到数据
			{
				BlueSerial_Get(&RX2);		//收到时显示数据并反色闪烁
				OLED_Printf(0, 32, OLED_8X16,  "RX1:%02X  RX2:%02X", RX1, RX2);
				OLED_ReverseArea(96, 32, 16, 16);
				OLED_Update();
				Delay_ms(100);
				OLED_ReverseArea(96, 32, 16, 16);
				OLED_Update();
			}
			
			OLED_Printf(0, 0, OLED_8X16,  "串口测试     4/4");
			OLED_Printf(0, 16, OLED_8X16,  "TX1:%02X  TX2:%02X", TX1, TX2);
			OLED_Printf(0, 32, OLED_8X16,  "RX1:%02X  RX2:%02X", RX1, RX2);
			OLED_Update();
			
			Delay_ms(50);
		}
	}
}

void SensorCalibration(void)		//传感器校准
{
	float GY_Array[20] = {0};
	float AngleAcc_Array[20] = {0};
	uint8_t p = 0;
	float GY_Sum, GY_Ave;
	float AngleAcc_Sum, AngleAcc_Ave;
	
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16,  "GY:+00000.00    ");
	OLED_Printf(0, 16, OLED_8X16, "Angle:+000.00   ");
	OLED_Printf(0, 32, OLED_8X16, " 保持竖直且静止 ");
	OLED_Printf(0, 48, OLED_8X16, "  单击K4键确认  ");
	
	/*测试前保持平衡车绝对竖直且静止，否则校准值可能有误*/
	/*校准项目，第一个是静止时陀螺仪的漂移，第二个是竖直时中心角度的偏差*/
	
	while (1)
	{
		if (Key_Check(KEY_4, KEY_SINGLE)) {Key_Clear(); break;}		//K4键确认
		if (Key_Check(KEY_4, KEY_LONG)) {Key_Clear(); return;}		//K4键长按退出
		
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);		//读取MPU6050原始数据
		AngleAcc = -atan2(AX, AZ) / 3.1415926535 * 180;		//根据角速度计计算中心角度
		
		/*取20次测量结果，计算平均值，避免噪声干扰*/
		GY_Array[p] = GY;
		AngleAcc_Array[p] = AngleAcc;
		p ++;
		p %= 20;
		
		GY_Sum = 0;
		AngleAcc_Sum = 0;
		for (uint8_t i = 0; i < 20; i ++)
		{
			GY_Sum += GY_Array[i];
			AngleAcc_Sum += AngleAcc_Array[i];
		}
		GY_Ave = GY_Sum / 20.0;
		AngleAcc_Ave = AngleAcc_Sum / 20.0;
		
		OLED_Printf(0, 0, OLED_8X16, "GY:%+09.2f", GY_Ave);
		OLED_Printf(0, 16, OLED_8X16, "Angle:%+07.2f", AngleAcc_Ave);
		OLED_Update();
		
	}
	
	/*确认后，退出循环，校准完成*/
	OLED_Printf(0, 32, OLED_8X16, "    校准完成    ");
	OLED_Printf(0, 48, OLED_8X16, "                ");
	OLED_Update();
	
	Delay_ms(2000);
	
	GY_Offset = -GY_Ave;				//绝对静止时，负的陀螺仪漂移为校准值
	AngleAcc_Offset = -AngleAcc_Ave;	//绝对竖直时，负的中心角度为校准值
	
	SaveParam();		//保持参数至FLASH，掉电不丢失
}

void DebugMode(void)
{
	while (1)
	{
		OLED_Clear();
		OLED_Printf(0, 0, OLED_8X16,  "   [调试模式]   ");
		OLED_Printf(0, 16, OLED_8X16, "K1：硬件测试    ");
		OLED_Printf(0, 32, OLED_8X16, "K2：传感器校准  ");
		OLED_Printf(0, 48, OLED_8X16, "  长按K4键返回  ");
		OLED_Update();
		
		if (Key_Check(KEY_1, KEY_SINGLE)) {Key_Clear();HardwareTest();}			//K1键硬件测试
		if (Key_Check(KEY_2, KEY_SINGLE)) {Key_Clear();SensorCalibration();}	//K2键传感器校准
		if (Key_Check(KEY_4, KEY_LONG)) {DebugFlag = 0; OLED_Clear(); Key_Clear(); break;}	//K4键长按退出
	}
}
