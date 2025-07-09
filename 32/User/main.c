#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Serial.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
int cy=0,cx=0;
//----------------------pid角度环控制

float kp=0.8,ki=0.01,kd=0.1;
float erro=0,erro_last=0,I=0;
float pid_value=0;
void pid_motor_forward(int angle,int speed)
{
	erro=0-angle;
	I+=erro;
	if(I>1000)
	{
		I=1000;
	}
	if(I<-1000)
	{
		I=-1000;
		
	}
	pid_value=kp*erro+ki*I+kd*(erro-erro_last);
	if(pid_value>speed)
	{
		pid_value=speed;
	}
	if(pid_value<-speed)
	{
		pid_value=-speed;
	}
  MotorA_SetSpeed((int)(speed-pid_value));
	MotorB_SetSpeed((int)(speed+pid_value));
	erro_last=erro;
}// 左边角度为正
void pid_motor_inplace(int angle,int speed)
{
	// 由于移除了MPU6050，此函数不再依赖Yaw，但保留其结构以避免其他潜在错误
	// 如果此函数不再需要，可以考虑删除
	erro=angle;// 陀螺仪pid时，angle-YAW
	I+=erro;
	if(I>1000)
	{
		I=1000;
	}
	if(I<-1000)
	{
		I=-1000;
		
	}
	pid_value=kp*erro+ki*I+kd*(erro-erro_last);
	if(pid_value>speed)
	{
		pid_value=speed;
	}
	if(pid_value<-speed)
	{
		pid_value=-speed;
	}
  MotorA_SetSpeed(-(int)pid_value);
	MotorB_SetSpeed(+(int)pid_value);
	erro_last=erro;
}

//_________统一串口解析函数_________
// 返回值: 0=启动, 1=左转, 2=右转, 3=循迹数据, 8=停止, -1=无有效数据
int Parse_Serial_Data(void)
{
	int result = -1; // 默认为无效数据
	if (Serial_RxFlag == 1)
	{
		char* p = Serial_RxPacket;
		switch(p[0])
		{
			case '0':
				result = 0; // 启动指令
				break;
			case '1':
				result = 1; // 左转指令
				break;
			case '2':
				result = 2; // 右转指令
				break;
			case '@':       // 循迹数据
				{
					// 解析循迹数据: "@rho_err,theta_err,mag"
					char* rho_str = strtok(p + 1, ",");
					char* theta_str = strtok(NULL, ",");
					if (rho_str && theta_str)
					{
						// OpenMV发来的theta_err就是我们需要的偏差值
						// 将其赋值给全局变量cx，以复用pid_motor_forward函数
						cx = (int)atof(theta_str); 
						result = 3; // 循迹数据
					}
				}
				break;
			case 'H':
				result = 8; // 停止指令
				break;
		}
		Serial_RxFlag = 0; // 清除标志位
	}
	return result;
}

int main(void)
{  
	OLED_Init();
	Serial_Init();
	Motor_Init();
	Move(200);
	Delay_ms(1000);
	Motor_TurnInPlace(150,1);
	Delay_ms(1000);
	Motor_TurnInPlace(150,0);
	Delay_ms(1000);
	Motor_Stop();

}
// 	// 1. 等待启动指令 '0'
// 	while(Parse_Serial_Data() != 0)
// 	{
// 		Delay_ms(100);
// 	}

// 	OLED_Clear();
// 	OLED_ShowString(1, 1, "Start Tracking...");
	
// 	// 2. 进入主循环 + 等待转向指令
// 	while (1)
// 	{
// 		int cmd = Parse_Serial_Data();
		
// 		if (cmd == 3) // 收到循迹数据
// 		{
// 			// 根据OpenMV传来的PID循迹数据控制小车，速度150
// 			pid_motor_forward(cx, 150); 
// 			OLED_ShowString(1, 1, "Tracking...");
// 			OLED_ShowString(2, 1, "CX:");
// 			OLED_ShowSignedNum(2, 4, cx, 5);
// 		}
// 		else if (cmd == 1) // 收到左转指令
// 		{
// 			OLED_Clear();
// 			OLED_ShowString(1, 1, "CMD: 1, Turn Left");
// 			MotorA_SetSpeed(0); // 停止电机
// 			MotorB_SetSpeed(0); // 停止电机
// 			OLED_Clear();
// 			OLED_ShowString(1, 1, "Turn function removed."); // 提示转向功能已移除
// 		}
// 		else if (cmd == 2) // 收到右转指令
// 		{
// 			OLED_Clear();
// 			OLED_ShowString(1, 1, "CMD: 2, Turn Right");
// 			MotorA_SetSpeed(0); // 停止电机
// 			MotorB_SetSpeed(0); // 停止电机
// 			OLED_Clear();
// 			OLED_ShowString(1, 1, "Turn function removed."); // 提示转向功能已移除
// 		}
// 		else if (cmd == 8) // 收到停止指令
// 		{
// 			OLED_Clear();
// 			OLED_ShowString(1, 1, "CMD: H, Halt");
// 			MotorA_SetSpeed(0);
// 			MotorB_SetSpeed(0);
// 			// 如果需要彻底停止程序，可以在这里break跳出while循环
// 		}
// 		// 如果没有收到任何有效指令(cmd == -1)，小车会保持上一个速度状态
// 		// 除非收到停止指令，否则不会停止。如果需要没有收到数据就停止，请在else if (cmd == 8)后添加 MotorA_SetSpeed(0); MotorB_SetSpeed(0);
// 	}
// }
