#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Motor.h"
#include "Serial.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h" // 用于fabs函数
#include "ENCODER.h" // 包含编码器头文件

// 小车参数宏定义
#define WHEEL_DIAMETER_MM   96.0  // 车轮直径，单位毫米 (2 * 48mm)
#define WHEEL_BASE_MM       280.0 // 轮距，单位毫米 (14cm)
#define ENCODER_PPR         1040  // 编码器每圈脉冲数
#define PI                  3.1415926535f

volatile uint32_t g_usart_rx_count = 0; // 串口接收中断计数器
int left_current_pulses;
int right_current_pulses;
int cy=0,cx=0;
//----------------------pid角度环控制

float kp=0.8,ki=0.01,kd=0.1;
float erro=0,erro_last=0,I=0;
float pid_value=0;
void pid_motor_forward(int angle,int speed)  //巡线PID
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


// 使用编码器实现任意角度转弯（带完整PID控制）
// angle: 要旋转的角度，正数向右转，负数向左转
// max_speed: 转弯时的最大速度
void turn_degrees(float angle, int max_speed) {
    // 计算旋转指定角度时，车体中心转过的圆心角对应的弧长
    // 弧长 = (角度 / 360) * PI * 轮距
    float center_arc_length = (fabs(angle) / 360.0f) * PI * WHEEL_BASE_MM;
    
    // 计算每个轮子需要转动的圈数
    // 圈数 = 弧长 / 轮子周长
    float turns = center_arc_length / (PI * WHEEL_DIAMETER_MM);
    
    // 计算所需的总脉冲数
    int required_pulses = (int)(turns * ENCODER_PPR);

    // --- PID 参数 ---
    float turn_kp = 1.8f;
    float turn_ki = 0.005f;
    float turn_kd = 2.0f;
    
    int min_speed = 60;

    // PID 控制器变量
    float integral = 0;
    float last_error = 0;
		
		// 根据角度确定旋转方向
		int direction = (angle > 0) ? 1 : -1;

    Clear_Encoder_Count();

    while (1) {
        // 读取编码器值，并根据方向统一为正数处理
        left_current_pulses = abs(Read_Left_Encoder());
        right_current_pulses = abs(Read_Right_Encoder());

        int average_pulses = (left_current_pulses + right_current_pulses) / 2;
        
        int error = required_pulses - average_pulses;
        integral += error;
        float derivative = error - last_error;

        if (abs(error) <= 20 && abs(derivative) <= 5) {
            MotorA_SetSpeed(0);
            MotorB_SetSpeed(0);
            break;
        }

        int pid_output = (int)(turn_kp * error + turn_ki * integral + turn_kd * derivative);
        
        if (pid_output > max_speed) {
            pid_output = max_speed;
            integral -= error; 
        }
        if (pid_output < -max_speed) {
            pid_output = -max_speed;
            integral -= error;
        }
				
				if (pid_output > 0 && pid_output < min_speed) {
						pid_output = min_speed;
				} else if (pid_output < 0 && pid_output > -min_speed) {
						pid_output = -min_speed;
				}

        last_error = error;

        // 根据direction变量设置电机转动方向
        MotorA_SetSpeed(direction * pid_output);
        MotorB_SetSpeed(-direction * pid_output);

        OLED_ShowString(2, 1, "Err: ");
        OLED_ShowSignedNum(2, 5, error, 5);
        OLED_ShowString(3, 1, "PID: ");
        OLED_ShowSignedNum(3, 5, pid_output, 5);
        OLED_ShowString(4, 1, "Ang: ");
        OLED_ShowSignedNum(4, 5, (int)angle, 5);

        Delay_ms(10);
    }
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
}

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

// 定义小车运行状态
typedef enum {
    STATE_WAITING,  // 等待启动指令
    STATE_TRACKING, // 循迹状态
		STATE_STOPPED   // 停止状态
} CarState;

int main(void)
{
    // --- 初始化 ---
    OLED_Init();
    Serial_Init();
    Motor_Init();
    Encoder_Init();

    CarState current_state = STATE_WAITING;
    int command = -1;
		int tracking_speed = 150; // 设定循迹速度
		int turning_speed = 150;  // 设定转向速度

    OLED_ShowString(1, 1, "System Ready.");
    OLED_ShowString(2, 1, "State: WAITING");

    while(1)
    {
        command = Parse_Serial_Data(); // 持续解析串口指令

        // --- 调试代码：显示中断计数 ---
        OLED_ShowString(4, 1, "RX_CNT:");
        OLED_ShowNum(4, 8, g_usart_rx_count, 5);
        // -----------------------------

        switch (current_state)
        {
            case STATE_WAITING:
                Move(0); // 确保电机停止
                if (command == 0) // 收到启动指令 '0'
                {
                    current_state = STATE_TRACKING;
                    OLED_Clear();
                    OLED_ShowString(1, 1, "State: TRACKING");
                }
                break;

            case STATE_TRACKING:
                if (command == 1) // 收到左转指令 '1'
                {
                    OLED_Clear();
                    OLED_ShowString(1, 1, "CMD: Turn Left");
                    Move(0); // 转弯前先停一下
                    Delay_ms(100);
                    turn_degrees(-90, turning_speed);
                    OLED_Clear();
                    OLED_ShowString(1, 1, "State: TRACKING");
                }
                else if (command == 2) // 收到右转指令 '2'
                {
                    OLED_Clear();
                    OLED_ShowString(1, 1, "CMD: Turn Right");
                       Move(0); // 转弯前先停一下
                    Delay_ms(100);
                    turn_degrees(90, turning_speed);
                    OLED_Clear();
                    OLED_ShowString(1, 1, "State: TRACKING");
                }
                else if (command == 3) // 收到循迹数据 '@'
                {
                    // 根据OpenMV传来的PID循迹数据控制小车
                    pid_motor_forward(cx, tracking_speed); 
                    OLED_ShowString(2, 1, "CX:");
                    OLED_ShowSignedNum(2, 4, cx, 5);
                }
                else if (command == 8) // 收到停止指令 'H'
                {
                    current_state = STATE_STOPPED;
                    Move(0); // 转弯前先停一下
                    OLED_Clear();
                    OLED_ShowString(1, 1, "State: STOPPED");
                    OLED_ShowString(2, 1, "Task Finished.");
                }
                // 如果没有收到任何有效指令(command == -1)，小车会保持上一个速度状态
                // 这对于连续循迹是必要的
                break;
						
						case STATE_STOPPED:
								// 在停止状态下，什么都不做，等待复位
								 Move(0); // 转弯前先停一下
								break;
        }
        // 可以在这里加一个小的延时，防止CPU占用过高，但要注意不能影响循迹的实时性
        // Delay_ms(1); 
    }
}
