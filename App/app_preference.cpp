#include "app_preference.h"

ID_Data_t ID_Data[ID_e_count];

void Prefence_Init(void)
{
	for(int i = 0;i < ID_e_count;i++)
	{
		ID_Data[i].Data_ID = ID_e(i);
	}
}

#ifndef __APP_PREFERENCE_H
#error 缺少配置文件，请使用下面的备份创建 app_preference.h 在App/Inc下
#endif
/*2023.4.29备份
#ifndef __APP_PREFERENCE_H
#define __APP_PREFERENCE_H

//忽略警告，摆烂
#pragma diag_suppress 177
#pragma diag_suppress 550
#pragma diag_suppress 3337

//轮组选择 useMecanum useSteering
#define useSteering

//发送 格式 Serialx_Ctrl （x用3，6，7，8代替），串口不能重复！
#define JUDGE_SERIAL Serial7_Ctrl
#define JudgeData SerialData7
#define JUDGE_SERIAL_BAUD 115200
#define GIMBAL_SERIAL Serial6_Ctrl
#define GimbalData SerialData6
#define GIMBAL_SERIAL_BAUD 115200

//无参数则为 NULL ，会直接跳过。
//Lenth用于检验数据长度，Buffer_size用于创建环形缓冲区和数据缓冲区，为0则中断直接发送通知给Message
//Serialx_ITPending 可选 USART_IT_IDLE USART_IT_RXNE USART_IT_RXNE_AND_IDLE
#define Serial3_Data_Header 0xff
#define Serial3_Data_Tail NULL
#define Serial3_Data_Lenth NULL
#define Serial3_Buffer_Size 30
#define Serial3_ITPending USART_IT_RXNE_AND_IDLE

#define Serial6_Data_Header 0xff
#define Serial6_Data_Tail NULL
#define Serial6_Data_Lenth NULL
#define Serial6_Buffer_Size 30
#define Serial6_ITPending USART_IT_RXNE_AND_IDLE

#define Serial7_Data_Header NULL
#define Serial7_Data_Tail NULL
#define Serial7_Data_Lenth NULL
#define Serial7_Buffer_Size 0
#define Serial7_ITPending USART_IT_IDLE

#define Serial8_Data_Header NULL
#define Serial8_Data_Tail NULL
#define Serial8_Data_Lenth NULL
#define Serial8_Buffer_Size 0
#define Serial8_ITPending USART_IT_IDLE

//云台电机相对底盘正方向的ECD
#define Gimbal_Motor_Yaw_Offset_ECD 7375

//选择遥控器控制模式(三挡位功能如下0:不跟随 跟随 小陀螺 1:不跟随 视觉 视觉发弹)
#define RC_CONTRAL_MODE 0
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.003030303f//0.0015f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.003030303f//0.0015f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

//等级速度对应表
#define CHASSIS_SPEED_GEAR_0 {0.5f, 1.f, 1.f}
#define CHASSIS_SPEED_GEAR_1 {1.f, 1.5f, 1.5f}
#define CHASSIS_SPEED_GEAR_2 {2.f, 2.5f, 2.f}
#define CHASSIS_SPEED_GEAR_3 {3.f, 3.5f, 2.f}

//遥控器死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔
#define CHASSIS_CONTROL_TIME (0.001 * CHASSIS_CONTROL_TIME_MS)
//底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE (1000 / CHASSIS_CONTROL_TIME_MS)
//底盘3508最大can发送电流值
#define MAX_MOTOR_3508_CAN_CURRENT 20000.0f
//底盘6020最大can发送电压值
#define MAX_MOTOR_6020_CAN_CURRENT 30000.0f

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机或轮子需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘电机最大速度
#define MAX_WHEEL_SPEED 10.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 10.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 10.0f
//底盘运动过程最大旋速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 10.0f

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.1f

//底盘功率速度控制PID
#define VELOCILY_SPEED_PID_KP 0.001f
#define VELOCILY_SPEED_PID_KI 0.00052f
#define VELOCILY_SPEED_PID_KD 0.0f
#define VELOCILY_SPEED_PID_MAX_OUT 5.0f
#define VELOCILY_SPEED_PID_MAX_IOUT 4.0f
#define VELOCILY_SPEED_PID_BAND_I  3000.0f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 1.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_3508_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define M3505_MOTOR_SPEED_PID_BAND_I  3000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 1.1f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.001f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 2000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 1000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_BAND_I 3000.0f

#ifdef useSteering
//底盘6020ecd偏差值
#define MOTOR_6020_1_offset 2082
#define MOTOR_6020_2_offset 719
#define MOTOR_6020_3_offset 7511
#define MOTOR_6020_4_offset 3450
// 6020 ecd中值
const static float MOTOR_6020_offset[4] = { MOTOR_6020_1_offset, MOTOR_6020_2_offset, MOTOR_6020_3_offset, MOTOR_6020_4_offset };

//底盘6020电机角度环PID
#define M6020_MOTOR_ANGLE_PID_KP 10.0f
#define M6020_MOTOR_ANGLE_PID_KI 0.0f
#define M6020_MOTOR_ANGLE_PID_KD 1.0f
#define M6020_MOTOR_ANGLE_PID_MAX_OUT MAX_MOTOR_6020_CAN_CURRENT
#define M6020_MOTOR_ANGLE_PID_MAX_IOUT 2000.0f
#define M6020_MOTOR_ANGLE_PID_BAND_I 3000.0f
//底盘6020电机速度环PID
#define M6020_MOTOR_SPEED_PID_KP 2.0f
#define M6020_MOTOR_SPEED_PID_KI 0.005f
#define M6020_MOTOR_SPEED_PID_KD 0.0f
#define M6020_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_6020_CAN_CURRENT
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#define M6020_MOTOR_SPEED_PID_BAND_I 3000.0f
#endif

#ifndef useMecanum
#define useMecanum
#endif
#ifdef useSteering
#undef useMecanum
#endif

typedef enum
{
	FaultData = 0x00,
	CanData1,
	CanData2,
	SerialData3,
	SerialData6,
	SerialData7,
	SerialData8,
	RCData,
	RefereeData,
	MessageData,
	ChassisData,
	UIdrawData,
	CorrespondenceData,
	SupercapData,
	ID_e_count
}ID_e;

struct ID_Data_t
{
	ID_e Data_ID;
	void *Data_Ptr;
};

extern ID_Data_t ID_Data[ID_e_count];

void Prefence_Init(void);
#endif

*/
