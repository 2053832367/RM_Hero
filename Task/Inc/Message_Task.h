#ifndef Message_TASK_H
#define Message_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "dev_can.h"
#include "app_serial.h"
#include "protocol_dbus.h"
#include "protocol_judgement.h"
#include "drivers_statistic.h"

#ifdef __cplusplus
extern "C" {
#endif

	void Message_Task(void *pvParameters);
	void CAN1_Rx_Task(void *pvParameters);
	void CAN2_Rx_Task(void *pvParameters);
	void Serial_Rx_Task(void *pvParameters);
	void DR16_Rx_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif


extern QueueHandle_t Message_Queue;   		//消息队列句柄
extern QueueHandle_t Message_Queue;
extern QueueHandle_t CAN1_Rx_Queue;
extern QueueHandle_t CAN2_Rx_Queue;
extern QueueHandle_t Serial_Rx_Queue;
extern QueueHandle_t DR16_Rx_Queue;

#define VISUAL_TARGET_MAX 200

union F
{
	uint8_t I[4];
	fp32 F;
};

#define sampleFreq	512.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//卡尔曼滤波
#define Yaw_Q_angle  0.1f		//角度数据置信度，角度噪声的协方差  0.001f
#define Yaw_Q_gyro   0.3f		//角速度数据置信度，角速度噪声的协方差  0.003f
#define Yaw_R_angle  0.05f			//加速度计测量噪声的协方差  0.5f
static float PP[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P，初始值为单位阵

struct gyro_receive_data_t
{
	fp32 AngleP;//pitch
	fp32 AngleR;//roll
	fp32 AngleY;//yaw

	int16_t SpeedX;
	int16_t SpeedY;
	int16_t SpeedZ;

	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;

	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;
};


struct gyro_calc_data_t
{
	float twoKp;
	float twoKi;
	float q0, q1, q2, q3;

	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, hz, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float integralFBx, integralFBy, integralFBz;
	gyro_calc_data_t(void)
	{
		q0 = 1.0f;
		twoKp = twoKpDef;
		twoKi = twoKiDef;
	}
};

typedef struct //视觉目标速度测量
{
	int delay_cnt; //计算相邻两帧目标不变持续时间,用来判断速度是否为0
	int freq;
	int last_time;         //上次受到目标角度的时间
	float last_position;   //上个目标角度
	float speed;           //速度
	float last_speed;      //上次速度
	float processed_speed; //速度计算结果
} speed_calc_data_t;

struct gyro_angle_data_t
{
	fp32 AngleP_Calc;
	fp32 AngleR_Calc;
	fp32 AngleY_Calc;

	fp32 AngleP_Speed;
	fp32 AngleR_Speed;
	fp32 AngleY_Speed;

	fp32 AngleP_Acc;
	fp32 AngleR_Acc;
	fp32 AngleY_Acc;

	fp32 AngleP_Mag;
	fp32 AngleR_Mag;
	fp32 AngleY_Mag;
};

struct gyro_data_t
{
	gyro_receive_data_t data;
	gyro_calc_data_t calc;
	gyro_angle_data_t angle;

	fp32 Yaw_angle;
	fp32 Yaw_angle_offset;
	fp32 Yaw_speed;
	int32_t Yaw_cycle;
	fp32 Last_angle;
	fp32 Error_angle;
	fp32 Pitch_speed;

	uint32_t time[4];
	uint32_t last_time[4];
	uint32_t differ_time[4];

	Statistic Gyro_Acc_fps;
	Statistic Gyro_Speed_fps;
	Statistic Gyro_Mag_fps;
	Statistic Gyro_Angle_fps;
};

struct Visual_Receive_Data_t
{
	fp32 Error_X;
	fp32 Error_Y;
	fp32 Error_Z;

	fp32 Error_X_kalman;
	fp32 Error_Y_kalman;

	fp32 Error_X_Lost;
	fp32 Error_Y_Lost;

	fp32 Yaw_Speed;
	fp32 Pitch_Speed;

	fp32 Yaw_Speed_kalman;
	fp32 Pitch_Speed_kalman;

	fp32 *Yaw_Kalman_result;
	fp32 *Pitch_Kalman_result;

	Statistic Fps;
	uint8_t target;
	bool State;
	bool Goal;
};

#pragma pack(1)
struct game_robot_state_t
{
	uint8_t Header;
	uint8_t Mode;
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	fp32 bullet_speed;
	uint16_t bullet_speed_limit;
	uint8_t CRC8;
};
#pragma pack()

#pragma pack(1)
struct game_status_t
{
    uint8_t Header;
    uint8_t Mode;
		uint8_t game_progress; 	//比赛当前状态
    uint8_t CRC8;
};
#pragma pack()

#pragma pack(1)
struct Chassis_Receive_Data_t
{
	uint8_t Header;
	uint8_t Mode;
	uint8_t mode;
	uint8_t supercap;
	uint8_t CRC8;
};
#pragma pack()

class Message_Ctrl:public Statistic
{
public:
	game_robot_state_t Game_State;
	ext_game_robot_HP_t Game_HP;
	game_status_t game_status;
	Visual_Receive_Data_t VisualR;
	Chassis_Receive_Data_t ChassisR;
	gyro_data_t Gyro;

	RC_ctrl_t *RC_Ptr;

	union F Error_X;
	union F Error_Y;
	union F Shoot;

	void Hook();
	void Init();
	void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void ToEulerAngles(fp32 q1, fp32 q2, fp32 q3, fp32 q4);
	void CAN1_Process(CanRxMsg *Rx_Message);
	void CAN2_Process(CanRxMsg *Rx_Message);
	void Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl);
	void Visual_Serial_Hook(uint8_t *Rx_Message);
	void Gyro_Serial_Hook(uint8_t *Rx_Message);
	void Chassis_Serial_Hook(uint8_t *Rx_Message);
	void Gyro_CAN_Hook(uint32_t *Rx_Message ,uint8_t *Rx_Data);
	float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
private:
	speed_calc_data_t Visual_Yaw_Speed;
	speed_calc_data_t Visual_Pitch_Speed;

	fp32 bullet_speed_last;


};

Message_Ctrl *get_message_ctrl_pointer(void);
RC_ctrl_t *get_remote_control_point(void);

typedef struct
{
	uint8_t	key_flag;
	uint8_t  count;//次数
	uint8_t  last_count;
}count_num_key;

typedef enum
{
	single = 0,
	even,
	count,
}key_count_e;

struct rc_key_v_t
{
	//键盘
	count_num_key W;
	count_num_key S;
	count_num_key A;
	count_num_key D;
	count_num_key shift;
	count_num_key ctrl;
	count_num_key Q;
	count_num_key E;
	count_num_key R;
	count_num_key F;
	count_num_key G;
	count_num_key Z;
	count_num_key X;
	count_num_key C;
	count_num_key V;
	count_num_key B;
};

struct rc_press_t
{
	//鼠标
	count_num_key L;
	count_num_key R;
};

class rc_key_c
{
public:
	rc_key_v_t Key;
	rc_press_t Press;

	void rc_key_v_set(RC_ctrl_t *RC);
	uint8_t read_key(count_num_key *temp_count, key_count_e mode, bool clear);
	bool read_key(count_num_key *temp_count, key_count_e mode, bool *temp_bool);
	void clear_key_count(count_num_key *temp_count);
private:
	bool read_key_single(count_num_key *temp_count);
	bool read_key_single(count_num_key *temp_count, bool *temp_bool);
	bool read_key_even(count_num_key *temp_count);
	bool read_key_even(count_num_key *temp_count, bool *temp_bool);
	void sum_key_count(int16_t key_num, count_num_key *temp_count);
};

#endif
