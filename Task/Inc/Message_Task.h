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
#include "MWMotor.h"
#include "CANDrive.h" 

#ifdef __cplusplus
extern "C" {
#endif

	void Message_Task(void *pvParameters);
	void CAN1_Rx_Task(void *pvParameters);
	void CAN2_Rx_Task(void *pvParameters);
	void CAN3_Rx_Task(void *pvParameters);
	void Serial_Rx_Task(void *pvParameters);
	void Referee_Rx_Task(void *pvParameters);
	void DR16_Rx_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

extern QueueHandle_t Message_Queue;   		//消息队列句柄
extern QueueHandle_t Message_Queue;
extern QueueHandle_t CAN1_Rx_Queue;
extern QueueHandle_t CAN2_Rx_Queue;
extern QueueHandle_t CAN3_Rx_Queue;
extern QueueHandle_t Serial_Rx_Queue;
extern QueueHandle_t Referee_Rx_Queue;
extern QueueHandle_t DR16_Rx_Queue;

#define CAP_CHECK 0x00//检查状态
#define CAP_CLOSE 0x0f//关闭状态
#define CAP_OPEN  0xf0//开启状态
#define CAP_ERROR 0xff//错误状态
#define CAP_MODE1 0x00//关闭补偿
#define CAP_MODE2 0xff//开启补偿


#define UIN16_MPU_RAD        182
#define FP32_MPU_RAD         0.005494505494505494//0.0000958772770853307766059
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_ACCEL_6G_SEN  0.00179443359375f
#define IMU_ID 48

static const uint8_t CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20,
    0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1,
    0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e,
    0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39,
    0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45,
    0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
    0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
    0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0,
    0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
    0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
    0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54,
    0xd7, 0x89, 0x6b, 0x35};


union F
{
	uint8_t I[2];
	float F;
};

struct DM_receive_data_t
{
	fp32 DM_Picth;
	fp32 DM_Roll;
	fp32 DM_Yaw;
	fp32 DM_SpeedZ;
	fp32 DM_SpeedX;
	fp32 DM_SpeedY;
	
	fp32 Acce_Z;
	fp32 Acce_Y;
	
	fp32 DM_Error_angle;
	fp32 DM_Yaw_cycle;
	fp32 Last_YAW_angle;
	fp32 DM_Yaw_angle;
	fp32 DM_Yaw_speed;
	
};

//union uint32_t_uint8_t
//{
//	uint8_t uint_8[4];
//	uint32_t uint_32;
//	
//	
//}


union uint8_t_uint32_t
{
	uint8_t uint_8[4];
	uint32_t uint_32;
};

union int16_t_uint8_t
{
    uint8_t uint_8[2];
    int16_t int_16;
};

struct MPU_Data_tZ
{
			uint8_t HHH;
			uint8_t KEY;
	    int16_t_uint8_t AngleZ;
	    int16_t_uint8_t Speed_Z;
      int16_t_uint8_t Acce_Z;
      int16_t_uint8_t Acce_Y;
};

struct MPU_Data_tXY
{
			uint8_t HHH;
			uint8_t KEY;
			int16_t_uint8_t AngleX;
			int16_t_uint8_t Speed_X;
			int16_t_uint8_t AngleY;
			int16_t_uint8_t Speed_Y;
};

struct supercap_Receive_Data_t
{
    uint8_t situation;  //0x00自检 0x0f关闭 0xf0开启 0xff错误
    uint8_t mode;       //0x00模式1 0xff模式2
    float power;
    uint8_t energy;
    uint8_t power_limit;
    uint8_t errorcode;
    uint8_t *ptr;
};


/*视觉板接收数据结构体*/
struct Visual_Receive_Data_t
{
	  uint8_t Len;
		bool    fire;
	  union F yaw;
	  union F pitch;
	  union F distance;
		uint8_t check_byte;
	  
};

union Radar_Serial_Receive_Data_t
{
	uint8_t I[2];
	int16_t I_16;
};


/*XRobot陀螺仪接收数据结构体*/
typedef struct 
{
    uint8_t id;
	
	 uint8_t_uint32_t time;
	F quat_q0;
	F quat_q1;
	F quat_q2;
	F quat_q3;
	//
	F gyro_X;
	F gyro_Y;
	F gyro_Z;
	//
	F accl_X;
	F accl_Y;
	F accl_Z;
	//
	F eulr_yaw;
	F eulr_pit;
	F eulr_rol;
	
	uint8_t crc8;
}IMU_Receive_Data_t;



//陀螺仪X，Y，Z   加速度计X，Y，Z
typedef struct __attribute__((packed)) {
    float x;
    float y;
    float z;
  } Vector3;
  
//四元数
  typedef struct __attribute__((packed)) {
    float q0;
    float q1;
    float q2;
    float q3;
  } Quaternion;
  
//欧拉角
  typedef struct __attribute__((packed)) {
    float yaw;
    float pit;
    float rol;
  } EulerAngles;


typedef struct __attribute__((packed)) {
    uint8_t prefix; 				   //头帧 0xA5
    uint8_t id;     				  //IMU ID 0x30
    uint32_t time;  				 //Timestamp 
    Quaternion quat_;       //四元数
    Vector3 gyro_;			   //陀螺仪
    Vector3 accl_;        //加速度计
    EulerAngles eulr_;   //欧拉角
    uint8_t crc8; 			//CRC-8 checksum
  } IMU_Data_t;

typedef struct __attribute__((packed)) {
  uint32_t id;
  uint16_t data[4];
}can_pack_t;

	
	



class Message_Ctrl:public Statistic
{
public: 

	supercap_Receive_Data_t    SuperCapR;
  Visual_Receive_Data_t      VisualR;


	const judge_type_t *robo;
	MPU_Data_tZ  MPU_DataZ;
	MPU_Data_tXY MPU_DataXY;

  IMU_Data_t XRobot_Data;
  IMU_Receive_Data_t XRobot_Receive_Data;

	can_pack_t can_pack;
  
  


	DM_receive_data_t DM_data;

	Radar_Serial_Receive_Data_t X;
  Radar_Serial_Receive_Data_t Y;

	RC_ctrl_t *RC_Ptr;

	void Hook();
	void Init();

	void CAN1_Process(CanRxMsg *Rx_Message);
	void CAN2_Process(CanRxMsg *Rx_Message);
	void CAN3_Process(CanRxMsg *Rx_Message);
	void Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl);
	void Visual_Serial_Hook(uint8_t *Rx_Message);
	void Gyro_Serial_Hook(uint8_t *Rx_Message);
  void Radar_Serial_Hook(uint8_t *Rx_Message);
	
	
private:

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
	bool read_key_single(count_num_key *temp_count);
	bool read_key_single(count_num_key *temp_count, bool *temp_bool);
	bool read_key_even(count_num_key *temp_count);
	bool read_key_even(count_num_key *temp_count, bool *temp_bool);
	void sum_key_count(int16_t key_num, count_num_key *temp_count);
private:

};



uint8_t CalculateCRC8(const uint8_t *buf, size_t len, uint8_t crc);
bool VerifyData(const uint8_t *buf, size_t len);



#endif
