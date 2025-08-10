#ifndef __APP_PREFERENCE_H
#define __APP_PREFERENCE_H

//忽略警告，摆烂
#pragma diag_suppress 177
#pragma diag_suppress 550
#pragma diag_suppress 3337
// #pragma diag_suppress 1299

//轮组选择 useMecanum useSteering useBalance
#define useMecanum
//兵种选择 useHero useInfantry
#define useHero  

#define g_HENGYANG 9.7907f//衡阳
#define g_WUHAN    9.7936f//武汉
#define g_CHANGSHA 9.7915f//长沙

//发送 格式 Serialx_Ctrl （x用3，6，7，8代替），串口不能重复！
//SerialDatax 用于判断外设是否在线
#define VISUAL_SERIAL      Serial3_Ctrl
#define VisualData         SerialData1
#define VISUAL_SERIAL_BAUD 115200

#define JUDGE_SERIAL      Serial7_Ctrl
#define JudgeData         SerialData7
#define JUDGE_SERIAL_BAUD 115200
#define JUDGERX_BUF_NUM   255u
#define JUDGETX_BUF_NUM   255u

#define GYRO_SERIAL 			Serial8_Ctrl
#define GYRO_SERIAL_BAUD	1000000//115200
#define GYRO_BUF_NUM      16//118u//16


#define DR16_SERIAL 		       Serial4_Ctrl
#define DbusData				 			 SerialData4
#define DR16_SERIAL_Data_Lenth 18
#define DR16_SERIAL_BAUD 			 100000

#define VISUAL_SERIAL_HEADER 0xff
#define VISUAL_SERIAL_TAIL   0xfe

#define CHASSIS_SERIAL_HEADER 0xff
#define CHASSIS_SERIAL_TAIL   0xfe

//无参数则为 NULL ，会直接跳过。
//Lenth用于检验数据长度，Buffer_size用于创建环形缓冲区和数据缓冲区，为0则中断直接发送通知给Meesge,Buffer_size应大于Lenth
//Serialx_ITPending 可选 USART_IT_IDLE USART_IT_RXNE USART_IT_RXNE_AND_IDLE

#define Serial_NORMAL_Mode 0
#define Serial_DMA_Mode    1


#define Serial1_Data_Header 0xA5
#define Serial1_Data_Tail   NULL
#define Serial1_Data_Lenth0 NULL
#define Serial1_Data_Lenth1 NULL
#define Serial1_Data_Lenth2 NULL
#define Serial1_Data_Lenth3 NULL
#define Serial1_Buffer_Size 80
#define Serial1_ITPending   USART_IT_RXNE_AND_IDLE
#define Serial1_Mode        Serial_NORMAL_Mode

#define Serial3_Data_Header 0xFF
#define Serial3_Data_Tail 	NULL
#define Serial3_Data_Lenth0 32
#define Serial3_Data_Lenth1 NULL
#define Serial3_Data_Lenth2 NULL
#define Serial3_Data_Lenth3 NULL
#define Serial3_Buffer_Size 64
#define Serial3_ITPending   USART_IT_RXNE_AND_IDLE
#define Serial3_Mode        Serial_NORMAL_Mode

#define Serial4_Data_Header NULL
#define Serial4_Data_Tail   NULL
#define Serial4_Data_Lenth0 DR16_SERIAL_Data_Lenth
#define Serial4_Data_Lenth1 NULL
#define Serial4_Data_Lenth2 NULL
#define Serial4_Data_Lenth3 NULL
#define Serial4_Buffer_Size 38
#define Serial4_ITPending USART_IT_IDLE
#define Serial4_Mode Serial_NORMAL_Mode

#define Serial7_Data_Header 0xA5
#define Serial7_Data_Tail   NULL
#define Serial7_Data_Lenth0 NULL
#define Serial7_Data_Lenth1 NULL
#define Serial7_Data_Lenth2 NULL
#define Serial7_Data_Lenth3 NULL
#define Serial7_Buffer_Size JUDGERX_BUF_NUM
#define Serial7_ITPending   USART_IT_RXNE_AND_IDLE
#define Serial7_Mode        Serial_NORMAL_Mode

#define Serial8_Data_Header 0xAA//0xA5
#define Serial8_Data_Tail 	NULL
#define Serial8_Data_Lenth0 10//59
#define Serial8_Data_Lenth1 20
#define Serial8_Data_Lenth2 30
#define Serial8_Data_Lenth3 40
#define Serial8_Buffer_Size GYRO_BUF_NUM
#define Serial8_ITPending   USART_IT_RXNE_AND_IDLE
#define Serial8_Mode        Serial_NORMAL_Mode

#define FDCAN1_Buffer_Size 50
#define FDCAN2_Buffer_Size 50
#define FDCAN3_Buffer_Size 70

/*选择遥控器控制模式(
三挡位功能如下
0:	不跟随 	跟随 	小陀螺 
1:	不跟随 	视觉 	视觉发弹)*/
#define Gimbal_RC_CONTRAL_MODE       1
#define Chassis_RC_CONTRAL_MODE      0


//拨弹连发(可改写按键)
#define AUTOSHOOT            1
//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN       0.0003f
#define Pitch_RC_SEN     0.00005f //0.0001f

//yaw,pitch角度和鼠标输入的比例
#define Yaw_Mouse_SEN    0.0010f
#define Pitch_Mouse_SEN -0.00018f//0.001f

/*********************------------------------------------------------------云台参数------------------------------------------------**********************/


#define RAD_TO_ANGLE    57.29577951308232088f
#define ANGLE_TO_RAD    0.017453292519943f


//望远镜
#define MIRROR_OPEN_DUTY   37
#define MIRROR_CLOSE_DUTY  25

#define IMAGE_HANGING_SHOT 45
#define IMAGE_NORMAL       41//8米前哨站图传  pitch16度    // 12米前哨战图传 42 


//#define TUCHUAN_NORMAL    260     //2006ECD   0-8191
//#define TUCHUAN_LOB_SHOT  5500    //

//pitch轴ECD设置（注意数据可能过零点或min大于max的情况，会自动处理）
#define GIMBAL_PITCH_OFFSET_ECD 5640
#define GIMBAL_PITCH_MAX_ECD    6400
#define GIMBAL_PITCH_MIN_ECD    5070

#define GIMBAL_PITCH_MAX_ANGLE  +45//+45
#define GIMBAL_PITCH_MIN_ANGLE  -8//-9


#define FRIC_SPEED_SET  5230
#define Starting_Value  10
#define Frame_PerioD    1


//拨弹盘旋转方向 (1,-1)
#define TRIGGER_MOTOR_REVERSE   1.0f
//拨弹轮一圈总共多少发
#define TRIGGER_ONCE_SHOOT_NUM  7.0f
//拨弹轮连发速度（1秒多少发）
#define TRIGGER_ONE_S_SHOOT_NUM1 6.0f
#define TRIGGER_ONE_S_SHOOT_NUM2 8.0f
#define TRIGGER_ONE_S_SHOOT_NUM3 12.0f
//拨弹轮堵转判断电流
#define TRIGGER_BLOCKED_CURRENT 5000

//拨弹盘时间模式下的设定值
#define PLUNK_RPM_SET       3.0f


//低通滤波
#define GIMBAL_YAW_SPEED_NUM 0.007f
#define follow_x_NUM 0.3f
#define follow_y_NUM 0.3f

/*YAW角度环*/
#define Yaw_POSITION_PID_MAX_OUT     10000
#define Yaw_POSITION_PID_MAX_IOUT    3
#define Yaw_POSITION_PID_Deadband	   0.0
#define Yaw_POSITION_PID_KP  			   0.2
#define Yaw_POSITION_PID_KI				   0.0
#define Yaw_POSITION_PID_KD          0 
#define Yaw_POSITION_PID_I_KA			   3
#define Yaw_POSITION_PID_I_KB			   0//2
#define Yaw_POSITION_PID_OUT_Filter  0
#define Yaw_POSITION_PID_D_Filter	   0 
#define Yaw_POSITION_PID_ols_order	 0
#define Yaw_POSITION_PID_IP				   0x23 //0010 0011 
/*YAW 速度环*/
#define Yaw_SPEED_PID_MAX_OUT    		 30000
#define Yaw_SPEED_PID_MAX_IOUT   		 25000
#define Yaw_SPEED_PID_Deadband	 		 0
#define Yaw_SPEED_PID_KP  			 		 20000
#define Yaw_SPEED_PID_KI				 		 0
#define Yaw_SPEED_PID_KD				 		 0 
#define Yaw_SPEED_PID_I_KA			 		 0
#define Yaw_SPEED_PID_I_KB			 		 0
#define Yaw_SPEED_PID_OUT_Filter 		 0
#define Yaw_SPEED_PID_D_Filter	 		 0 
#define Yaw_SPEED_PID_ols_order	 		 0
#define Yaw_SPEED_PID_IP				 		 0x23 //0010 0011


/*YAW角度环*/
#define Yaw_LOB_SHOT_POSITION_PID_MAX_OUT     16333
#define Yaw_LOB_SHOT_POSITION_PID_MAX_IOUT    3
#define Yaw_LOB_SHOT_POSITION_PID_Deadband	   0.0
#define Yaw_LOB_SHOT_POSITION_PID_KP  			   1.0
#define Yaw_LOB_SHOT_POSITION_PID_KI				   0.0
#define Yaw_LOB_SHOT_POSITION_PID_KD          0 
#define Yaw_LOB_SHOT_POSITION_PID_I_KA			   3
#define Yaw_LOB_SHOT_POSITION_PID_I_KB			   0//2
#define Yaw_LOB_SHOT_POSITION_PID_OUT_Filter  0
#define Yaw_LOB_SHOT_POSITION_PID_D_Filter	   0 
#define Yaw_LOB_SHOT_POSITION_PID_ols_order	 0
#define Yaw_LOB_SHOT_POSITION_PID_IP				   0x23 //0010 0011 
/*YAW 速度环*/
#define Yaw_LOB_SHOT_SPEED_PID_MAX_OUT    30000
#define Yaw_LOB_SHOT_SPEED_PID_MAX_IOUT   25000
#define Yaw_LOB_SHOT_SPEED_PID_Deadband	 0
#define Yaw_LOB_SHOT_SPEED_PID_KP  			 15000
#define Yaw_LOB_SHOT_SPEED_PID_KI				 0
#define Yaw_LOB_SHOT_SPEED_PID_KD				 0 
#define Yaw_LOB_SHOT_SPEED_PID_I_KA			 0
#define Yaw_LOB_SHOT_SPEED_PID_I_KB			 0
#define Yaw_LOB_SHOT_SPEED_PID_OUT_Filter 0
#define Yaw_LOB_SHOT_SPEED_PID_D_Filter	 0 
#define Yaw_LOB_SHOT_SPEED_PID_ols_order	 0
#define Yaw_LOB_SHOT_SPEED_PID_IP				 0x23 //0010 0011


/*pitch 位置环*/
#define PITCH_POSITION_PID_MAX_OUT     5
#define PITCH_POSITION_PID_MAX_IOUT    2
#define PITCH_POSITION_PID_Deadband	   0
#define PITCH_POSITION_PID_KP  			   1.8//0.7
#define PITCH_POSITION_PID_KI				   0.65//0.6
#define PITCH_POSITION_PID_KD				   0 
#define PITCH_POSITION_PID_I_KA			   3
#define PITCH_POSITION_PID_I_KB			   2
#define PITCH_POSITION_PID_OUT_Filter  0
#define PITCH_POSITION_PID_D_Filter	   0 
#define PITCH_POSITION_PID_ols_order	 0
#define PITCH_POSITION_PID_IP				   0x27 //0010 0011 
/*pitch 速度环*/
#define PITCH_SPEED_PID_MAX_OUT    5
#define PITCH_SPEED_PID_MAX_IOUT   0 
#define PITCH_SPEED_PID_Deadband	 0
#define PITCH_SPEED_PID_KP  			 0.5//3
#define PITCH_SPEED_PID_KI				 0
#define PITCH_SPEED_PID_KD				 0 
#define PITCH_SPEED_PID_I_KA			 3
#define PITCH_SPEED_PID_I_KB			 2
#define PITCH_SPEED_PID_OUT_Filter 0
#define PITCH_SPEED_PID_D_Filter	 0 
#define PITCH_SPEED_PID_ols_order	 0
#define PITCH_SPEED_PID_IP				 0x23  //0010 0011


/*Trigger 位置环*/
#define TRIGGER_POSITION_PID_MAX_OUT     20.0
#define TRIGGER_POSITION_PID_MAX_IOUT    2.0
#define TRIGGER_POSITION_PID_Deadband	   0.005
#define TRIGGER_POSITION_PID_KP  			   0.8
#define TRIGGER_POSITION_PID_KI				   0.01
#define TRIGGER_POSITION_PID_KD				   0.0
#define TRIGGER_POSITION_PID_I_KA			   3.0
#define TRIGGER_POSITION_PID_I_KB			   2.0
#define TRIGGER_POSITION_PID_OUT_Filter  0.0
#define TRIGGER_POSITION_PID_D_Filter	   0.0
#define TRIGGER_POSITION_PID_ols_order	 0.0
#define TRIGGER_POSITION_PID_IP				   0x23 //0010 0011 
/*Trigger 速度环*/
#define TRIGGER_SPEED_PID_MAX_OUT    20
#define TRIGGER_SPEED_PID_MAX_IOUT   0.0
#define TRIGGER_SPEED_PID_Deadband	 0.0005
#define TRIGGER_SPEED_PID_KP  			 2.0
#define TRIGGER_SPEED_PID_KI				 0.0
#define TRIGGER_SPEED_PID_KD				 0.0 
#define TRIGGER_SPEED_PID_I_KA			 3.0
#define TRIGGER_SPEED_PID_I_KB			 2.0
#define TRIGGER_SPEED_PID_OUT_Filter 0.0
#define TRIGGER_SPEED_PID_D_Filter	 0.0 
#define TRIGGER_SPEED_PID_ols_order	 0.0
#define TRIGGER_SPEED_PID_IP				 0x23  //0010 0011




/*YAW 视觉角度环*/
#define Yaw_POSITION_VisualR_PID_MAX_OUT      10000//10
#define Yaw_POSITION_VisualR_PID_MAX_IOUT     3//3
#define Yaw_POSITION_VisualR_PID_Deadband	    0.0//0.0
#define Yaw_POSITION_VisualR_PID_KP  			    0.2//1.0
#define Yaw_POSITION_VisualR_PID_KI				    0.0//0.8
#define Yaw_POSITION_VisualR_PID_KD           0 //0 
#define Yaw_POSITION_VisualR_PID_I_KA			    3//3
#define Yaw_POSITION_VisualR_PID_I_KB			    0//2//2
#define Yaw_POSITION_VisualR_PID_OUT_Filter   0//0
#define Yaw_POSITION_VisualR_PID_D_Filter	    0 //0 
#define Yaw_POSITION_VisualR_PID_ols_order	  0//0
#define Yaw_POSITION_VisualR_PID_IP				    0x23 //0010 0011//0x23 // 0010 0011 
/*YAW视觉速度环*/                              //
#define Yaw_SPEED_PID_VisualR_MAX_OUT       	30000//30000
#define Yaw_SPEED_PID_VisualR_MAX_IOUT      	25000//3000 
#define Yaw_SPEED_PID_VisualR_Deadband	     	0//0
#define Yaw_SPEED_PID_VisualR_KP  			 			20000//15000.0
#define Yaw_SPEED_PID_VisualR_KI				 			0//0.0
#define Yaw_SPEED_PID_VisualR_KD				 			0 //0 
#define Yaw_SPEED_PID_VisualR_I_KA			 			0//0
#define Yaw_SPEED_PID_VisualR_I_KB			 			0//0
#define Yaw_SPEED_PID_VisualR_OUT_Filter 			0//0
#define Yaw_SPEED_PID_VisualR_D_Filter	 			0 //0 
#define Yaw_SPEED_PID_VisualR_ols_order	 			0//0
#define Yaw_SPEED_PID_VisualR_IP				 			0x23 //0010 0011//0x23 // 0010 0011


/*pitch视觉 角度环*/
#define PITCH_POSITION_PID_VisualR_MAX_OUT     15
#define PITCH_POSITION_PID_VisualR_MAX_IOUT    2.0
#define PITCH_POSITION_PID_VisualR_Deadband	   0.0
#define PITCH_POSITION_PID_VisualR_KP  			   0.8
#define PITCH_POSITION_PID_VisualR_KI				   0.5
#define PITCH_POSITION_PID_VisualR_KD				   0 
#define PITCH_POSITION_PID_VisualR_I_KA			   3
#define PITCH_POSITION_PID_VisualR_I_KB			   2
#define PITCH_POSITION_PID_VisualR_OUT_Filter  0
#define PITCH_POSITION_PID_VisualR_D_Filter	   0 
#define PITCH_POSITION_PID_VisualR_ols_order	 0
#define PITCH_POSITION_PID_VisualR_IP				   0x23 //0010 0011 
/*pitch视觉 速度环*/
#define PITCH_SPEED_PID_VisualR_MAX_OUT    4.0f
#define PITCH_SPEED_PID_VisualR_MAX_IOUT   2.0f 
#define PITCH_SPEED_PID_VisualR_Deadband	 0
#define PITCH_SPEED_PID_VisualR_KP  			 1.0
#define PITCH_SPEED_PID_VisualR_KI				 0.0
#define PITCH_SPEED_PID_VisualR_KD				 0.0
#define PITCH_SPEED_PID_VisualR_I_KA			 3
#define PITCH_SPEED_PID_VisualR_I_KB			 2
#define PITCH_SPEED_PID_VisualR_OUT_Filter 0
#define PITCH_SPEED_PID_VisualR_D_Filter	 0 
#define PITCH_SPEED_PID_VisualR_ols_order	 0
#define PITCH_SPEED_PID_VisualR_IP				 0x23  //0010 0011



////拨弹轮电机角度环
//#define TRIGGER_ANGLE_PID_KP       0.0f //
//#define TRIGGER_ANGLE_PID_KI       0.0f
//#define TRIGGER_ANGLE_PID_KD       0.0f  //10.0f
//#define TRIGGER_ANGLE_PID_MAX_OUT  50.0f
//#define TRIGGER_ANGLE_PID_MAX_IOUT 10.0f
//#define TRIGGER_ANGLE_PID_BAND_I   5.0f
////拨弹轮电机速度环
//#define TRIGGER_SPEED_PID_KP 				0.0f//
//#define TRIGGER_SPEED_PID_KI 				0.0f//
//#define TRIGGER_SPEED_PID_KD 				0.0f//
//#define TRIGGER_SPEED_PID_MAX_OUT 	25.0f
//#define TRIGGER_SPEED_PID_MAX_IOUT 	5.0f
//#define TRIGGER_SPEED_PID_BAND_I 		5.0f


//摩擦轮电机速度
#define FRIC1_SPEED_PID_KP 	10.0f//10.0f
#define FRIC1_SPEED_PID_KI 	0.0f
#define FRIC1_SPEED_PID_KD 	0.01f
#define FRIC1_PID_MAX_OUT 	15000.0f
#define FRIC1_PID_MAX_IOUT 	8000.0f
#define FRIC1_PID_BAND_I	 	3000.0f

#define FRIC2_SPEED_PID_KP 	10.0f//10.0f
#define FRIC2_SPEED_PID_KI 	0.0f
#define FRIC2_SPEED_PID_KD 	0.01f
#define FRIC2_PID_MAX_OUT		15000.0f
#define FRIC2_PID_MAX_IOUT 	8000.0f
#define FRIC2_PID_BAND_I 		3000.0f

#define FRIC3_SPEED_PID_KP 	10.0f//10.0f
#define FRIC3_SPEED_PID_KI 	0.0f
#define FRIC3_SPEED_PID_KD 	0.0f
#define FRIC3_PID_MAX_OUT 	15000.0f
#define FRIC3_PID_MAX_IOUT 	8000.0f
#define FRIC3_PID_BAND_I	 	3000.0f

#define FRIC6_SPEED_PID_KP 	10.0f//10.0f
#define FRIC6_SPEED_PID_KI 	0.0f
#define FRIC6_SPEED_PID_KD 	0.0f
#define FRIC6_PID_MAX_OUT 	15000.0f
#define FRIC6_PID_MAX_IOUT 	8000.0f
#define FRIC6_PID_BAND_I	 	3000.0f

//#define FRIC6_SPEED_PID_KP 	10.0f//10.0f
//#define FRIC6_SPEED_PID_KI 	0.0f
//#define FRIC6_SPEED_PID_KD 	0.01f
//#define FRIC6_PID_MAX_OUT 	9000.0f
//#define FRIC6_PID_MAX_IOUT 	5000.0f
//#define FRIC6_PID_BAND_I	 	3000.0f



/*图传2006角度环*/
#define TuChuan1_POSITION_PID_MAX_OUT     10000
#define TuChuan1_POSITION_PID_MAX_IOUT    1000
#define TuChuan1_POSITION_PID_Deadband	   0.0
#define TuChuan1_POSITION_PID_KP  			   0.2
#define TuChuan1_POSITION_PID_KI				   0.0
#define TuChuan1_POSITION_PID_KD           0 
#define TuChuan1_POSITION_PID_I_KA			   100
#define TuChuan1_POSITION_PID_I_KB			   100//2
#define TuChuan1_POSITION_PID_OUT_Filter  0
#define TuChuan1_POSITION_PID_D_Filter	   0 
#define TuChuan1_POSITION_PID_ols_order	 0
#define TuChuan1_POSITION_PID_IP				   0x23 //0010 0011 
/*图传2006速度环*/
#define TuChuan1_SPEED_PID_MAX_OUT    10000
#define TuChuan1_SPEED_PID_MAX_IOUT   5000
#define TuChuan1_SPEED_PID_Deadband	 0
#define TuChuan1_SPEED_PID_KP  			 3
#define TuChuan1_SPEED_PID_KI				 0.1
#define TuChuan1_SPEED_PID_KD				 0 
#define TuChuan1_SPEED_PID_I_KA			 100
#define TuChuan1_SPEED_PID_I_KB			 100
#define TuChuan1_SPEED_PID_OUT_Filter 0
#define TuChuan1_SPEED_PID_D_Filter	 0 
#define TuChuan1_SPEED_PID_ols_order	 0
#define TuChuan1_SPEED_PID_IP				 0x23 //0010 0011


/*图传2006角度环*/
#define WuangYuan_POSITION_PID_MAX_OUT     10000
#define WuangYuan_POSITION_PID_MAX_IOUT    1000
#define WuangYuan_POSITION_PID_Deadband	   0.0
#define WuangYuan_POSITION_PID_KP  			   0.2
#define WuangYuan_POSITION_PID_KI				   0.0
#define WuangYuan_POSITION_PID_KD           0 
#define WuangYuan_POSITION_PID_I_KA			   3
#define WuangYuan_POSITION_PID_I_KB			   0.1//2
#define WuangYuan_POSITION_PID_OUT_Filter  0
#define WuangYuan_POSITION_PID_D_Filter	   0 
#define WuangYuan_POSITION_PID_ols_order	 0
#define WuangYuan_POSITION_PID_IP				   0x23 //0010 0011 
/*图传2006速度环*/
#define WuangYuan_SPEED_PID_MAX_OUT    3000
#define WuangYuan_SPEED_PID_MAX_IOUT   5000
#define WuangYuan_SPEED_PID_Deadband	 0.0
#define WuangYuan_SPEED_PID_KP  			 3.0
#define WuangYuan_SPEED_PID_KI				 0.0
#define WuangYuan_SPEED_PID_KD				 0.0
#define WuangYuan_SPEED_PID_I_KA			 3.0
#define WuangYuan_SPEED_PID_I_KB			 0.1
#define WuangYuan_SPEED_PID_OUT_Filter 0
#define WuangYuan_SPEED_PID_D_Filter	 0 
#define WuangYuan_SPEED_PID_ols_order	 0
#define WuangYuan_SPEED_PID_IP				 0x23 //0010 0011





/*图传2006速度环*/
#define WuangYuan_SPEED0_PID_MAX_OUT    2000
#define WuangYuan_SPEED0_PID_MAX_IOUT   1000
#define WuangYuan_SPEED0_PID_Deadband	 0.0
#define WuangYuan_SPEED0_PID_KP  			 4.0
#define WuangYuan_SPEED0_PID_KI				 0.0
#define WuangYuan_SPEED0_PID_KD				 0.0
#define WuangYuan_SPEED0_PID_I_KA			 3.0
#define WuangYuan_SPEED0_PID_I_KB			 0.1
#define WuangYuan_SPEED0_PID_OUT_Filter 0
#define WuangYuan_SPEED0_PID_D_Filter	 0 
#define WuangYuan_SPEED0_PID_ols_order	 0
#define WuangYuan_SPEED0_PID_IP				 0x23 //0010 0011





//底盘参数------------------------------------------------------------------------------------------------------

// 机器人底盘修改的参数,单位为m(米)
#define WHEEL_BASE  0.38              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 0.36           // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 150             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

/* 自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * 1.0f)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * 1.0f)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * 1.0f)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * 1.0f)


#define K_Low_setchassis    0.01

//云台电机相对底盘正方向的ECD
#define Gimbal_Motor_Yaw_Offset_ECD 3172 

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


#define MOTOR_DISTANCE_TO_CENTER 0.27f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 1
//底盘任务控制间隔
#define CHASSIS_CONTROL_TIME (0.001 * CHASSIS_CONTROL_TIME_MS)
//底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE (1000 / CHASSIS_CONTROL_TIME_MS)
//底盘3508最大can发送电流值
#define MAX_MOTOR_3508_CAN_CURRENT 20000.0f
//底盘6020最大can发送电压值
#define MAX_MOTOR_6020_CAN_CURRENT 30000.0f

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机或轮子需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR       0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘电机最大速度
#define MAX_WHEEL_SPEED            10.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 10.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 10.0f
//底盘运动过程最大旋速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 10.0f

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE  0.1f

#define Fric_Set_Frame_Period 1.6

//缓冲能量环PID
#define POWER_BUFFER_PID_KP       -1.5f
#define POWER_BUFFER_PID_KI       0
#define POWER_BUFFER_PID_KD       0.0f
#define POWER_BUFFER_PID_MAX_OUT  20
#define POWER_BUFFER_PID_MAX_IOUT 1.0f
#define POWER_BUFFER_PID_BAND_I   20.0f


//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 				5500.0f
#define M3505_MOTOR_SPEED_PID_KI 				0.0f
#define M3505_MOTOR_SPEED_PID_KD 				60.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 	MAX_MOTOR_3508_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 	8000.0f
#define M3505_MOTOR_SPEED_PID_BAND_I  	3000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP			 	 -20.0f   //-20
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 					-0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 					-100.0f   //-100
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT	 		20000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT		1000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_BAND_I 			3000.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP2			 		0.0016f


#ifndef useMecanum

#define useMecanum
#endif
#ifdef useSteering
#undef useMecanum
#endif
#ifdef useBalance
#undef useSteering
#endif

#ifndef useInfantry
#define useInfantry
#endif
#ifdef useHero
#undef useInfantry
#endif

typedef enum
{
	FaultData = 0x00,
	CanData1,
	CanData2,
	CanData3,
	SerialData1,
	SerialData3,
	SerialData4,
	SerialData7,
	SerialData8,
	RCData,
	RefereeData,
	GyroData,
	MessageData,
	GimbalData,
	ChassisData,
	UIdrawData,
	CorrespondenceData,
	SupercapData,
	ID_e_count
}ID_e;

typedef struct 
{
	ID_e Data_ID;
	void *Data_Ptr;
}ID_Data_t;

extern ID_Data_t ID_Data[ID_e_count];

void Prefence_Init(void);
#endif
