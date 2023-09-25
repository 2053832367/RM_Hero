#include "dev_serial.h"
#include "app_preference.h"
#include "protocol_judgement.h"
#include "protocol_crc.h"
#include <stdio.h>
#include "string.h"

//裁判系统串口

//static tFrame			                  tframe;
static tMsg_head                    judgedatahead;
//static ext_client_custom_graphic_single_t  ext_client_custom_graphic_single;
//static draw_data_struct_t		draw_data_struct;
static judge_type_t                   judge_type;


//裁判系统相关
void referee_data_solve(uint8_t *Rx_Message)
{
	static uint16_t start_pos = 0, next_start_pos = 0;
	memcpy(&judgedatahead.SOF, &Rx_Message[start_pos], FrameHeader_Len);
	/*先校验头帧0xA5 然后crc8校验帧头 再crc16位校验整包*/
	if((judgedatahead.SOF == (uint16_t)JudgeFrameHeader) \
		&& (1 == Verify_CRC8_Check_Sum(&Rx_Message[start_pos], FrameHeader_Len)) \
		&& (1 == Verify_CRC16_Check_Sum(&Rx_Message[start_pos], judgedatahead.DataLength + FrameHeader_Len + 4)))//数据位长度+帧头长度+命令码长度+校验码长度
	{
		memcpy(&judge_type.rxCmdId, (&Rx_Message[start_pos] + 5), sizeof(judge_type.rxCmdId));
		Rx_Message[start_pos]++;//每处理完一次就在帧头加一防止再次处理这帧数据
		next_start_pos = start_pos + 9 + judgedatahead.DataLength;//9为 5位帧头 2位数据长度 2校验位
		switch(judge_type.rxCmdId)
		{
		case CmdID_1:
		{
			memcpy(&judge_type.game_status, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
			break;
		}

		case CmdID_3:
		{
			memcpy(&judge_type.game_robot_HP, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
			break;
		}

		case CmdID_8://机器人状态数据，10Hz发送；
		{
			memcpy(&judge_type.game_robot_state, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);//把数组中的数据复制到对应的结构体中去
			break;
		}

		case CmdID_9://实时功率热量数据，50Hz周期发送；
		{
			memcpy(&judge_type.power_heat_data, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
			break;
		}

		case CmdID_14://实时射击数据，弹丸发射后发送；
		{
			memcpy(&judge_type.shoot_data, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
			break;
		}

		case CmdID_10://读取机器人位置信息
		{
			memcpy(&judge_type.game_robot_pos, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);
			break;
		}
		case CmdID_16:
		{
			memcpy(&judge_type.userinfo, (&Rx_Message[start_pos] + 7), judgedatahead.DataLength);//把数组中的数据复制到对应的结构体中去		
			break;
		}
		default:{
			break;
		}
		}
		start_pos = next_start_pos;
	}
	else
	{
		start_pos = 0;
	}
}

const judge_type_t *get_robo_data_Point(void)
{
	return &judge_type;
}

void Usart_SendBuff(uint8_t *buf, uint16_t len)
{
	if(len > 512)
	{
		return;
	}
	JUDGE_SERIAL.sendData(buf, len);
}

