#include "UIDraw_Task.h"
#include "tasks.h"
#include "arm_math.h"

#define AIM_X  956
#define AIM_Y  395


#define MIAO_ZHUN_START_X   
#define MIAO_ZHUN_END_X    


uint16_t x;


void rotate(uint16_t *x, uint16_t *y, float mid_x, float mid_y, float angle);
void rotate_arc(ui_config_t *config, float mid_x, float mid_y, float angle);

void UIDraw_Task(void *pvParameters)
{
/* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {		
		client_info_update();
		Ui_Info_Update();
		Ui_Send();
		xQueueSend(Message_Queue, &ID_Data[UIdrawData], 0);
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */  
}
ui_info_t dynamic_ui_info [DYNAMIC_UI_NUM] = 
{
    [RP_1] = {
			/*******不变配置*********/
		.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
		.ui_config.operate_type = MODIFY,
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE_ui,            // 颜色
    .ui_config.size = 35,                // 字体大小
    .ui_config.width = 4,                // 线条宽度
    .ui_config.start_x = 133,              // 起点 x 坐标
    .ui_config.start_y = 870,              // 起点 y 坐标
    .ui_config.text = "TOP",            // 显示的文字					
  },
    [RP_2] = {
  	/*******不变配置*********/
		.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
		.ui_config.operate_type = MODIFY,
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE_ui,            // 颜色
    .ui_config.size = 35,                // 字体大小
    .ui_config.width = 4,                // 线条宽度
    .ui_config.start_x = 133,              // 起点 x 坐标
    .ui_config.start_y = 790,              // 起点 y 坐标
    .ui_config.text = "CAP",            // 显示的文字	                
  },
    [RP_3] = {
  	/*******不变配置*********/
		.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
		.ui_config.operate_type = MODIFY,
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE_ui,            // 颜色
    .ui_config.size = 35,                // 字体大小
    .ui_config.width = 4,                // 线条宽度
    .ui_config.start_x = 133,              // 起点 x 坐标
    .ui_config.start_y = 710,              // 起点 y 坐标
    .ui_config.text = "VISIDN",            // 显示的文字	 	
  },
    [RP_4] = {
  	/*******不变配置*********/
		.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
		.ui_config.operate_type = MODIFY,
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = WHITE_ui,            // 颜色
    .ui_config.size = 35,                // 字体大小
    .ui_config.width = 4,                // 线条宽度
    .ui_config.start_x = 133,              // 起点 x 坐标
    .ui_config.start_y = 630,              // 起点 y 坐标
    .ui_config.text = "FOLLOW_YAW",            // 显示的文字	 	
  },
    [RP_5] = {
    /*不变配置*/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = FLOAT, // UI内容类型
    /*可变配置*/
    .ui_config.operate_type = MODIFY, // 操作类型
    .ui_config.layer = 1, // 图层数，0~9
    .ui_config.color = PINK_ui, // 颜色
    .ui_config.size = 20, // 字体大小
    .ui_config.width = 2, // 线条宽度
    .ui_config.start_x = 880, // 起点 x 坐标
    .ui_config.start_y = 850, // 起点 y 坐标
    .ui_config.float_num = 0, // 显示的数字
    .ui_config.decimal = 3, // 小数位有效个数
  },
    [RP_6] = {
    /*不变配置*/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = FLOAT, // UI内容类型
    /*可变配置*/
    .ui_config.operate_type = MODIFY, // 操作类型
    .ui_config.layer = 0, // 图层数，0~9
    .ui_config.color = CYAN_BLUE_ui, // 颜色
    .ui_config.size = 20, // 字体大小
    .ui_config.width = 2, // 线条宽度
    .ui_config.start_x = 1020, // 起点 x 坐标
    .ui_config.start_y = 850, // 起点 y 坐标
    .ui_config.float_num = 0, // 显示的数字
    .ui_config.decimal = 2, // 小数位有效个数
  },
    [RP_7] = {
    /*不变配置*/
  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = INT, // UI内容类型
  /*可变配置*/
  .ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 5, // 图层数，0~9
  .ui_config.color = GREEN_ui, // 颜色
  .ui_config.size = 35, // 字体大小
  .ui_config.width = 4, // 线条宽度
  .ui_config.start_x = 1005, //130, // 起点 x 坐标
  .ui_config.start_y = 213, //213, // 起点 y 坐标
  .ui_config.int_num = 0, // 显示的数字
  },
    [RP_8] = {
    /*不变配置*/
  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = CHAR, // UI内容类型
  /*可变配置*/
  .ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 0, // 图层数，0~9
  .ui_config.color = WHITE_ui, // 颜色
  .ui_config.size = 35, // 字体大小
  .ui_config.width = 4, // 线条宽度
  .ui_config.start_x = 1454, // 起点 x 坐标
  .ui_config.start_y = 427, // 起点 y 坐标
  .ui_config.text = "RFID", // 显示的文字
  },
		[RP_9] = {
    /*不变配置*/
  .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = INT, // UI内容类型
  /*可变配置*/
  .ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 5, // 图层数，0~9
  .ui_config.color = CYAN_BLUE_ui, // 颜色
  .ui_config.size = 15, // 字体大小
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x = 234, // 起点 x 坐标
  .ui_config.start_y = 810, // 起点 y 坐标
  .ui_config.int_num = 0, // 显示的数字
  },
		[VISION_CYCLE_1] = {
/*不变配置*/
  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = CHAR, // UI内容类型
  /*可变配置*/
  .ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 0, // 图层数，0~9
  .ui_config.color = WHITE_ui, // 颜色
  .ui_config.size = 35, // 字体大小
  .ui_config.width = 4, // 线条宽度
  .ui_config.start_x = 133, // 起点 x 坐标
  .ui_config.start_y = 560, // 起点 y 坐标
  .ui_config.text = "FRIC", // 显示的文字				
  },
  [VISION_CYCLE_2] = {
    /*不变配置*/
  .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = LINE, // UI内容类型
  /*可变配置*/
  .ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 1, // 图层数，0~9
  .ui_config.color = GREEN_ui, // 颜色
  .ui_config.width = 25, // 线条宽度
  .ui_config.start_x = 243, // 起点 x 坐标
  .ui_config.start_y = 771, // 起点 y 坐标
  .ui_config.end_x = 513, // 终点 x 坐标
  .ui_config.end_y = 771, // 终点 y 坐标
  },
  [TOP_MODE_CYCLE_1] = {
 /*不变配置*/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = FLOAT, // UI内容类型
    /*可变配置*/
    .ui_config.operate_type = MODIFY, // 操作类型
    .ui_config.layer = 0, // 图层数，0~9
    .ui_config.color = CYAN_BLUE_ui, // 颜色
    .ui_config.size = 20, // 字体大小
    .ui_config.width = 2, // 线条宽度
    .ui_config.start_x = 1520, // 起点 x 坐标
    .ui_config.start_y = 850, // 起点 y 坐标
    .ui_config.float_num = 0, // 显示的数字
    .ui_config.decimal = 2, // 小数位有效个数
  },
  [TOP_MODE_CYCLE_2] = {
		/*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CIRCLE,         // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = WHITE_ui,         // 颜色
    .ui_config.width = 10,               // 线条宽度
    .ui_config.start_x = 100, 					 // 圆心 x 坐标
    .ui_config.start_y = 690, 					 // 圆心 y 坐标
    .ui_config.radius = 20,              // 半径 	
  },
  
};

ui_info_t const_ui_info [17] = 
{
//  [0] = {
// /*******不变配置*********/
//    .ui_config.ui_type = CHAR,           // UI内容类型
//    /*******可变配置*********/
//    .ui_config.layer = 0,                // 图层数，0~9
//    .ui_config.color = RED_BLUE_ui,            // 颜色
//    .ui_config.size = 		35,                // 字体大小
//    .ui_config.width = 		4,                // 线条宽度
//    .ui_config.start_x = 	0 ,              // 起点 x 坐标
//    .ui_config.start_y = 	790,              // 起点 y 坐标
//    .ui_config.text = "  o",            // 显示的文字
//  },
//	  [1] = {
//    /*******不变配置*********/
//    .ui_config.ui_type = CHAR,           // UI内容类型
//    /*******可变配置*********/
//    .ui_config.layer = 0,                // 图层数，0~9
//    .ui_config.color = RED_BLUE_ui,            // 颜色
//    .ui_config.size = 		35,                // 字体大小
//    .ui_config.width = 		4,                // 线条宽度
//    .ui_config.start_x = 	0 ,              // 起点 x 坐标
//    .ui_config.start_y = 	790,              // 起点 y 坐标
//    .ui_config.text = "  o",            // 显示的文字
//  },
//		  [2] = {
//    /*******不变配置*********/
//    .ui_config.ui_type = CHAR,           // UI内容类型
//    /*******可变配置*********/
//    .ui_config.layer = 0,                // 图层数，0~9
//    .ui_config.color = RED_BLUE_ui,            // 颜色
//    .ui_config.size = 35,                // 字体大小
//    .ui_config.width = 4,                // 线条宽度
//    .ui_config.start_x = 137,              // 起点 x 坐标
//    .ui_config.start_y = 710,              // 起点 y 坐标
//    .ui_config.text = "VISIDN",            // 显示的文字
//  },
//	[3] = {
//    /*******不变配置*********/
//    .ui_config.ui_type = CHAR,           // UI内容类型
//    /*******可变配置*********/
//    .ui_config.layer = 0,                // 图层数，0~9
//    .ui_config.color = RED_BLUE_ui,            // 颜色
//    .ui_config.size = 35,                // 字体大小
//    .ui_config.width = 4,                // 线条宽度
//    .ui_config.start_x = 139,              // 起点 x 坐标
//    .ui_config.start_y = 630,              // 起点 y 坐标
//    .ui_config.text = "MODE",            // 显示的文字
//  },
	[4] = {
    /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = RED_BLUE_ui,            // 颜色
    .ui_config.size = 35,                // 字体大小
    .ui_config.width = 4,                // 线条宽度
    .ui_config.start_x = 768,              // 起点 x 坐标
    .ui_config.start_y = 211,              // 起点 y 坐标
    .ui_config.text = "LEVEL",            // 显示的文字
  },
	[5] = {
    /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = GREEN_ui,            // 颜色
    .ui_config.size = 20,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = 900,              // 起点 x 坐标
    .ui_config.start_y = 900,              // 起点 y 坐标
    .ui_config.text = "P",            // 显示的文字
  },
	[6] = {
    /*******不变配置*********/
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = GREEN_ui,            // 颜色
    .ui_config.size = 20,                // 字体大小
    .ui_config.width = 2,                // 线条宽度
    .ui_config.start_x = 1000,              // 起点 x 坐标
    .ui_config.start_y = 900,              // 起点 y 坐标
    .ui_config.text = "Y",            // 显示的文字
  },
  [7] = {
  /*不变配置*/
  //.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = ARC, // UI内容类型
  /*可变配置*/
  //.ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 8, // 图层数，0~9
  .ui_config.color = CYAN_BLUE_ui, // 颜色
  .ui_config.width = 1, // 线条宽度
  .ui_config.start_x = 962, // 圆心 x 坐标
  .ui_config.start_y = 539, // 圆心 y 坐标
  .ui_config.end_x = 392, // x 半轴长度
  .ui_config.end_y = 392, // y 半轴长度
  .ui_config.start_angel = 47, // 起始角度
  .ui_config.end_angel = 133, // 终止角度
  },
	[8] = {
  /*不变配置*/
  //.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = ARC, // UI内容类型
  /*可变配置*/
  //.ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 8, // 图层数，0~9
  .ui_config.color = CYAN_BLUE_ui, // 颜色
  .ui_config.width = 1, // 线条宽度
  .ui_config.start_x = 958, // 圆心 x 坐标
  .ui_config.start_y = 539, // 圆心 y 坐标
  .ui_config.end_x = 392, // x 半轴长度
  .ui_config.end_y = 392, // y 半轴长度
  .ui_config.start_angel = 227, // 起始角度
  .ui_config.end_angel = 313, // 终止角度
  },
	[9] = {
		  /*不变配置*/
  .ui_config.ui_type = RECTANGEL, // UI内容类型
  /*可变配置*/
  .ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 5, // 图层数，0~9
  .ui_config.color = RED_BLUE_ui, // 颜色
  .ui_config.width = 3, // 线条宽度
  .ui_config.start_x = 243, // 左上角 x 坐标
  .ui_config.start_y = 786, // 左上角 y 坐标
  .ui_config.end_x =   491, // 右下角 x 坐标
  .ui_config.end_y =   756, // 右下角 y 坐标
  },
  [10] = {
  /*不变配置*/
  //.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = LINE, // UI内容类型
  /*可变配置*/
  //.ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 4, // 图层数，0~9
  .ui_config.color = RED_BLUE_ui, // 颜色
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x =961, // 起点 x 坐标
  .ui_config.start_y = 0, // 起点 y 坐标
  .ui_config.end_x = 961, // 终点 x 坐标
  .ui_config.end_y = 539, // 终点 y 坐标
  },
  [11] = {
  /*不变配置*/
  //.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = LINE, // UI内容类型
  /*可变配置*/
  //.ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 4, // 图层数，0~9
  .ui_config.color = GREEN_ui, // 颜色
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x = 759, // 起点 x 坐标
  .ui_config.start_y = 539, // 起点 y 坐标
  .ui_config.end_x =   1164, // 终点 x 坐标
  .ui_config.end_y =   539, // 终点 y 坐标
  },
  [12] = {
  /*不变配置*/
  //.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = LINE, // UI内容类型
  /*可变配置*/
  //.ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 4, // 图层数，0~9
  .ui_config.color = FUCHSIA_ui, // 颜色
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x = 809, // 起点 x 坐标
  .ui_config.start_y = 494, // 起点 y 坐标
  .ui_config.end_x =   1111, // 终点 x 坐标
  .ui_config.end_y =   494, // 终点 y 坐标
  },  
  [13] = {
  /*不变配置*/
  //.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = LINE, // UI内容类型
  /*可变配置*/
  //.ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 4, // 图层数，0~9
  .ui_config.color = YELLOW_ui, // 颜色
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x = 858, // 起点 x 坐标
  .ui_config.start_y = 449, // 起点 y 坐标
  .ui_config.end_x =   1062, // 终点 x 坐标
  .ui_config.end_y =   449, // 终点 y 坐标
  },  
  [14] = {
  /*不变配置*/
  //.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = LINE, // UI内容类型
  /*可变配置*/
  //.ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 4, // 图层数，0~9
  .ui_config.color = CYAN_BLUE_ui, // 颜色
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x = 888, // 起点 x 坐标
  .ui_config.start_y = 409, // 起点 y 坐标
  .ui_config.end_x =   1037, // 终点 x 坐标
  .ui_config.end_y =   409, // 终点 y 坐标
  },
	[15] = {
  /*不变配置*/
  //.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = LINE, // UI内容类型
  /*可变配置*/
  //.ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 4, // 图层数，0~9
  .ui_config.color = RED_BLUE_ui, // 颜色
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x = 888, // 起点 x 坐标
  .ui_config.start_y = 409, // 起点 y 坐标
  .ui_config.end_x =   888, // 终点 x 坐标
  .ui_config.end_y =   539, // 终点 y 坐标
  },
  [16] = {
  /*不变配置*/
  //.ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
  .ui_config.ui_type = LINE, // UI内容类型
  /*可变配置*/
  //.ui_config.operate_type = MODIFY, // 操作类型
  .ui_config.layer = 4, // 图层数，0~9
  .ui_config.color = RED_BLUE_ui, // 颜色
  .ui_config.width = 2, // 线条宽度
  .ui_config.start_x = 1037, // 起点 x 坐标
  .ui_config.start_y = 409, // 起点 y 坐标
  .ui_config.end_x =   1037, // 终点 x 坐标
  .ui_config.end_y =   539, // 终点 y 坐标
  },


	

};


/**
  @brief: 初始化UI链表 main.c中调用
 */
void My_Ui_Init(void)
{
  Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info)/sizeof(ui_info_t),const_ui_info, sizeof(const_ui_info)/sizeof(ui_info_t));
}


extern uint8_t send_test;

void Ui_Info_Update(void)
{
	
  if (send_test != 0)
  {
    return;
  }
	
//	dynamic_ui_info[TOP_MODE_CYCLE_2].ui_config.float_num = Gimbal.Data.VisualR_Goal;
//	Enqueue_Ui_For_Sending(&dynamic_ui_info[TOP_MODE_CYCLE_2]);
	
	
	dynamic_ui_info[TOP_MODE_CYCLE_1].ui_config.float_num = Message.VisualR.distance.F;
	Enqueue_Ui_For_Sending(&dynamic_ui_info[TOP_MODE_CYCLE_1]);
	
	dynamic_ui_info[RP_5].ui_config.float_num = Gimbal.DM_Pitch.angle;
	Enqueue_Ui_For_Sending(&dynamic_ui_info[RP_5]);
	
	dynamic_ui_info[RP_6].ui_config.float_num = Gimbal.Yaw.angle;
	Enqueue_Ui_For_Sending(&dynamic_ui_info[RP_6]);

	dynamic_ui_info[RP_9].ui_config.int_num = Message.SuperCapR.energy;
  Enqueue_Ui_For_Sending(&dynamic_ui_info[RP_9]);	
	
	dynamic_ui_info[VISION_CYCLE_2].ui_config.end_x = 492*Message.SuperCapR.energy/100.0f;
  Enqueue_Ui_For_Sending(&dynamic_ui_info[VISION_CYCLE_2]);
	
		static uint8_t last_Mode = 0;
  if (Chassis.Mode != last_Mode)
  {
    if (Chassis.Mode == CHASSIS_LITTLE_TOP)
    {
      dynamic_ui_info[RP_1].ui_config.color = GREEN_ui;
    
    }
    else
    {
      dynamic_ui_info[RP_1].ui_config.color = WHITE_ui;
   
    }
    Enqueue_Ui_For_Sending(&dynamic_ui_info[RP_1]);
    
  }
	last_Mode = Chassis.Mode;
	
	static uint8_t last_level = 0;
	if (judge_type.game_robot_state.robot_level != last_level)
  {
		dynamic_ui_info[RP_7].ui_config.int_num = judge_type.game_robot_state.robot_level;
		Enqueue_Ui_For_Sending(&dynamic_ui_info[RP_7]);
 
	}
    last_level = judge_type.game_robot_state.robot_level;
	
	
	static uint8_t last_cap_state = 0;
  if (Chassis.Power_Ctrl.cap_state != last_cap_state)
  {
    if (Chassis.Power_Ctrl.cap_state == 1)
    {
      dynamic_ui_info[RP_2].ui_config.color = GREEN_ui;
    
    }
    else
    {
      dynamic_ui_info[RP_2].ui_config.color = WHITE_ui;
   
    }
    Enqueue_Ui_For_Sending(&dynamic_ui_info[RP_2]);
    
  }
	last_cap_state = Chassis.Power_Ctrl.cap_state;
	
	
	static uint8_t last_Visual_Flag = 0;
  if (Gimbal.Flags.Visual_Flag   != last_Visual_Flag)
  {
    if (Gimbal.Flags.Visual_Flag == true)
    {
      dynamic_ui_info[RP_3].ui_config.color = GREEN_ui; 
    }
    else
    {
      dynamic_ui_info[RP_3].ui_config.color = WHITE_ui;
    }
		
    Enqueue_Ui_For_Sending(&dynamic_ui_info[RP_3]);
    
  }
	last_Visual_Flag = Gimbal.Flags.Visual_Flag  ;
	
	
	static uint8_t last_Chassis_Mode = 0;
  if (Chassis.Mode != last_Chassis_Mode)
  {
    if (Chassis.Mode == CHASSIS_FOLLOW_YAW)
    {
      dynamic_ui_info[RP_4].ui_config.color = GREEN_ui;
    
    }
    else
    {
      dynamic_ui_info[RP_4].ui_config.color = WHITE_ui;
   
    }
    Enqueue_Ui_For_Sending(&dynamic_ui_info[RP_4]);
  
  }
	last_Chassis_Mode = Chassis.Mode;
	
	
	static uint8_t last_rfid_status = 0;
  if (judge_type.rfid_state.rfid_status != last_rfid_status)
  {
    if (judge_type.rfid_state.rfid_status != 0)
    {
      dynamic_ui_info[RP_5].ui_config.color = GREEN_ui;
    
    }
    else
    {
      dynamic_ui_info[RP_5].ui_config.color = WHITE_ui;
   
    }
    Enqueue_Ui_For_Sending(&dynamic_ui_info[RP_3]);
  
  }
	last_rfid_status = judge_type.rfid_state.rfid_status;
	
	static uint8_t last_Fric = 0;
  if (Gimbal.Flags.Fric_Flag != last_Fric)
  {
    if (Gimbal.Flags.Fric_Flag == true)
    {
      dynamic_ui_info[VISION_CYCLE_1].ui_config.color = GREEN_ui;  
    }
    else
    {
      dynamic_ui_info[VISION_CYCLE_1].ui_config.color = WHITE_ui;  
    }
    Enqueue_Ui_For_Sending(&dynamic_ui_info[VISION_CYCLE_1]); 
  }
	last_Fric = Gimbal.Flags.Fric_Flag;
	
	
	
	
	
	
//	static uint8_t last_Visual_true = 0;
//  if (Gimbal.Flags.Visual_true_Flag != last_Visual_true)
//  {
//    if (Gimbal.Flags.Visual_true_Flag == true)
//    {
//      dynamic_ui_info[TOP_MODE_CYCLE_2].ui_config.color = GREEN_ui;
//    
//    }
//    else
//    {
//      dynamic_ui_info[TOP_MODE_CYCLE_2].ui_config.color = WHITE_ui;
//   
//    }
//    Enqueue_Ui_For_Sending(&dynamic_ui_info[TOP_MODE_CYCLE_2]);
//  }
//  last_Visual_true = Gimbal.Flags.Visual_true_Flag;





}

