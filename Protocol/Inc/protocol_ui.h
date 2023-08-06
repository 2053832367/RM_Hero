#ifndef __PROTOCOL_UI_H
#define __PROTOCOL_UI_H

#include "stm32f4xx.h"
#include "protocol_ui.h"
#include "protocol_crc.h"
#include "protocol_judgement.h"

#define NULL 0

#define Graphic_Color1    1
#define Graphic_Color2    2      //多色备用
#define Graphic_Color3    3      //多色备用
//颜色：0红蓝主色；1黄色；2绿色；3橙色；4紫红色；5粉色；6青色；7黑色；8白色；

#define Graphic_Color_Main    		  0  //红蓝主色
#define Graphic_Color_Yellow 				1
#define Graphic_Color_Green 				2
#define Graphic_Color_Orange 				3
#define Graphic_Color_Purplish_red  4   //紫红色
#define Graphic_Color_Pink 					5
#define Graphic_Color_Cyan 					6   //青色
#define Graphic_Color_Black 				7
#define Graphic_Color_White 				8

#define Graphic_Width     3

#define R1_Hero 			1
#define R2_Engineer 		2
#define R3_Standard1      3
#define R4_Standard2      4
#define R5_Standard3      5
#define R6_Aerial         6
#define R_Sentry          7
#define R_Radar           9

#define B1_Hero 			101
#define B2_Engineer 		102
#define B3_Standard1      103
#define B4_Standard2      104
#define B5_Standard3      105
#define B6_Aerial         106
#define B_Sentry          107
#define B_Radar           109

#define R1_Hero_Client 				0x101
#define R2_Engineer_Client			0x102
#define R3_Standard1_Client       0x103
#define R4_Standard2_Client       0x104
#define R5_Standard3_Client       0x105
#define R6_Aerial_Client          0x106

#define B1_Hero_Client 			0x165
#define B2_Engineer_Client		0x166
#define B3_Standard1_Client       0x167
#define B4_Standard2_Client       0x168
#define B5_Standard3_Client       0x169
#define B6_Aerial_Client          0x16A

#define UI_Graph_Nop              0 
#define UI_Graph_ADD 							1
#define UI_Graph_Change 					2
#define UI_Graph_Del 							3

#define UI_Data_Del_Nop 					0
#define UI_Data_Del_Layer 				1
#define UI_Data_Del_ALL 					2

#define UI_Graph_Line 						0    //直线
#define UI_Graph_Rectangle 				1    //矩形
#define UI_Graph_Circle 					2    //整圆
#define UI_Graph_Ellipse 					3    //椭圆
#define UI_Graph_Arc 							4    //圆弧
#define UI_Graph_Int 							5    //浮点型
#define UI_Graph_Float 						6    //整形
#define UI_Graph_Char 						7    //字符型

#define Type_Flag_Cap							0
#define Type_Flag_Level						1
#define Type_Flag_Chassis         2
#define Type_Flag_Spin	          3
#define Type_Flag_Auto_Aiming     4
#define Type_Flag_Shoot_Mode	    5
#define Type_Flag_Arm_size	      6
#define Type_Flag_Z	              7
#define Type_Flag_Frie_Speed	    8
#define Type_Flag_Vol	            9
#define Type_Flag_Predict	    10
#define Type_Flag_Energy		11
#define Type_Flag_Speed_up		12
#define Type_Flag_Buff		13


extern char *Char_Splicing(char *dest, char *src);

void Char_Painter(char name[3], char msg[], uint32_t Operate_tpye, uint16_t Sender_ID, uint16_t Receiver_ID,
	uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t Size, uint32_t start_x, uint32_t start_y, uint8_t Type_Flag);
void Graph_Painter(char name[3], uint32_t Operate_tpye, uint32_t Graphic_tpye, uint16_t Sender_ID, uint16_t Receiver_ID,
	uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y,
	uint32_t Radius, uint32_t start_angle, uint32_t end_angle);
void Num_Painter(char name[3], uint32_t Operate_tpye, uint32_t Graphic_tpye, uint16_t Sender_ID, uint16_t Receiver_ID,
	uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t start_x, uint32_t start_y,
	uint32_t start_angle, uint32_t end_angle, int Int, float Float);
void UI_Delete(uint8_t Del_Operate, uint8_t Del_Layer, uint16_t Sender_ID, uint16_t Receiver_ID);
void UI_Map(uint16_t Target_Robot_ID, float Target_Position_x, float Target_Position_y, float Reserverd);
void Line_Of_Sight(uint16_t Sender_ID, uint16_t Receiver_ID, uint32_t Layer[7], uint32_t Color[7], uint32_t Width[7],
	uint32_t Start_x[7], uint32_t Start_y[7], uint32_t End_x[7], uint32_t End_y[7]);


typedef enum
{
	Nop_e,		//空操作
	ADD_e,		//增加
	Change_e,	//修改
	Delete_e,	//删除
}UI_graph_mode_e;

typedef enum
{
	Nopd_e,		//空操作
	Layer_e,	//图层
	ALL_e,		//全部
}UI_graph_delete_e;

typedef enum
{
	Line_e,		//直线
	Rectangle_e,//矩形
	Circle_e,	//整圆
	Ellipse_e,	//椭圆
	Arc_e,		//圆弧
	Int_e,		//浮点型
	Float_e,	//整形
	Char_e,		//字符型
}UI_graph_type_e;

typedef enum
{
	Main_e,			//红蓝主色
	Yellow_e,		//黄色
	Green_e,		//绿色
	Orange_e,		//橙色
	Purplish_red_e,	//紫红色
	Pink_e,			//粉色
	Cyan_e,			//青色
	Black_e,		//黑色
	White_e,		//白色
}UI_graph_color_e;

class UIGraph
{
public:
	UIGraph(char name[3], UI_graph_type_e Type);
	void Add(char msg[], uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t Size, uint32_t start_x, uint32_t start_y);
	void Add(uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y,
		uint32_t Radius, uint32_t start_angle, uint32_t end_angle);
	void Add(int Int, uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t Size, uint32_t start_x, uint32_t start_y);
	void Add(float Float, uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t Size, uint32_t Lenth, uint32_t start_x, uint32_t start_y);

	void Change(char msg[], uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t Size, uint32_t start_x, uint32_t start_y);
	void Change(uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y,
		uint32_t Radius, uint32_t start_angle, uint32_t end_angle);
	void Change(int Int, uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t Size, uint32_t start_x, uint32_t start_y);
	void Change(float Float, uint32_t Layer, uint32_t Color, uint32_t Width, uint32_t Size, uint32_t Lenth, uint32_t start_x, uint32_t start_y);

	UI_graph_type_e Type;
	int8_t GetData(uint8_t *ptr, uint8_t &Len);
private:
	char *CharData;
	graphic_data_struct_t Data;
};

#endif /* __PROTOCOL_UI_H */



