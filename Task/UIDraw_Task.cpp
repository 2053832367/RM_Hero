#include "UIDraw_Task.h"
#include "tasks.h"
#include "arm_math.h"


UI_Draw_Ctrl UIDraw;
bool UI_Send = 1;

void UIDraw_Task(void *pvParameters)
{
/* USER CODE BEGIN StartDefaultTask */
	UIDraw.Init();
  /* Infinite loop */
  for(;;)
  {		
		UIDraw.Feedback_Update();
    if(UIDraw.Mode == UIADD)
    {
			UIDraw.UI_ADD();
    }
    if(UIDraw.Mode == UIChange)
    {
			UIDraw.UI_Change();
    }
		xQueueSend(Message_Queue, &ID_Data[UIdrawData], 0);
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */  
}

void UI_Draw_Ctrl::Init()
{
//    usart7_DMA_init();
    while(Message.robo->game_robot_state.robot_id == 0)
    {
        vTaskDelay(1);
    }
}
void UI_Draw_Ctrl::Feedback_Update()
{
    Sender_ID = Message.robo->game_robot_state.robot_id;
    if(Message.robo->game_robot_state.robot_id < 100)
    {
        Receiver_ID = 0x100 + uint32_t(Message.robo->game_robot_state.robot_id);
    }
    else
    {
        Receiver_ID = 0x164 + uint32_t(Message.robo->game_robot_state.robot_id%100);
    }

    if(UI_Send == true)
    {
        Mode = UIADD;
        UI_Send = false;
    }
    else
    {
        Mode = UIChange;
    }

    Statistic_Update(xTaskGetTickCount());
}

void UI_Draw_Ctrl::UI_ADD()
{
//    for(uint8_t i = 0;i < 2;i++)
//    {
//			//name,Operate_tpye,Graphic_tpye,Sender_ID,Receiver_ID,Layer,Color,Width,start_x,start_y,start_angle,end_angle,Int,Float
//			//name,Operate_tpye,Graphic_tpye,Sender_ID,Receiver_ID,Layer,Color,Width,start_x,start_y,end_x,end_y,Radius,start_angle,end_angle
//        //电容状态
//        Num_Painter("vo", UI_Graph_ADD, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_White, 3, 1600, 410, 20, NULL, NULL, 0);
//        Num_Painter("vo", UI_Graph_ADD, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_White, 3, 1600, 410, 20, NULL, NULL, 0);
//        Num_Painter("li", UI_Graph_ADD, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_White, 3, 1600, 370, 20, NULL, NULL, 0);
//        Graph_Painter(" ", UI_Graph_ADD, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_White, 10, 500, 100, 500, 100, NULL, NULL, NULL);

//				//底盘相对角度
//        Graph_Painter("Li4", UI_Graph_ADD, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_White, 10, 960, 540, 960, 540, NULL, NULL, NULL);
//				Graph_Painter("Li5", UI_Graph_ADD, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Main, 17, 120, 625, 120, 750, NULL, NULL, NULL);

//        //近战坐标线，弹道偏移
//        Graph_Painter("LI6", UI_Graph_ADD, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Yellow, 3, 940, 540, 940, 240, NULL, NULL, NULL);//960,540-240
//        Graph_Painter("LI7", UI_Graph_ADD, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Yellow, 3, 880, 455, 1000, 455, NULL, NULL, NULL);//930,990
//        Graph_Painter("Li8", UI_Graph_ADD, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Yellow, 3, 890, 415, 990, 415, NULL, NULL, NULL);//920,1000
//        Graph_Painter("Li9", UI_Graph_ADD, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Yellow, 3, 900, 395, 980, 395, NULL, NULL, NULL);//910,1010
//        Graph_Painter("LI1", UI_Graph_ADD, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Yellow, 3, 910, 375, 970, 375, NULL, NULL, NULL);//900,1020
//        Graph_Painter("LI2", UI_Graph_ADD, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Yellow, 3, 927, 354, 953, 354, NULL, NULL, NULL);//900,1020
//    }
		for(uint8_t i = 0;i < 2;i++)
    {
			//Draw_Number(name0,Operate_tpye0,Graphic_tpye0,Layer0,Color0,start_angle0,end_angle0,Width0,start_x0,start_y0,Number0)
			//Draw_Graphic(name0,Operate_tpye0,Graphic_tpye0,Layer0,Color0,start_angle0,end_angle0,Width0,start_x0,start_y0,radius0,end_x0,end_y0)
        UIDraw.Draw_Number(	"vo", UI_Graph_ADD, UI_Graph_Float , 3, Graphic_Color_White, 20 , NULL , 3 , 1600, 410 , 0 ,
														"vo", UI_Graph_ADD, UI_Graph_Float , 3, Graphic_Color_White, 20 , NULL , 3 , 1600, 410 , 0 ,
														"li", UI_Graph_ADD, UI_Graph_Float , 3, Graphic_Color_White, 20 , NULL , 3 , 1600, 370 , 0 ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL );
			
			//电容状态 和 底盘相对角度
				UIDraw.Draw_Graphic(" ", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 10 , 500, 100 , NULL , 500, 100 ,
														"Li4", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 10 , 960, 540 , NULL , 960, 540 ,
														"Li5", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 17 , 120, 625 , NULL , 120, 750 ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL );
				
			//近战坐标线，弹道偏移
				UIDraw.Draw_Graphic("Li6", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 3 , 940, 540 , NULL , 940, 240 ,
														"Li7", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 3 , 880, 455 , NULL , 1000, 455 , 
														"Li8", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 3 , 890, 415 , NULL , 990, 415 , 
														"Li9", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 3 , 900, 395 , NULL , 980, 395 ,
														"Li1", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 3 , 910, 375 , NULL , 970, 375 ,
														"Li2", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 3 , 927, 354 , NULL , 953, 354 ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL);
    }
}

void UI_Draw_Ctrl::UI_Change()
{
		uint32_t x1, x2, x3, x4, y1, y2, y3, y4;
	
    //底盘相对角度
    x1 = 120 - arm_sin_f32(-Chassis.chassis_relative_RAD + 0.52359877f) * 100.0f;
    y1 = 625 + arm_cos_f32(-Chassis.chassis_relative_RAD + 0.52359877f) * 100.0f;
    x2 = 120 - arm_cos_f32(-Chassis.chassis_relative_RAD + 1.04719754f) * 100.0f;
    y2 = 625 - arm_sin_f32(-Chassis.chassis_relative_RAD + 1.04719754f) * 100.0f;
    x3 = 120 + arm_sin_f32(-Chassis.chassis_relative_RAD + 0.52359877f) * 100.0f;
    y3 = 625 - arm_cos_f32(-Chassis.chassis_relative_RAD + 0.52359877f) * 100.0f;
    x4 = 120 + arm_cos_f32(-Chassis.chassis_relative_RAD + 1.04719754f) * 100.0f;
    y4 = 625 + arm_sin_f32(-Chassis.chassis_relative_RAD + 1.04719754f) * 100.0f;
	
    //超级电容
    if(Message.SuperCapR.energy < 20)
    {
//        Num_Painter("vo", UI_Graph_Change, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_Orange, 3, 1300, 100, 20, NULL, NULL, Message.SuperCapR.energy);
//        Num_Painter("vo", UI_Graph_Change, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_Orange, 3, 1300, 100, 20, NULL, NULL, Message.SuperCapR.energy);
//        Graph_Painter(" ", UI_Graph_Change, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Orange, 10, 500, 100, 500 + Message.SuperCapR.energy * 8, 100, NULL, NULL, NULL);
				
				UIDraw.Draw_Number(	"vo", UI_Graph_Change, UI_Graph_Float , 3, Graphic_Color_Orange, 20 , NULL , 3 , 1600, 410 , Message.SuperCapR.energy * 1000 ,
														"vo", UI_Graph_Change, UI_Graph_Float , 3, Graphic_Color_Orange, 20 , NULL , 3 , 1600, 410 , Message.SuperCapR.energy * 1000 ,
														"li", UI_Graph_ADD, UI_Graph_Float , 3, Graphic_Color_White, 20 , NULL , 3 , 1600, 370 , Message.SuperCapR.power * 1000 ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL );
			//电容状态 和 底盘相对角度
				UIDraw.Draw_Graphic(" ", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_Orange, NULL , NULL , 10 , 500, 100 , NULL , 500 + Message.SuperCapR.energy * 8, 100 ,
														"Li4", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 10 , x4, y4 , NULL , x1, y1 ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL );
    }
    else if(Message.SuperCapR.energy < 50)
    {
//        Num_Painter("vo", UI_Graph_Change, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_Green, 3, 1300, 100, 20, NULL, NULL, Message.SuperCapR.energy);
//        Num_Painter("vo", UI_Graph_Change, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_Green, 3, 1300, 100, 20, NULL, NULL, Message.SuperCapR.energy);
//        Graph_Painter(" ", UI_Graph_Change, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Green, 10, 500, 100, 500 + Message.SuperCapR.energy * 8, 100, NULL, NULL, NULL);
			
				UIDraw.Draw_Number(	"vo", UI_Graph_Change, UI_Graph_Float , 3, Graphic_Color_Green, 20 , NULL , 3 , 1600, 410 , Message.SuperCapR.energy * 1000 ,
														"vo", UI_Graph_Change, UI_Graph_Float , 3, Graphic_Color_Green, 20 , NULL , 3 , 1600, 410 , Message.SuperCapR.energy * 1000 ,
														"li", UI_Graph_ADD, UI_Graph_Float , 3, Graphic_Color_White, 20 , NULL , 3 , 1600, 370 , Message.SuperCapR.power * 1000 ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL );
			//电容状态 和 底盘相对角度
				UIDraw.Draw_Graphic(" ", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_Green, NULL , NULL , 10 , 500, 100 , NULL , 500 + Message.SuperCapR.energy * 8, 100 ,
														"Li4", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 10 , x4, y4 , NULL , x1, y1 ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL );
    }
    else
    {
//        Num_Painter("vo", UI_Graph_Change, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_Yellow, 3, 1300, 100, 20, NULL, NULL, Message.SuperCapR.energy);
//        Num_Painter("vo", UI_Graph_Change, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_Yellow, 3, 1300, 100, 20, NULL, NULL, Message.SuperCapR.energy);
//        Graph_Painter(" ", UI_Graph_Change, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Yellow, 10, 500, 100, 500 + Message.SuperCapR.energy * 8, 100, NULL, NULL, NULL);
			
				UIDraw.Draw_Number(	"vo", UI_Graph_Change, UI_Graph_Float , 3, Graphic_Color_Yellow, 20 , NULL , 3 , 1600, 410 , Message.SuperCapR.energy * 1000 ,
														"vo", UI_Graph_Change, UI_Graph_Float , 3, Graphic_Color_Yellow, 20 , NULL , 3 , 1600, 410 , Message.SuperCapR.energy * 1000 ,
														"li", UI_Graph_ADD, UI_Graph_Float , 3, Graphic_Color_White, 20 , NULL , 3 , 1600, 370 , Message.SuperCapR.power * 1000 ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Float , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL );
			//电容状态 和 底盘相对角度
				UIDraw.Draw_Graphic(" ", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_Yellow, NULL , NULL , 10 , 500, 100 , NULL , 500 + Message.SuperCapR.energy * 8, 100 ,
														"Li4", UI_Graph_ADD, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , 10 , x4, y4 , NULL , x1, y1 ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL ,
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL , 
														"Nop", UI_Graph_Nop, UI_Graph_Line , 3, Graphic_Color_White, NULL , NULL , NULL , NULL , NULL , NULL , NULL , NULL );
    }
//    Num_Painter("li", UI_Graph_Change, UI_Graph_Float, Sender_ID, Receiver_ID, 3, Graphic_Color_Green, 3, 1360, 100, 20, NULL, NULL, Message.SuperCapR.power);
//		
//    Graph_Painter("Li4", UI_Graph_Change, UI_Graph_Line, Sender_ID, Receiver_ID, 3, Graphic_Color_Main, 13, x4, y4, x1, y1, NULL, NULL, NULL);
}

UI_Draw_Ctrl *get_UI_Draw_Ctrl_Pointer()
{
    return &UIDraw;
}


