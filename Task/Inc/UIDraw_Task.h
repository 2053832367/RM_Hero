#ifndef UIDraw_TASK_H
#define UIDraw_TASK_H

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "protocol_ui.h"
#include "drivers_statistic.h"

#ifdef __cplusplus
extern "C" {
#endif

    void UIDraw_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

extern QueueHandle_t Message_Queue;


typedef enum
{
    UIDefult,
    UIADD,
    UIChange,
}UI_Mode_e;

class UI_Graph_c
{
public:
    UI_Graph_c()
        :Chassis_Mode("cm", Char_e),
        Relative_Line("ra", Line_e),
        Main_Line("ml", Line_e),
        Cap_Per("cr", Int_e),
        Cap_Power("cp", Float_e),
        Cap_Line("cl", Line_e),
        Line1("l1", Line_e),
        Line2("l2", Line_e),
        Line3("l3", Line_e),
        Line4("l4", Line_e),
        Line5("l5", Line_e),
        Line6("l6", Line_e),
        Line7("l7", Line_e)
    {}

    UIGraph Chassis_Mode;
    UIGraph Relative_Line;
    UIGraph Main_Line;

    UIGraph Cap_Per;
    UIGraph Cap_Power;
    UIGraph Cap_Line;

    UIGraph Line1;
    UIGraph Line2;
    UIGraph Line3;
    UIGraph Line4;
    UIGraph Line5;
    UIGraph Line6;
    UIGraph Line7;
};


class UI_Draw_Ctrl :public Statistic, UI_Graph_c
{
public:
    UI_Mode_e Mode;

    void Init();
    void Feedback_Update();
    void UI_ADD();
    void UI_Change();
    void UI_Sent(UIGraph *Graph);
private:
    void UI_Sent(uint8_t *ptr, uint8_t Len);
    void UI_Char_Sent(uint8_t *ptr);

    uint8_t Seq;
    uint8_t Len;
    uint8_t GraphData[105];

    uint16_t standard_ID1;//本车id
    uint16_t standard_ID2;//客户端id

    ext_client_custom_graphic_single_tt ext_client_custom_graphic_single;
    ext_client_custom_character_tt ext_client_custom_character;
};

UI_Draw_Ctrl *get_UI_Draw_Ctrl_Pointer();
#endif

