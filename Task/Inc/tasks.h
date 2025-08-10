#ifndef __TASKS_H
#define __TASKS_H

#include "Robot_Task.h"
#include "Message_Task.h"
#include "Guard_Task.h"
#include "Correspond_Task.h"
#include "UIDraw_Task.h"

// 有符号整数类型
typedef signed char        int8_t;
typedef short int          int16_t;
typedef int                int32_t;
typedef long long int      int64_t;

// 无符号整数类型
typedef unsigned char      uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;

extern Chassis_Ctrl  Chassis;
extern Gimbal_Ctrl   Gimbal;
extern Message_Ctrl  Message;
extern Guard_Ctrl    Guard;
extern Correspondence_ctrl Corres;

#endif
