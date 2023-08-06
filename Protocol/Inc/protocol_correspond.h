#ifndef _PROTOCOL_CORRESPOND_H
#define _PROTOCOL_CORRESPOND_H

#include "dev_system.h"

#define COMMON_SERIAL_HEADER 0xff

class correspond_bag
{
public:
    correspond_bag(uint8_t Header_, uint8_t Mode_, uint8_t Len_);
    void Pack(const void *ptr);
    int8_t Check(const void *ptr);
    int8_t Check(const void *ptr, uint8_t len);

    uint8_t *Data_ptr;//总数据指针
    uint8_t Lenth;//总数据长度
private:
    uint8_t Header;//帧头
    uint8_t Mode;//功能码
    uint8_t *Data;//数据部分
    uint8_t CRC8;//crc8校验
    uint8_t Len;//数据部分长度
};
extern "C++"
{
template<typename T>
class correspond_bag_
{
public:
    correspond_bag_(uint8_t Header, uint8_t Mode);
    void Pack(const void *ptr);
    int8_t Check(const void *ptr);

    uint8_t Lenth;//总数据长度

    // Bag_t Bag;

    struct Bag_t
    {
        uint8_t Header;
        uint8_t Mode;
        T Data;
        uint8_t CRC8;
    }__attribute__((packed));//关闭内存对齐
};
}


#endif
