#include "protocol_correspond.h"
#include "protocol_crc.h"
#include "drivers_statistic.h"
ValidData judge;

//Header_ 帧头 Mode_ 功能码 Len_ 数据部分长度，在结构体数据格式仅为uint8_t时，建议使用sizeof传入
correspond_bag::correspond_bag(uint8_t Header_, uint8_t Mode_, uint8_t Len_)
{
    this->Header = Header_;
    this->Mode = Mode_;
    this->Len = Len_;
    this->Lenth = this->Len + 3;
    Data_ptr = NULL;
}

void correspond_bag::Pack(const void *ptr)
{
    if(judge.IsInvalid(ptr))
    {
        return;
    }

    if(Data_ptr == NULL)
    {
        Data_ptr = new uint8_t[Len + 3];
        Data_ptr[0] = Header;
        Data_ptr[1] = Mode;
    }

    Data = (uint8_t *)ptr;
    memcpy(&Data_ptr[2], Data, Len);
    Append_CRC8_Check_Sum(Data_ptr, Len + 3);
    CRC8 = Data_ptr[Len + 2];
}

//该函数适配串口数据格式，默认第一位为长度
int8_t correspond_bag::Check(const void *ptr)
{
    if(judge.IsInvalid(ptr))
    {
        return -1;
    }

    uint8_t *ch = (uint8_t *)ptr;
    if(Verify_CRC8_Check_Sum(&ch[1], ch[0]) == 0)
    {
        return -2;
    }

    if(Lenth != ch[0] || Header != ch[1] || Mode != ch[2])
    {
        return 0;
    }
    memcpy(&Data_ptr[2], &ch[1], Len);

    Data = &ch[3];
    CRC8 = ch[ch[0]];
    return 1;
}

//ptr数据起始地址 lenth数据总长度
int8_t correspond_bag::Check(const void *ptr, uint8_t lenth)
{
    if(judge.IsInvalid(ptr) || judge.IsInvalid(lenth))
    {
        return -1;
    }

    uint8_t *ch = (uint8_t *)ptr;
    if(Verify_CRC8_Check_Sum(&ch[0], lenth) == 0)
    {
        return -2;
    }

    if(Lenth != lenth || Header != ch[0] || Mode != ch[1])
    {
        return 0;
    }
    memcpy(&Data_ptr[2], &ch[0], Len);

    Data = &ch[2];
    CRC8 = ch[lenth - 1];
    return 1;
}

// template<typename T>
// correspond_bag_<T>::correspond_bag_(uint8_t Header, uint8_t Mode)
// {
//     this->Bag.Header = Header;
//     this->Bag.Mode = Mode;
//     this->Lenth = sizeof(T) + 3;
// }

// template<typename T>
// void correspond_bag_<T>::Pack(const void *ptr)
// {
//     if(judge.IsInvalid(ptr))
//     {
//         return;
//     }

//     Data = (uint8_t *)ptr;
//     memcpy(&Bag.Data, Data, Lenth - 3);
//     Append_CRC8_Check_Sum(&Bag, Lenth);
// }

