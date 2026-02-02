#include <serial/serial.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <string>

enum class SendPackType : unsigned char
{
    /// 向MCU发送的数据包-类型1
    SEND_PACK_1 = 0,
    /// 向MCU发送的数据包-类型2
    SEND_PACK_2 = 1,
    /// debug用数据包，保留备用，使用时直接设置成与sendpack相同的值
    SEND_PACK_DEBUG = 2,
};
enum class RecvPackType : unsigned char
{
    NOT_VALID_PACK = 0xbc,
    /// 从 MCU 接收的对时包
    READ_PACK_FOR_TIME = 0,
    /// 从 MCU 接收的姿态包
    READ_PACK_FOR_POSE = 1,
};

class serialport
{
private:
    /* data */
public:
    serialport(/* args */);
    ~serialport();
    serial::Serial sp;
    bool InitSerialPort(std::string serial);
    int getBit(std::uint8_t b,int i);
    void setByteFlags(std::uint8_t &byte,int start_index,int len,bool flag);
};

