/**
 * @file judge_test_case.cpp
 * @brief 裁判系统通信协议发送端测试程序
 * @details 通过串口模拟裁判系统发送数据
 */

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <chrono>
#include <thread>
#include <ctime>
#include <iomanip>
#include <functional>
#include <random>
#define _REFEREE_CRC_H__

// 包含裁判系统头文件
#include "judge_send.hpp"

// 定义通信协议相关常量
#define SOF_BYTE 0xA5
#define UART_DEVICE "/dev/ttyUSB0"
#define UART_BAUD_RATE B115200

// crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
    {
        0x00,
        0x5e,
        0xbc,
        0xe2,
        0x61,
        0x3f,
        0xdd,
        0x83,
        0xc2,
        0x9c,
        0x7e,
        0x20,
        0xa3,
        0xfd,
        0x1f,
        0x41,
        0x9d,
        0xc3,
        0x21,
        0x7f,
        0xfc,
        0xa2,
        0x40,
        0x1e,
        0x5f,
        0x01,
        0xe3,
        0xbd,
        0x3e,
        0x60,
        0x82,
        0xdc,
        0x23,
        0x7d,
        0x9f,
        0xc1,
        0x42,
        0x1c,
        0xfe,
        0xa0,
        0xe1,
        0xbf,
        0x5d,
        0x03,
        0x80,
        0xde,
        0x3c,
        0x62,
        0xbe,
        0xe0,
        0x02,
        0x5c,
        0xdf,
        0x81,
        0x63,
        0x3d,
        0x7c,
        0x22,
        0xc0,
        0x9e,
        0x1d,
        0x43,
        0xa1,
        0xff,
        0x46,
        0x18,
        0xfa,
        0xa4,
        0x27,
        0x79,
        0x9b,
        0xc5,
        0x84,
        0xda,
        0x38,
        0x66,
        0xe5,
        0xbb,
        0x59,
        0x07,
        0xdb,
        0x85,
        0x67,
        0x39,
        0xba,
        0xe4,
        0x06,
        0x58,
        0x19,
        0x47,
        0xa5,
        0xfb,
        0x78,
        0x26,
        0xc4,
        0x9a,
        0x65,
        0x3b,
        0xd9,
        0x87,
        0x04,
        0x5a,
        0xb8,
        0xe6,
        0xa7,
        0xf9,
        0x1b,
        0x45,
        0xc6,
        0x98,
        0x7a,
        0x24,
        0xf8,
        0xa6,
        0x44,
        0x1a,
        0x99,
        0xc7,
        0x25,
        0x7b,
        0x3a,
        0x64,
        0x86,
        0xd8,
        0x5b,
        0x05,
        0xe7,
        0xb9,
        0x8c,
        0xd2,
        0x30,
        0x6e,
        0xed,
        0xb3,
        0x51,
        0x0f,
        0x4e,
        0x10,
        0xf2,
        0xac,
        0x2f,
        0x71,
        0x93,
        0xcd,
        0x11,
        0x4f,
        0xad,
        0xf3,
        0x70,
        0x2e,
        0xcc,
        0x92,
        0xd3,
        0x8d,
        0x6f,
        0x31,
        0xb2,
        0xec,
        0x0e,
        0x50,
        0xaf,
        0xf1,
        0x13,
        0x4d,
        0xce,
        0x90,
        0x72,
        0x2c,
        0x6d,
        0x33,
        0xd1,
        0x8f,
        0x0c,
        0x52,
        0xb0,
        0xee,
        0x32,
        0x6c,
        0x8e,
        0xd0,
        0x53,
        0x0d,
        0xef,
        0xb1,
        0xf0,
        0xae,
        0x4c,
        0x12,
        0x91,
        0xcf,
        0x2d,
        0x73,
        0xca,
        0x94,
        0x76,
        0x28,
        0xab,
        0xf5,
        0x17,
        0x49,
        0x08,
        0x56,
        0xb4,
        0xea,
        0x69,
        0x37,
        0xd5,
        0x8b,
        0x57,
        0x09,
        0xeb,
        0xb5,
        0x36,
        0x68,
        0x8a,
        0xd4,
        0x95,
        0xcb,
        0x29,
        0x77,
        0xf4,
        0xaa,
        0x48,
        0x16,
        0xe9,
        0xb7,
        0x55,
        0x0b,
        0x88,
        0xd6,
        0x34,
        0x6a,
        0x2b,
        0x75,
        0x97,
        0xc9,
        0x4a,
        0x14,
        0xf6,
        0xa8,
        0x74,
        0x2a,
        0xc8,
        0x96,
        0x15,
        0x4b,
        0xa9,
        0xf7,
        0xb6,
        0xe8,
        0x0a,
        0x54,
        0xd7,
        0x89,
        0x6b,
        0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
    unsigned char ucIndex;
    while (dwLength--)
    {
        ucIndex = ucCRC8 ^ (*pchMessage++);
        ucCRC8 = CRC8_TAB[ucIndex];
    }
    return (ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucExpected = 0;
    // 验证指针有效性和长度合理性
    if ((pchMessage == NULL) || (dwLength <= 2))
        return 0;
    ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
    return (ucExpected == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
    unsigned char ucCRC = 0;
    // 检查指针有效性和长度范围
    if ((pchMessage == NULL) || (dwLength <= 2))
        return;
    ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
    pchMessage[dwLength - 1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
    {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while (dwLength--)
    {
        chData = *pchMessage++;
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    // 检查指针有效性和长度范围
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
                                                                  pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wCRC = 0;
    // 检查指针有效性和长度范围
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}

/**
 * @brief 串口初始化函数
 * @param device 串口设备名称
 * @return 串口文件描述符，失败返回-1
 */
int uart_init(const char *device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {
        std::cerr << "Error opening serial port: " << device << std::endl;
        return -1;
    }

    // 配置串口参数
    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率
    cfsetispeed(&options, UART_BAUD_RATE);
    cfsetospeed(&options, UART_BAUD_RATE);

    // 设置数据位、停止位、校验位等
    options.c_cflag &= ~PARENB; // 无校验位
    options.c_cflag &= ~CSTOPB; // 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;            // 8位数据位
    options.c_cflag &= ~CRTSCTS;       // 禁用硬件流控
    options.c_cflag |= CREAD | CLOCAL; // 启用接收并忽略调制解调器状态线

    // 设置为原始模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // 设置读取超时
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    // 应用设置
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        std::cerr << "Error configuring serial port" << std::endl;
        close(fd);
        return -1;
    }

    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);

    return fd;
}

/**
 * @brief 打包裁判系统数据
 * @param cmd_id 命令ID
 * @param data 数据
 * @param data_len 数据长度
 * @param seq 序列号
 * @return 打包好的数据包
 */
std::vector<uint8_t> pack_judge_system_data(uint16_t cmd_id, const uint8_t *data, uint16_t data_len, uint8_t seq = 0)
{
    // 计算总长度: frame_header(5) + cmd_id(2) + data(n) + frame_tail(2)
    uint32_t total_len = 5 + 2 + data_len + 2;
    std::vector<uint8_t> packet(total_len, 0);

    // 填充帧头
    packet[0] = SOF_BYTE; // SOF

    // 数据长度，注意字节序
    uint16_t len_network = data_len;
    memcpy(&packet[1], &len_network, sizeof(uint16_t));

    // 序列号
    packet[3] = seq;

    // 计算帧头CRC8校验
    Append_CRC8_Check_Sum(&packet[0], 5); // 帧头校验

    // 填充命令ID，注意字节序
    uint16_t cmd_id_network = cmd_id;
    memcpy(&packet[5], &cmd_id_network, sizeof(uint16_t));

    // 填充数据
    if (data != nullptr && data_len > 0)
    {
        memcpy(&packet[7], data, data_len);
    }

    // 计算整包CRC16校验
    Append_CRC16_Check_Sum(&packet[0], total_len);

    return packet;
}

/**
 * @brief 字节序转换工具函数，用于处理结构体中的字段
 * @param input 输入结构体
 * @param output 输出结构体(字节序已转换)
 */
void convert_endianness_for_transmission(const void *input, void *output, size_t size)
{
    // 这个函数可以根据需要实现，用于处理复杂结构体中的字节序
    // 目前简单复制，在实际项目中可能需要对特定字段进行字节序转换
    memcpy(output, input, size);
}

/**
 * @brief 发送裁判系统数据
 * @param fd 串口文件描述符
 * @param cmd_id 命令ID
 * @param data 数据
 * @param data_len 数据长度
 * @param seq 序列号
 * @return 发送是否成功
 */
bool send_judge_system_data(int fd, uint16_t cmd_id, const uint8_t *data, uint16_t data_len, uint8_t seq = 0)
{
    // 1% 随机丢包
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(1, 100);

    if (dis(gen) == 1)
    {
        std::cout << "[模拟丢包] 本次数据未发送 (1%)" << std::endl;
        return true;
    }

    std::vector<uint8_t> packet = pack_judge_system_data(cmd_id, data, data_len, seq);

    // 发送数据
    ssize_t bytes_written = write(fd, packet.data(), packet.size());

    if (bytes_written != static_cast<ssize_t>(packet.size()))
    {
        std::cerr << "Error sending data: " << bytes_written << " bytes sent, expected "
                  << packet.size() << std::endl;
        return false;
    }

    // 打印发送的数据（调试用）
    std::cout << "Sent packet: ";
    for (size_t i = 0; i < packet.size(); ++i)
    {
        printf("%02X ", packet[i]);
    }
    std::cout << std::endl;

    return true;
}

/**
 * @brief 发送比赛状态数据
 * @param fd 串口文件描述符
 * @param game_type 比赛类型
 * @param game_progress 比赛阶段
 * @param stage_remain_time 当前阶段剩余时间，单位 s
 * @param sync_time_stamp 机器人接收到该指令的精确 Unix 时间，单位 us
 * @return 发送是否成功
 */
bool send_game_status(int fd, uint8_t game_type, uint8_t game_progress,
                      uint16_t stage_remain_time, uint64_t sync_time_stamp)
{
    // 使用judge_receive.h中定义的ext_game_status_t结构体
    ext_game_status_t game_status;

    // 填充数据
    game_status.game_type = game_type;
    game_status.game_progress = game_progress;
    game_status.stage_remain_time = stage_remain_time; // 注意：在发送前会自动进行字节序转换
    game_status.SyncTimeStamp = sync_time_stamp;

    // 发送数据
    return send_judge_system_data(fd, 0x0001, (const uint8_t *)&game_status, sizeof(ext_game_status_t));
}

/**
 * @brief 发送比赛结果数据
 * @param fd 串口文件描述符
 * @param winner 获胜方
 * @return 发送是否成功
 */
bool send_game_result(int fd, uint8_t winner)
{
    // 使用judge_receive.h中定义的ext_game_result_t结构体
    ext_game_result_t game_result;
    game_result.winner = winner;

    return send_judge_system_data(fd, 0x0002, (const uint8_t *)&game_result, sizeof(ext_game_result_t));
}

/**
 * @brief 发送机器人血量数据
 * @param fd 串口文件描述符
 * @param robot_hp 机器人血量数据结构体
 * @return 发送是否成功
 */
bool send_robot_hp(int fd, const ext_game_robot_HP_t *robot_hp)
{
    return send_judge_system_data(fd, 0x0003, (const uint8_t *)robot_hp, sizeof(ext_game_robot_HP_t));
}

/**
 * @brief 发送场地事件数据
 * @param fd 串口文件描述符
 * @param event_type 事件类型
 * @return 发送是否成功
 */
bool send_event_data(int fd, uint32_t event_type)
{
    // 使用judge_receive.h中定义的ext_event_data_t结构体
    ext_event_data_t event_data;
    event_data.event_type = event_type;

    return send_judge_system_data(fd, 0x0101, (const uint8_t *)&event_data, sizeof(ext_event_data_t));
}

/**
 * @brief 发送机器人状态数据
 * @param fd 串口文件描述符
 * @param robot_status 机器人状态结构体
 * @return 发送是否成功
 */
bool send_robot_status(int fd, const ext_game_robot_status_t *robot_status)
{
    return send_judge_system_data(fd, 0x0201, (const uint8_t *)robot_status, sizeof(ext_game_robot_status_t));
}

/**
 * @brief 发送实时功率热量数据
 * @param fd 串口文件描述符
 * @param power_heat_data 功率热量数据结构体
 * @return 发送是否成功
 */
bool send_power_heat_data(int fd, const ext_power_heat_data_t *power_heat_data)
{
    return send_judge_system_data(fd, 0x0202, (const uint8_t *)power_heat_data, sizeof(ext_power_heat_data_t));
}

/**
 * @brief 发送机器人位置数据
 * @param fd 串口文件描述符
 * @param robot_pos 机器人位置数据结构体
 * @return 发送是否成功
 */
bool send_robot_position(int fd, const ext_robot_pos_t *robot_pos)
{
    return send_judge_system_data(fd, 0x0203, (const uint8_t *)robot_pos, sizeof(ext_robot_pos_t));
}

/**
 * @brief 发送机器人增益数据
 * @param fd 串口文件描述符
 * @param buff 机器人增益数据结构体
 * @return 发送是否成功
 */
bool send_robot_buff(int fd, const ext_buff_t *buff)
{
    return send_judge_system_data(fd, 0x0204, (const uint8_t *)buff, sizeof(ext_buff_t));
}

/**
 * @brief 发送伤害状态数据
 * @param fd 串口文件描述符
 * @param hurt_data 伤害状态数据结构体
 * @return 发送是否成功
 */
bool send_hurt_data(int fd, const ext_hurt_data_t *hurt_data)
{
    return send_judge_system_data(fd, 0x0206, (const uint8_t *)hurt_data, sizeof(ext_hurt_data_t));
}

/**
 * @brief 发送实时射击信息
 * @param fd 串口文件描述符
 * @param shoot_data 射击信息数据结构体
 * @return 发送是否成功
 */
bool send_shoot_data(int fd, const ext_shoot_data_t *shoot_data)
{
    return send_judge_system_data(fd, 0x0207, (const uint8_t *)shoot_data, sizeof(ext_shoot_data_t));
}

/**
 * @brief 发送裁判警告信息
 * @param fd 串口文件描述符
 * @param level 警告等级
 * @param offending_robot_id 违规机器人ID
 * @param count 警告次数
 * @return 发送是否成功
 */
bool send_referee_warning(int fd, uint8_t level, uint8_t offending_robot_id, uint8_t count)
{
    ext_referee_warning_t warning;
    warning.level = level;
    warning.offending_robot_id = offending_robot_id;
    warning.count = count;

    return send_judge_system_data(fd, 0x0104, (const uint8_t *)&warning, sizeof(ext_referee_warning_t));
}

/**
 * @brief 发送飞镖发射口倒计时数据
 * @param fd 串口文件描述符
 * @param dart_remaining_time 飞镖发射口倒计时，单位s
 * @param dart_info 飞镖发射口状态
 * @return 发送是否成功
 */
bool send_dart_info(int fd, uint8_t dart_remaining_time, uint16_t dart_info)
{
    ext_dart_info_t dart_status;
    dart_status.dart_remaining_time = dart_remaining_time;
    dart_status.dart_info = dart_info;

    return send_judge_system_data(fd, 0x0105, (const uint8_t *)&dart_status, sizeof(ext_dart_info_t));
}

/**
 * @brief 发送飞镖机器人客户端指令数据
 * @param fd 串口文件描述符
 * @param dart_launch_opening_status 当前飞镖发射口状态
 * @param target_change_time 切换目标时间
 * @param latest_launch_cmd_time 最新发射指令时间
 * @return 发送是否成功
 */
bool send_dart_client_cmd(int fd, uint8_t dart_launch_opening_status,
                          uint16_t target_change_time, uint16_t latest_launch_cmd_time)
{
    ext_dart_client_cmd_t dart_cmd;
    dart_cmd.dart_launch_opening_status = dart_launch_opening_status;
    dart_cmd.reserved = 0;
    dart_cmd.target_change_time = target_change_time;
    dart_cmd.latest_launch_cmd_time = latest_launch_cmd_time;

    return send_judge_system_data(fd, 0x020A, (const uint8_t *)&dart_cmd, sizeof(ext_dart_client_cmd_t));
}

/**
 * @brief 打印当前比赛状态
 * @param game_progress 比赛阶段
 * @param remain_time 剩余时间
 */
void print_game_status(uint8_t game_progress, uint16_t remain_time)
{
    std::cout << "====================================" << std::endl;
    std::cout << "当前比赛状态: ";
    switch (game_progress)
    {
    case 0:
        std::cout << "未开始比赛";
        break;
    case 1:
        std::cout << "准备阶段";
        break;
    case 2:
        std::cout << "裁判系统自检阶段";
        break;
    case 3:
        std::cout << "五秒倒计时";
        break;
    case 4:
        std::cout << "比赛中";
        break;
    case 5:
        std::cout << "比赛结算中";
        break;
    default:
        std::cout << "未知状态";
    }
    std::cout << " | 剩余时间: " << remain_time << "秒" << std::endl;
    std::cout << "====================================" << std::endl;
}

/**
 * @brief 打印飞镖状态
 * @param opening_status 飞镖发射口状态
 * @param remaining_time 发射口倒计时
 */
void print_dart_status(uint8_t opening_status, uint8_t remaining_time)
{
    std::cout << "飞镖发射口状态: ";
    switch (opening_status)
    {
    case 0:
        std::cout << "已经开启";
        break;
    case 1:
        std::cout << "关闭";
        break;
    case 2:
        std::cout << "正在开启或者关闭中";
        break;
    default:
        std::cout << "未知状态";
    }

    if (remaining_time > 0)
    {
        std::cout << " | 倒计时: " << static_cast<int>(remaining_time) << "秒" << std::endl;
    }
    else
    {
        std::cout << std::endl;
    }
}

/**
 * @brief 获取当前系统时间戳（微秒）
 * @return 时间戳（微秒）
 */
uint64_t get_current_timestamp_us()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::time_point_cast<std::chrono::microseconds>(now).time_since_epoch().count();
    return static_cast<uint64_t>(us);
}

// 定义飞镖机器人测试状态机
enum class GamePhase
{
    STANDBY,     // 未开始
    PREPARATION, // 准备阶段 180s
    SELF_CHECK,  // 裁判系统自检 15s
    COUNTDOWN,   // 五秒倒计时 5s
    IN_GAME,     // 比赛中 420s
    SETTLEMENT   // 比赛结算 15s
};

// 定义飞镖发射状态机
enum class DartPhase
{
    CLOSED,    // 关闭状态
    OPENING,   // 正在开启
    OPENED,    // 已开启，等待发射
    COUNTDOWN, // 发射倒计时
    CLOSING    // 正在关闭
};

/**
 * @brief 主函数，测试发送裁判系统数据
 */
int main(int argc, char **argv)
{
    std::string port_device = UART_DEVICE;
    int send_interval_ms = 100; // 通信频率，10Hz

    // 解析命令行参数
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help")
        {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  -h, --help     显示帮助信息" << std::endl;
            std::cout << "  -p, --port DEV 指定串口设备 (默认: /dev/ttyUSB0)" << std::endl;
            std::cout << "  -i, --interval N 设置发送间隔(ms) (默认: 100)" << std::endl;
            return 0;
        }
        else if (arg == "-p" || arg == "--port")
        {
            if (i + 1 < argc)
            {
                port_device = argv[++i];
            }
        }
        else if (arg == "-i" || arg == "--interval")
        {
            if (i + 1 < argc)
            {
                send_interval_ms = std::atoi(argv[++i]);
                if (send_interval_ms < 10)
                {
                    std::cerr << "Interval too small. Using 10ms." << std::endl;
                    send_interval_ms = 10;
                }
            }
        }
    }

    // 初始化串口
    int fd = uart_init(port_device.c_str());
    if (fd < 0)
    {
        std::cerr << "Failed to initialize UART on " << port_device << std::endl;
        return 1;
    }

    std::cout << "UART initialized successfully on " << port_device << std::endl;

    // 初始化随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis_hp(800, 1000); // 血量随机范围

    // 状态机初始化
    GamePhase game_phase = GamePhase::STANDBY;
    DartPhase dart_phase = DartPhase::CLOSED;

    uint16_t game_time_remaining = 0; // 当前阶段剩余时间
    uint8_t game_type = 1;            // 比赛类型：RoboMaster 机甲大师超级对抗赛
    uint8_t game_progress = 0;        // 当前比赛阶段

    uint8_t dart_opening_status = 1;     // 飞镖发射口状态：1-关闭
    uint8_t dart_remaining_time = 0;     // 飞镖发射倒计时
    uint16_t target_change_time = 0;     // 切换目标时间
    uint16_t latest_launch_cmd_time = 0; // 最新发射指令时间

    // 初始化各种数据结构
    ext_game_robot_HP_t robot_hp_data;
    memset(&robot_hp_data, 0, sizeof(robot_hp_data));
    robot_hp_data.red_1_robot_HP = 1000;
    robot_hp_data.red_2_robot_HP = 1000;
    robot_hp_data.red_3_robot_HP = 1000;
    robot_hp_data.red_4_robot_HP = 1000;
    robot_hp_data.red_5_robot_HP = 1000;
    robot_hp_data.red_7_robot_HP = 1000;
    robot_hp_data.red_outpost_HP = 1000;
    robot_hp_data.red_base_HP = 5000;
    robot_hp_data.blue_1_robot_HP = 1000;
    robot_hp_data.blue_2_robot_HP = 1000;
    robot_hp_data.blue_3_robot_HP = 1000;
    robot_hp_data.blue_4_robot_HP = 1000;
    robot_hp_data.blue_5_robot_HP = 1000;
    robot_hp_data.blue_7_robot_HP = 1000;
    robot_hp_data.blue_outpost_HP = 1000;
    robot_hp_data.blue_base_HP = 5000;

    ext_game_robot_status_t robot_status_data;
    memset(&robot_status_data, 0, sizeof(robot_status_data));
    robot_status_data.robot_id = 7; // 7号为飞镖机器人
    robot_status_data.robot_level = 1;
    robot_status_data.current_HP = 1000;
    robot_status_data.maximum_HP = 1000;
    robot_status_data.shooter_barrel_cooling_value = 0;
    robot_status_data.shooter_barrel_heat_limit = 0;
    robot_status_data.chassis_power_limit = 0;
    robot_status_data.power_management_gimbal_output = 1;
    robot_status_data.power_management_chassis_output = 1;
    robot_status_data.power_management_shooter_output = 1;

    // 计时相关变量
    auto last_update_time = std::chrono::high_resolution_clock::now();
    auto last_hp_update_time = last_update_time;
    auto last_status_update_time = last_update_time;
    auto game_start_time = last_update_time;
    auto dart_phase_start_time = last_update_time;
    auto current_time = last_update_time;

    // 比赛专用计时器
    int total_game_seconds = 0;

    // 飞镖机器人特殊事件计时器
    bool first_dart_event_triggered = false;  // 60秒飞镖事件
    bool second_dart_event_triggered = false; // 180秒飞镖事件

    std::cout << "开始模拟裁判系统数据发送，模拟飞镖机器人测试用例..." << std::endl;
    std::cout << "按 Ctrl+C 退出程序" << std::endl;

    try
    {
        while (true)
        {
            current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update_time).count();

            // 每100ms更新一次状态和发送数据
            if (elapsed >= send_interval_ms)
            {
                last_update_time = current_time;

                // 计算比赛时间流逝
                static int ms_accumulated = 0;
                if (game_phase != GamePhase::STANDBY)
                {
                    ms_accumulated += elapsed;
                    while (ms_accumulated >= 1000 && game_time_remaining > 0)
                    {
                        --game_time_remaining;
                        ms_accumulated -= 1000;
                    }

                    if (game_phase == GamePhase::IN_GAME)
                    {
                        total_game_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                                                 current_time - game_start_time)
                                                 .count();
                    }
                }

                // 状态机处理
                switch (game_phase)
                {
                case GamePhase::STANDBY:
                    game_progress = 0; // 未开始比赛
                    game_time_remaining = 0;

                    // 开始比赛
                    game_phase = GamePhase::PREPARATION;
                    game_time_remaining = 3; // 准备阶段180秒
                    game_progress = 1;
                    std::cout << "比赛开始，进入准备阶段!" << std::endl;
                    break;

                case GamePhase::PREPARATION:
                    game_progress = 1; // 准备阶段
                    if (game_time_remaining == 0)
                    {
                        game_phase = GamePhase::SELF_CHECK;
                        game_time_remaining = 15; // 自检阶段15秒
                        game_progress = 2;
                        std::cout << "准备阶段结束，进入裁判系统自检阶段!" << std::endl;
                    }
                    break;

                case GamePhase::SELF_CHECK:
                    game_progress = 2; // 裁判系统自检阶段
                    if (game_time_remaining == 0)
                    {
                        game_phase = GamePhase::COUNTDOWN;
                        game_time_remaining = 1; // 倒计时5秒
                        game_progress = 3;
                        std::cout << "裁判系统自检完成，进入五秒倒计时!" << std::endl;
                    }
                    break;

                case GamePhase::COUNTDOWN:
                    game_progress = 3; // 五秒倒计时
                    if (game_time_remaining == 0)
                    {
                        game_phase = GamePhase::IN_GAME;
                        game_time_remaining = 100; // 比赛时间420秒
                        game_progress = 4;
                        game_start_time = current_time; // 记录比赛正式开始时间
                        std::cout << "比赛正式开始!" << std::endl;

                        // 重置飞镖相关状态
                        first_dart_event_triggered = false;
                        second_dart_event_triggered = false;
                    }
                    break;

                case GamePhase::IN_GAME:
                    game_progress = 4; // 比赛中

                    // 飞镖机器人在比赛进行60s和180s时触发发射事件
                    if (!first_dart_event_triggered && total_game_seconds >= 5)
                    {
                        dart_phase = DartPhase::OPENING;
                        dart_opening_status = 2; // 正在开启
                        dart_phase_start_time = current_time;
                        first_dart_event_triggered = true;
                        std::cout << "\n飞镖事件触发! (60秒标记)" << std::endl;
                        std::cout << "飞镖舱门开始打开..." << std::endl;
                    }

                    if (!second_dart_event_triggered && total_game_seconds >= 45)
                    {
                        dart_phase = DartPhase::OPENING;
                        dart_opening_status = 2; // 正在开启
                        dart_phase_start_time = current_time;
                        second_dart_event_triggered = true;
                        std::cout << "\n飞镖事件触发! (180秒标记)" << std::endl;
                        std::cout << "飞镖舱门开始打开..." << std::endl;
                    }

                    if (game_time_remaining == 0)
                    {
                        game_phase = GamePhase::SETTLEMENT;
                        game_time_remaining = 15; // 结算时间15秒
                        game_progress = 5;
                        std::cout << "比赛结束，进入结算阶段!" << std::endl;
                    }
                    break;

                case GamePhase::SETTLEMENT:
                    game_progress = 5; // 比赛结算中
                    if (game_time_remaining == 0)
                    {
                        // 发送比赛结果
                        uint8_t winner = (robot_hp_data.red_base_HP > robot_hp_data.blue_base_HP) ? 1 : 2; // 1红方胜，2蓝方胜
                        send_game_result(fd, winner);
                        std::cout << "比赛结束! " << (winner == 1 ? "红方胜利!" : "蓝方胜利!") << std::endl;

                        // 重新开始比赛
                        game_phase = GamePhase::STANDBY;
                        std::cout << "\n3秒后重新开始比赛..." << std::endl;
                        std::this_thread::sleep_for(std::chrono::seconds(3));
                    }
                    break;
                }

                // 飞镖状态机处理
                auto dart_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - dart_phase_start_time).count();

                switch (dart_phase)
                {
                case DartPhase::CLOSED:
                    dart_opening_status = 1; // 关闭
                    dart_remaining_time = 0;
                    break;

                case DartPhase::OPENING:
                    dart_opening_status = 2; // 正在开启
                    if (dart_elapsed >= 7)
                    { // 7秒后开启完成
                        dart_phase = DartPhase::OPENED;
                        dart_opening_status = 0; // 已开启
                        dart_phase_start_time = current_time;
                        std::cout << "飞镖舱门已完全打开!" << std::endl;
                    }
                    break;

                case DartPhase::OPENED:
                    dart_opening_status = 0; // 已开启
                    dart_phase = DartPhase::COUNTDOWN;
                    dart_remaining_time = 20; // 20秒倒计时
                    dart_phase_start_time = current_time;
                    std::cout << "飞镖发射倒计时开始: 20秒" << std::endl;
                    break;

                case DartPhase::COUNTDOWN:
                    dart_opening_status = 0; // 已开启
                    dart_remaining_time = std::max(0, 20 - static_cast<int>(dart_elapsed));

                    if (dart_elapsed >= 20)
                    { // 倒计时结束
                        dart_phase = DartPhase::CLOSING;
                        dart_opening_status = 2; // 关闭中
                        dart_phase_start_time = current_time;
                        std::cout << "飞镖发射倒计时结束，舱门开始关闭" << std::endl;

                        // 模拟发射事件
                        latest_launch_cmd_time = static_cast<uint16_t>(total_game_seconds % 65536);
                    }
                    break;

                case DartPhase::CLOSING:
                    dart_opening_status = 2; // 关闭中
                    dart_remaining_time = 0;

                    if (dart_elapsed >= 7)
                    { // 7秒后关闭完成
                        dart_phase = DartPhase::CLOSED;
                        dart_opening_status = 1; // 关闭
                        dart_phase_start_time = current_time;
                        std::cout << "飞镖舱门已完全关闭!" << std::endl;
                    }
                    break;
                }

                // 发送周期性数据包

                // 1Hz 发送比赛状态数据
                send_game_status(fd, game_type, game_progress, game_time_remaining, get_current_timestamp_us());

                // 3Hz 发送机器人血量数据
                auto hp_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_hp_update_time).count();
                if (hp_elapsed >= 333)
                { // 约3Hz
                    last_hp_update_time = current_time;
                    send_robot_hp(fd, &robot_hp_data);

                    // 随机波动血量以模拟真实比赛
                    if (game_phase == GamePhase::IN_GAME)
                    {
                        robot_hp_data.red_1_robot_HP = dis_hp(gen);
                        robot_hp_data.blue_1_robot_HP = dis_hp(gen);
                    }
                }

                // 10Hz 发送机器人状态数据
                auto status_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_status_update_time).count();
                if (status_elapsed >= 100)
                { // 约10Hz
                    last_status_update_time = current_time;
                    send_robot_status(fd, &robot_status_data);

                    // 在比赛中随机变化当前血量
                    if (game_phase == GamePhase::IN_GAME)
                    {
                        robot_status_data.current_HP = dis_hp(gen);
                    }
                }

                // 1Hz 发送飞镖相关数据
                if (game_phase == GamePhase::IN_GAME)
                {
                    send_dart_info(fd, dart_remaining_time, dart_opening_status);

                    // 3Hz 发送飞镖客户端指令数据
                    if (hp_elapsed >= 333)
                    { // 复用3Hz计时器
                        send_dart_client_cmd(fd, dart_opening_status, target_change_time, latest_launch_cmd_time);
                    }
                }

                // 打印当前状态信息
                print_game_status(game_progress, game_time_remaining);
                if (game_phase == GamePhase::IN_GAME)
                {
                    print_dart_status(dart_opening_status, dart_remaining_time);
                }
            }

            // 休眠一小段时间，避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // 关闭串口
    close(fd);

    return 0;
}