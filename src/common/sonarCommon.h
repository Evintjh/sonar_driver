/*
* @file sonarCommon.h
* @brief 设备通用定义
* @details
* @author liuhai
* @date 2019/10/15 11:05
*/

#pragma once
#include <cstdint>
#include <string>

const uint16_t UDP_SOFT_ASK_PORT = 1009;    //软件udp端口
const uint16_t UDP_DATA_PORT = 5001;        //声呐设备upd数据发送端口
const uint16_t TCP_CMD_PORT = 5007;         //声呐设备tcp命令接收端口
const uint16_t UDP_REC_PORT = 5008;         //声呐设备udp接收端口
const uint16_t UDP_REC_SDK_PORT = 5009;     //声呐设备udp接收端口（SDK）
const uint16_t UDP_DEV_ASK_PORT = 9001;     //设备udp查询端口

const int IMAGE_WIDTH = 968;
const int IMAGE_HEIGHT = 512;
