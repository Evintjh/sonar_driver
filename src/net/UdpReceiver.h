// /*!
// * @file UdpReceiver.h
// * @brief udp数据接收类
// * @details 使用QUdpsocket
// * @author liuhai
// * @date
// */
// #pragma once

// #ifdef USE_WINDOWS
// #include <winsock2.h>
// #include <windows.h>
// #elif USE_LINUX
// #include <sys/types.h>
// #include <sys/socket.h>
// #include <sys/ioctl.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include <signal.h>
// #include <unistd.h>
// #include <string.h>
// #endif

// #include <iostream>
// #include <stdio.h>
// #include "LhForwardFrame.h"
// #include "../common/CircleBuffer.h"
// #include "../common/WhileThread.h"

// #include <QDebug>
// #include <QObject>

// class UdpReceiver : public QObject, public WhileThread
// {
//     Q_OBJECT

// public:
//     explicit UdpReceiver(CircleBuffer*& circleBuffer, QObject *parent = nullptr);
// 	~UdpReceiver();

//     /**
//      * @brief socket初始化
//      */
//     void udpInit();

// signals:
//     //void sigIpAddr(char*);
//     //void sigDeviceData(QByteArray, char*);
//     void sigDeviceData(QByteArray, QString);

// private:
//     uint16_t PACK_SIZE = LhForwardSDK::LhForwardFrame::PACK_SIZE;
//     void threadFunc();
//     CircleBuffer*& m_circleBuffer;

// #ifdef USE_WINDOWS
//     SOCKET m_udpSocket;
//     SOCKADDR_IN m_udpAddr;
//     SOCKADDR_IN m_recvAddr;
// #elif USE_LINUX
//     int m_udpSocket;
//     struct sockaddr_in m_udpAddr;
//     struct sockaddr_in m_recvAddr;
// #endif

// };



#pragma once

#ifdef USE_WINDOWS
#include <windows.h>
#include <winsock2.h>
#elif USE_LINUX
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include "../common/CircleBuffer.h"
#include "../common/WhileThread.h"
#include "LhForwardFrame.h"

class UdpReceiver : public WhileThread {
public:
    explicit UdpReceiver(CircleBuffer *&circleBuffer);
    ~UdpReceiver();

    void udpInit();

private:
    uint16_t PACK_SIZE = LhForwardSDK::LhForwardFrame::PACK_SIZE;
    void threadFunc();
    CircleBuffer *&m_circleBuffer;

#ifdef USE_WINDOWS
    SOCKET m_udpSocket;
    SOCKADDR_IN m_udpAddr;
    SOCKADDR_IN m_recvAddr;
#elif USE_LINUX
    int m_udpSocket;
    struct sockaddr_in m_udpAddr;
    struct sockaddr_in m_recvAddr;
#endif
};