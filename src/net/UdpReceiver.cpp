// #include "UdpReceiver.h"
// #include "../common/sonarCommon.h"

// #ifdef USE_WINDOWS
// UdpReceiver::UdpReceiver(CircleBuffer *&circleBuffer, QObject *parent)
//         : QObject(parent),
//           m_circleBuffer(circleBuffer),
//           m_udpSocket(INVALID_SOCKET) {
//     WSADATA wsa;
//     WSAStartup(MAKEWORD(2, 2), &wsa);
// }

// UdpReceiver::~UdpReceiver() { }

// void UdpReceiver::threadFunc() {
//     udpInit();
//     int m_recvAddrSize = sizeof(m_recvAddr);
//     char *buffer       = new char[PACK_SIZE];
//     while (m_runFlag) {
//         memset(buffer, 0, PACK_SIZE);
//         int recvNum = recvfrom(m_udpSocket, buffer, PACK_SIZE, 0, (SOCKADDR *)&m_recvAddr, &m_recvAddrSize);
//         if (recvNum == -1) {
//             continue;
//         }
//         m_circleBuffer->writeData(buffer, PACK_SIZE);
//     }
//     closesocket(m_udpSocket);
//     m_udpSocket = INVALID_SOCKET;
// }

// void UdpReceiver::udpInit() {
//     m_udpSocket = socket(AF_INET, SOCK_DGRAM, 0);

//     int netBuffer = 1'024 * 1'024 * 10;  // 10M 缓冲
//     setsockopt(m_udpSocket, SOL_SOCKET, SO_RCVBUF, (const char *)&netBuffer, sizeof(int));

//     int outTime = 50;  //  50ms 超时等待
//     setsockopt(m_udpSocket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&outTime, sizeof(int));

//     m_udpAddr.sin_family           = AF_INET;
//     m_udpAddr.sin_port             = htons(UDP_REC_SDK_PORT);
//     m_udpAddr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
//     bind(m_udpSocket, (sockaddr *)&m_udpAddr, sizeof(m_udpAddr));
// }

// #elif USE_LINUX
// UdpReceiver::UdpReceiver(CircleBuffer *&circleBuffer, QObject *parent)
//         : QObject(parent),
//           m_circleBuffer(circleBuffer),
//           m_udpSocket(-1) { }

// UdpReceiver::~UdpReceiver() { }

// void UdpReceiver::threadFunc() {
//     udpInit();
//     socklen_t m_recvAddrSize = sizeof(m_recvAddr);
//     char *buffer             = new char[PACK_SIZE];
//     while (m_runFlag) {
//         memset(buffer, 0, PACK_SIZE);
//         int recvNum = recvfrom(m_udpSocket, buffer, PACK_SIZE, 0, (struct sockaddr *)&m_recvAddr, &m_recvAddrSize);
//         if (recvNum == -1) {
//             continue;
//         }
//         m_circleBuffer->writeData(buffer, PACK_SIZE);
//     }

//     close(m_udpSocket);
//     m_udpSocket = -1;
// }

// void UdpReceiver::udpInit() {
//     m_udpSocket = socket(AF_INET, SOCK_DGRAM, 0);

//     int netBuffer = 1'024 * 1'024 * 10;  // 10M 缓冲
//     setsockopt(m_udpSocket, SOL_SOCKET, SO_RCVBUF, (const char *)&netBuffer, sizeof(int));

//     struct timeval outTime;
//     outTime.tv_sec  = 0;
//     outTime.tv_usec = 50'000;  //  50ms 超时等待
//     setsockopt(m_udpSocket, SOL_SOCKET, SO_RCVTIMEO, &outTime, sizeof(outTime));

//     m_udpAddr.sin_family      = AF_INET;
//     m_udpAddr.sin_port        = htons(UDP_REC_SDK_PORT);
//     m_udpAddr.sin_addr.s_addr = INADDR_ANY;
//     bind(m_udpSocket, (struct sockaddr *)&m_udpAddr, sizeof(m_udpAddr));
// }
// #endif


#include "UdpReceiver.h"
#include "../common/sonarCommon.h"

#ifdef USE_WINDOWS
UdpReceiver::UdpReceiver(CircleBuffer *&circleBuffer) : m_circleBuffer(circleBuffer), m_udpSocket(INVALID_SOCKET) {
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
}

UdpReceiver::~UdpReceiver() {
    if (m_udpSocket != INVALID_SOCKET) {
        closesocket(m_udpSocket);
    }
}

void UdpReceiver::threadFunc() {
    udpInit();
    int m_recvAddrSize = sizeof(m_recvAddr);
    char *buffer       = new char[PACK_SIZE];
    while (m_runFlag) {
        memset(buffer, 0, PACK_SIZE);
        int recvNum = recvfrom(m_udpSocket, buffer, PACK_SIZE, 0, (SOCKADDR *)&m_recvAddr, &m_recvAddrSize);
        if (recvNum == -1) {
            continue;
        }
        m_circleBuffer->writeData(buffer, PACK_SIZE);
    }
    delete[] buffer;
    closesocket(m_udpSocket);
    m_udpSocket = INVALID_SOCKET;
}

void UdpReceiver::udpInit() {
    m_udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udpSocket == INVALID_SOCKET) {
        throw std::runtime_error("Failed to create UDP socket");
    }

    int netBuffer = 1'024 * 1'024 * 10;  // 10M buffer
    setsockopt(m_udpSocket, SOL_SOCKET, SO_RCVBUF, (const char *)&netBuffer, sizeof(int));

    int outTime = 50;  // 50ms timeout
    setsockopt(m_udpSocket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&outTime, sizeof(int));

    m_udpAddr.sin_family           = AF_INET;
    m_udpAddr.sin_port             = htons(UDP_REC_SDK_PORT);
    m_udpAddr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
    if (bind(m_udpSocket, (sockaddr *)&m_udpAddr, sizeof(m_udpAddr)) == SOCKET_ERROR) {
        closesocket(m_udpSocket);
        throw std::runtime_error("Failed to bind UDP socket");
    }
}

#elif USE_LINUX
UdpReceiver::UdpReceiver(CircleBuffer *&circleBuffer) : m_circleBuffer(circleBuffer), m_udpSocket(-1) { }

UdpReceiver::~UdpReceiver() {
    if (m_udpSocket != -1) {
        close(m_udpSocket);
    }
}

void UdpReceiver::threadFunc() {
    udpInit();
    socklen_t m_recvAddrSize = sizeof(m_recvAddr);
    char *buffer             = new char[PACK_SIZE];
    while (m_runFlag) {
        memset(buffer, 0, PACK_SIZE);
        int recvNum = recvfrom(m_udpSocket, buffer, PACK_SIZE, 0, (struct sockaddr *)&m_recvAddr, &m_recvAddrSize);
        if (recvNum == -1) {
            continue;
        }
        m_circleBuffer->writeData(buffer, PACK_SIZE);
    }
    delete[] buffer;
    close(m_udpSocket);
    m_udpSocket = -1;
}

void UdpReceiver::udpInit() {
    m_udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udpSocket < 0) {
        throw std::runtime_error("Failed to create UDP socket");
    }

    int netBuffer = 1'024 * 1'024 * 10;  // 10M buffer
    setsockopt(m_udpSocket, SOL_SOCKET, SO_RCVBUF, (const char *)&netBuffer, sizeof(int));

    struct timeval outTime;
    outTime.tv_sec  = 0;
    outTime.tv_usec = 50'000;  // 50ms timeout
    setsockopt(m_udpSocket, SOL_SOCKET, SO_RCVTIMEO, &outTime, sizeof(outTime));

    m_udpAddr.sin_family      = AF_INET;
    m_udpAddr.sin_port        = htons(UDP_REC_SDK_PORT);
    m_udpAddr.sin_addr.s_addr = INADDR_ANY;
    if (bind(m_udpSocket, (struct sockaddr *)&m_udpAddr, sizeof(m_udpAddr)) < 0) {
        close(m_udpSocket);
        throw std::runtime_error("Failed to bind UDP socket");
    }
}
#endif