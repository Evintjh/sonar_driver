/*!
* Copyright (C) 珠海蓝衡科技有限公司
* All rights reserved
* @File LhForwardCMD.h
* @Author liuhai
* @Brief 前视声呐数据解析库头文件
* @Details 数据解析与拼接
* @LastEditors liuhai
* @LastEditTime 2020/4/13 11:16
* @Version 3.3
*/

#pragma once

#ifdef USE_WINDOWS
#define DLL_API _declspec(dllexport)
#else
#define DLL_API
#endif // !DLL_API

#ifdef USE_WINDOWS
#include <WinSock2.h>
#elif USE_LINUX
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#define SOCKADDR_IN sockaddr_in
#define SOCKADDR sockaddr
#define closesocket close
#endif
#include <string>
#include <cstdint>
using std::string;

namespace LhForwardSDK
{
#ifdef USE_WINDOWS
    typedef SOCKET TYPE_SOCKET;
#elif USE_LINUX
    typedef int TYPE_SOCKET;
    #define INVALID_SOCKET -1
#endif

#define VERSION_BUFFER 4096         // 定义接收的版本号的缓冲的大小

    class DLL_API LhForwardCMD
    {
    public:
        enum LfCMDRet
        {
            ok = 0,
            connectErr = 1,		//设备连接错误，只有connectDevice函数可能返回，说明连接失败，请检查网络连接
            noConnect = 2,		//设备未连接错误，sendCMD函数可能返回，说明未与设备连接就进行命令发送
            cmdArgErr = 3,		//命令参数错误，sendCMD函数可能返回，说明命令的参数不匹配，注：只能检测出部分参数错误
            sendErr = 4,		//命令发送错误，sendCMD函数可能返回，说明可能存在设备连接问题，重新连接后再次发送
            receiveErr = 5,		//命令回复错误，sendCMD函数可能返回，说明声呐设备接收到了发送的命令，但是命令不正确，可能是命令参数错误
            noReceive = 6,		//没有命令回复，sendCMD函数可能返回，说明可能存在连接问题，请重新连接后再次发送
        };

        enum LfCMDInfo
        {
            udpData = 0x00,		//udp数据接收端信息配置
            sync = 0x01,		//同步
            perid = 0x02,		//周期
            workMode = 0x05,    //工作模式
            startStop = 0x06,	//启动停止
            maxDist = 0x09,		//最大距离
            doa = 0x0B,			//doa模式
            doaFc = 0x0C,		//doa模式的fc设置
            power = 0x10,		//发射功率
            signal = 0x11,		//信号模式
            tvg = 0x12,			//内置TVG
            delay = 0x13,		//同步延迟
            acousticVelocity = 0x14,    //声速
            dataSource = 0x15,	//上传数据源
            winfunc = 0x16,		//内置窗函数
            tvgAdv = 0x17,		//高级tvg设置
            brightness = 0x18,	//亮度
            compassCalibration = 0x19,	//指南针校准
            distRes = 0x1A,     //距离分辨率
            angleRes = 0x1B,    //角度分辨率
            timeService = 0x1C, //向下授时
            dynImprove = 0x1D,	//动态改善
            pressZero = 0x1E,	//压力归0
            udpSend = 0x1F,		//udp发送控制
            phaseControlMode = 0x30,    //相控发射模式
            saveConfigInfo = 0x20,      //保存当前配置
            restoreFacSet = 0x21,       //恢复出厂设置
            workTimeSet = 0x23,		//启动后自动停止时间
            setDevDefaultIp = 0x24,	//设置设备默认ip
            rmDevLog = 0x25,		//清空设备log
            gamma = 0x26,           //伽马系数
            dhcpFlay = 0x27,
            threshold = 0x28,       //检测阀值
            paramInfo = 0x40,		//设备当前参数
            tcpKeepAlive = 0x4d,    //心跳检测
            versionInfo = 0x4E,		//版本信息

            //AESA
            AESAstartFrequency = 0x51,      //起始频率
            AESAscanWidth = 0x52,           //扫频宽度
            AESAoriginalPhase = 0x53,       //起始相位
            AESApulseWidth = 0x55,          //发射脉宽
            AESAtransmittedPower = 0x56,    //发射功率

            firN = 0x70,
        };

#pragma pack(1)
        struct s_sonarParamBymode {
            float range;
            uint8_t signalMode;
            uint8_t tvgMode;
            uint8_t windowMode;
            uint8_t trxPower;
            float tvgA, tvgB, tvgC;
            float rangeRes;
            float angleHor;
            float angleResV;
            uint8_t rsv[32];
        };

        struct s_sonarParam {
            uint8_t validFlag;
            uint32_t hostUdpAddr;
            uint16_t hostUdpPort;
            float prt;
            float sv;
            uint8_t dataSource;
            uint8_t syncSource;
            uint8_t sonarMode;
            uint8_t brightness;
            uint8_t compressLevel;
            uint8_t aesaMode;
            float aesaAngle;
            float syncDelay;
            uint8_t runStatus;
            uint8_t udpSendFlag;
            uint8_t rsv[225];
            s_sonarParamBymode paramBymode[8];
        };
#pragma pack()

        /**
        * @brief 构造函数
        * @param [in] sonarIP 声呐设备的IP
        * @param [in] sonarPort 声呐设备命令端口号
        */
        LhForwardCMD(string sonarIP, uint16_t sonarPort);
        ~LhForwardCMD();

        std::string getVersion();

        /**
        * @brief 连接设备函数
        */
        LhForwardCMD::LfCMDRet connectDevice();
        LhForwardCMD::LfCMDRet connectDevice(string inHostIp); // 绑定电脑特定ip 建立tcp 连接的函数

        /**
        * @brief 断开连接设备函数
        */
        void disconnectDevice();

        /**
        * @brief 发送命令函数
        * @param [in] info 命令号
        * @param [in] arg 命令参数
        * @param [out] reply 回复信息
        */
        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info);                 //发送无参数命令
        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info, uint8_t arg);    //发送参数是uint8类型命令
        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info, uint16_t arg);	//发送参数是uint16类型命令
        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info, float arg);      //发送参数是float类型命令

        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info, uint8_t arg, float arg1); //发送相控

        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info, float arg1, float arg2, float arg3);	//发送高级tvg命令
        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info, uint32_t arg);	//发送参数是uint32_t类型的命令
        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info, uint64_t arg);   //向下授时
        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info, void* reply);	//发送有回复信息的命令，目前只有版本信息

        LhForwardCMD::LfCMDRet AESAsendCMD(LfCMDInfo info, uint8_t arg, float arg1);    //AESA
        LhForwardCMD::LfCMDRet AESAsendCMD(LfCMDInfo info, uint8_t arg, uint8_t arg1);  //AESA发射功率

        /**
        * @brief 发送udp数据接收端信息配置
        * @param [in] info 命令号
        * @param [in] ip 接收声呐设备数据的ip地址 例如 ip[0]=192,ip[1]=168,ip[2]=1,ip[3]=83
        * @param [in] port 接收声呐设备数据的udp端口号
        */
        LhForwardCMD::LfCMDRet sendCMD(LfCMDInfo info, uint8_t ip[4], uint16_t port);			//发送udpdata命令

        /**
        * @brief cmd的socket获取
        */
        TYPE_SOCKET getCMDSocket() { return m_cmdSocket; }

    private:
        LfCMDRet sendAndRecvActive(char* cmd, int cmdSize);

        TYPE_SOCKET m_cmdSocket;
        string m_sonarIP;
        uint16_t m_sonarPort;
    };

}
