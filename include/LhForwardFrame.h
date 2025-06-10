/*!
* Copyright (C) 珠海蓝衡科技有限公司
* All rights reserved
* @File LhForwardFrame.h
* @Author liuhai
* @Brief 前视声呐数据解析库头文件
* @Details 数据解析与拼接
* @LastEditors liuhai
* @LastEditTime 2020/4/17 10:17
* @Version 3.3
*/

#pragma once
#include <cstring>
#include <stdint.h>
#include <map>
#include <string>

#ifdef USE_WINDOWS
#ifdef DLL_EXPORTS
#define DLL_API _declspec(dllexport)
#else
#define DLL_API _declspec(dllimport)
#endif
#else
#define DLL_API
#endif // !DLL_API

namespace LhForwardSDK
{
	const uint16_t PACK_INFO_SIZE = 256;  //包头信息长度
	const uint16_t PACK_DATA_INDEX = 256; //每包数据索引
	//const uint16_t PACK_SIZE = 5800;      //每包数据总字节数
	const uint16_t PACK_MAX_NUM = 12000;  //每帧数据最大包数

#pragma pack(1)
	struct DLL_API packageInfo
	{
        uint8_t packType = 0;           //包类型
        uint8_t dataWidth = 0;          //数据位宽
        uint32_t packTime1 = 0;         //时间戳1（us）
        uint16_t packTime2 = 0;         //时间戳2（us）
        uint16_t numPerRow = 0;         //每行点数
        uint16_t numPerCol = 0;         //每列点数
        uint16_t rowNumPerPack = 0;     //每包行数
        uint16_t rowSerialNum = 0;      //行序号
        uint16_t packSerialNum = 0;     //包序号
        uint16_t packNumPerFrame = 0;   //每帧包数
        uint32_t frameSerialNum = 0;    //帧序号
        uint8_t reserve1[8] = {0};      //保留
        char deviceName[15];            //设备名称
        uint8_t workMode = 0;           //工作模式
        float acousticVelocity = 0;		//声速（m/s）
        uint32_t dataHeight = 0;	//距离向点数
        float maxDist = 0;		    //量程（m）
        float distReso = 0;			//距离分辨率（m）
        float horAngleReso = 0;		//水平角度分辨率/水平开角（度）
		float verAngleReso = 0;		//垂直角度分辨率
		float longitude = 0;		//经度
		float latitude = 0;			//纬度
        float altitude = 0;			//高度
		float status = 0;			//状态
		float yaw = 0;				//偏航角
		float pitch = 0;			//俯仰角
		float roll = 0;				//横滚角
		float speed = 0;			//速度
		float direction = 0;		//航向
		uint8_t syncModel = 0;		//同步模式
        float workCycle = 0;		//工作周期（s）
		uint8_t power = 0;			//发射功率
		uint8_t sigModel = 0;		//信号模式
		uint8_t TVGSet = 0;			//TVG设置
        uint32_t syncDelay = 0;		//同步延时（us）
		uint8_t windowFunc = 0;		//窗函数
        float TVGA = 0;             //TVG高级设置A
        float TVGB = 0;             //TVG高级设置B
        float TVGC = 0;             //TVG高级设置C
		uint8_t brightness = 0;		//亮度
		uint8_t dynImprove = 0;		//动态改善
        uint8_t blank = 0;          //保留
        float press = 0;            //压力
        float temp1 = 0;            //外部温度
        float temp2 = 0;            //内部温度
        uint64_t utcTime = 0;       //UTC时间
        uint8_t reservel[10] = {0}; //保留
        uint8_t versionMajor = 0;   //协议主版本号
        uint8_t versionMinor = 0;   //协议副版本号
        float gamma = 0;            //伽马系数
        uint8_t reserve2[4] = {0};  //保留
        float doaFreq = 0;          //doa最大频率
        float doaFregValue = 0;     //doa频率峰均比
        float doa = 0;              //doa角度结果
        float doaValue = 0;         //doa角度峰均比
        uint8_t rsv[64] = {0};		//保留
	};
#pragma pack()

	class DLL_API LhForwardFrame
	{
	public:
		enum LfFrameRet
		{
			writePackOK = 0,		//正确写入一包数据
			writePackErr = -1,		//写入错误，包头信息不正确
			oneFrameOK = 1,			//一帧数据写入完成
		};

        static uint16_t PACK_SIZE;

		LhForwardFrame(uint16_t packSize = 5800);
		~LhForwardFrame();

        std::string getVersion();

		/**
		* @brief 写入帧缓冲
		* @param [in] data 待写入的数据
		* @return 返回写入状态，枚举LfFameRet
		*/
		int writeOnePackData(const uint8_t* data);

		/**
		* @brief 获取一帧数据
		* @param [out] packInfo 包头信息
		* @param [out] dataBuf 数据信息，需要开辟足够内存
		* @note writeOnePackData返回oneFrameOK后，马上调用，取出数据
		*/
		void getOneFrame(struct packageInfo* packInfo, uint8_t* dataBuf);

		/**
		* @brief 初始化
		* @note
		*/
		void init();

	private:
		void swapPingPang();

	private:
		struct packageInfo m_packInfoPing;
		struct packageInfo m_packInfoPang;
		struct packageInfo* m_curPackInfo;
		uint8_t* m_frameBufPing;
		uint8_t* m_frameBufPang;
		uint8_t* m_curFrameBuf;
		bool m_pingWritingFlag;
		std::map<int, int> m_dataWidthSize; // 不同位宽对应数据大小
	};
}
