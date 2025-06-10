/*!
* Copyright (C) 珠海蓝衡科技有限公司
* All rights reserved
* @File LhForwardImage.h
* @Author liuhai
* @Brief 前视声呐成像库头文件
* @Details 数据预处理，图像生成，双线性插值
* @LastEditors liuhai
* @LastEditTime 2020/3/31 11:35
* @Version 2.0
*/

#pragma once
#include <stdint.h>
#include "LhForwardFrame.h"

#ifdef USE_WINDOWS
#define DLL_API _declspec(dllexport)
#else
#define DLL_API
#endif // !DLL_API

namespace LhForwardSDK
{
	class DLL_API LhForwardImage
	{
	public:
        LhForwardImage(int width = 800, int height = 600);
		~LhForwardImage();

		enum DataWidth
		{
			bit8 = 0x00,
			bit16 = 0x01,
			bit32 = 0x02,
			bit64 = 0x03,
			complexBit16 = 0x10,
			complexBit32 = 0x11,
			complexBit64 = 0x12,
			complexBit128 = 0x13,
			hfType = 0x20,
			floatType = 0x21,
			doubleType = 0x22,
		};

		/**
        * @brief 数据预处理,进行数据类型转换
		* @param [in] frameInfo 数据帧信息
		* @param [in] dataPtr 转换前数据
		* @param [out] intensityPtr 转换后的数据
		*/
		void preprocessData(const packageInfo& frameInfo, const uint8_t* dataPtr, double* intensityPtr);

    	/**
		* @brief 极坐标数据图像生成
		* @param [in] frameInfo 帧信息
		* @param [in] dataPtr 帧数据
		* @param [out] sectorMemoryPtr 插值后的内存数据
		* @param [out] imageRes	图像分辨率 米/像素
		* @note sectorMemoryPtr参数对应的内存大小为width*height，即图像数据内存大小
		*       imageRes图像分辨率：物理尺寸下的扇形图弦长/(生成的图像宽度*0.95)
		*/
		void generateForwardImage(const packageInfo& frameInfo, const uint8_t* dataPtr, uint8_t* sectorMemPtr, double& imageRes);

		/*
		* @brief 色板数据
		* @param [out] colorTablePtr 色板指针
		* @note 内部做数据拷贝，初始化时调用一次即可
		*/
		void getColorTable(char* colorTable);

#ifdef HAVE_LIBPNG // 如果定义了这个宏代表需要使用libpng 库
        // 外部需要使用，暂设为public
        /**
        * @brief png数据解码处理
        */
        int decodePng(const packageInfo& frameInfo, const uint8_t* dataPtr, int& width, int& height, uint8_t* dataOut);
#endif

    private:
		/**
		* @brief 双线性插值，计算扇形图内存
		* @param [in] dataPtr 插值前的帧数据
		* @param [in] dataWidth 数据位宽
		* @param [out] sectorMemoryPtr 插值后的内存数据
		* @note
		*/
		void generateImageMem(const uint8_t* dataPtr, uint8_t dataWidth, uint8_t* sectorMemPtr);

		/**
		* @brief 双线性插值参数计算
		* @param [in] numPerRow 每行点数
		* @param [in] numInDist 距离向点数，即每包行数 x 每帧包数
		* @param [in] maxRange 最大作用距离
		* @param [in] distRes 距离分辨率
		* @param [in] horAngleRes 水平角度分辨率
		* @param [out] imageRes	图像分辨率 米/像素
		* @note
		*/
        void generateParameter(int numPerRow, int numInDist, double maxRange, double distReso, double horAngleReso, double& imageRes);

		void bilinerInp360Cal(const uint8_t* dataPtr, uint8_t dataWidth, int numInDist, double maxDist, int numPerRow, double distReso, uint8_t* sectorMemPtr, double horAngleReso, double& imageRes);

		void bilinerInp360GenerateMap();

	private:
		// 数据参数
		int m_numPerRow = 0;		//每行点数，即成像波束数
		int m_numInDist = 0;		//距离向点数，即每包行数x每帧包数
		double m_distReso = 0;		//距离分辨率
		double m_horAngleReso = 0;	//水平角度分辨率 /度
		double m_maxDist = 0;		//最大作用距离
		// 图像参数
		int	m_width;	//成像后图像宽度 /像素
		int m_height;	//成像后图像高度 /像素
		double m_imageRes;	//成像图像分辨率 米/像素
		int m_sectorImgSize;
		// 双线性插值系数
		int* m_channelCount;
		int* m_sampleCount;
		double* m_dy;
		double* m_dx;
		// 色板
		char m_colorTable[256][3];
		void generateColorTable();
	};

}
