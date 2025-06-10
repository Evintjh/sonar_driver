/*!
* @file CircleBuffer.h
* @brief 循环缓冲实现
* @details
* @author liuhai
* @date 2019/11/27 12:38
*/
#pragma once

#include <cstdint>
#include <string>
#include <string.h>
#include <iostream>
#include <mutex>

class CircleBuffer
{
public:
	/**
	* @brief 循环缓冲类构造函数
	* @param [in] bufSize 每个缓冲的大小/字节
	* @param [in] bufNum 缓冲个数
	*/
	CircleBuffer(uint16_t bufSize, uint16_t bufNum);
	~CircleBuffer();

	/**
	* @brief 写入循环缓冲
	* @param [in] writeDataPtr 待写入的数据指针
	* @param [in] dataSize 写入数据大小/字节
	*/
	void writeData(const char* writeDataPtr, uint16_t dataSize);
	
	/**
	* @brief 读出循环缓冲
	* @param [in] dataSize 读出数据大小/字节
	* @param [out] readDataPtr 读出位置指针
	*/
	void readData(uint16_t dataSize, char* readDataPtr);

	/**
	* @brief 读出循环缓冲
	* @return 返回本次读取数据的地址
	*/
	char* readDataAddr();
	
    /**
     * @brief 文件保存读出缓冲
     * @return 返回读取地址
     */
    char* readDataAddr2();
	/**
	* @brief 判断当前循环缓冲是否有未读出的数据
	* @return true:有未读出数据 false:没有
	*/   
	bool isAnyData();
    bool isAnyData2();
    /**
    * @brief 设置当前读取的位置
    */
    void setRead2Pos();

private:
	char* m_bufHeadPtr;
	char* m_bufEndPtr;
	char* m_writePtr;
	char* m_readPtr;
    char* m_readPtr2;
	const uint16_t m_bufSize;
	const uint16_t m_bufNum;
	std::mutex m_mutex;
	std::mutex m_mutex2;
};
