#include "CircleBuffer.h"

CircleBuffer::CircleBuffer(uint16_t bufSize, uint16_t bufNum)
	: m_bufHeadPtr(new char[(uint64_t)bufSize * bufNum + 1])
	, m_bufEndPtr(m_bufHeadPtr + (uint64_t)bufSize * bufNum)
	, m_writePtr(m_bufHeadPtr)
	, m_readPtr(m_bufHeadPtr)
    , m_readPtr2(m_bufHeadPtr)
	, m_bufSize(bufSize)
	, m_bufNum(bufNum)
{
	memset(m_bufHeadPtr, 0, (uint64_t)bufSize * bufNum + 1);
}

CircleBuffer::~CircleBuffer()
{
	delete[]m_bufHeadPtr;
}

void CircleBuffer::writeData(const char* writeDataPtr, uint16_t dataSize)
{
	memcpy(m_writePtr, writeDataPtr, dataSize);
	std::lock_guard<std::mutex> lock(m_mutex);
	std::lock_guard<std::mutex> lock2(m_mutex2);
	m_writePtr += m_bufSize;
	if (m_writePtr == m_bufEndPtr) 
		m_writePtr = m_bufHeadPtr;
}

void CircleBuffer::readData(uint16_t dataSize, char* readDataPtr)
{
	memcpy(readDataPtr, m_readPtr, dataSize);
	std::lock_guard<std::mutex> lock(m_mutex);
	m_readPtr += m_bufSize;
	if (m_readPtr == m_bufEndPtr)
    {
		m_readPtr = m_bufHeadPtr;
    }
}

char* CircleBuffer::readDataAddr()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	char* dataPosPtr = m_readPtr;
	m_readPtr += m_bufSize;
	if (m_readPtr == m_bufEndPtr)
		m_readPtr = m_bufHeadPtr;
	return dataPosPtr;
}

char* CircleBuffer::readDataAddr2()
{
	std::lock_guard<std::mutex> lock(m_mutex2);
	char* dataPosPtr = m_readPtr2;
    m_readPtr2 += m_bufSize;
    if (m_readPtr2 == m_bufEndPtr) 
		m_readPtr2 = m_bufHeadPtr;
	return dataPosPtr;
}

bool CircleBuffer::isAnyData()
{
	bool haveData;
	std::lock_guard<std::mutex> lock(m_mutex);
	haveData = !(m_writePtr == m_readPtr);
	return haveData;
}

bool CircleBuffer::isAnyData2()
{
    bool haveData;
	std::lock_guard<std::mutex> lock(m_mutex2);
    haveData = !(m_writePtr == m_readPtr2);
    return haveData;
}

void CircleBuffer::setRead2Pos()
{
    std::lock_guard<std::mutex> lock(m_mutex2);
    m_readPtr2 = m_writePtr;
}
