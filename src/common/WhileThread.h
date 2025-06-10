/*!
* @file WhileThread.h
* @brief 基于C++的while循环式线程抽象类
* @details
* @author liuhai
* @date 2020/3/5 13:46
*/

#pragma once

#include <thread>
#include <atomic>

class WhileThread
{
public:
	WhileThread()
		: m_thread(nullptr)
		, m_runFlag(false)
	{};
	virtual ~WhileThread()
	{
		m_runFlag = false;
		if (m_thread != nullptr)
		{
			m_thread->join();
			delete m_thread;
			m_thread = nullptr;
		}
	}

	WhileThread(const WhileThread&) = delete;
	WhileThread& operator=(const WhileThread&) = delete;

	virtual void start()
	{
		m_runFlag = true;
		m_thread = new std::thread(&WhileThread::threadFunc, this);
	};

	virtual void stop()
	{
		m_runFlag = false;
		if (m_thread != nullptr)
		{
			m_thread->join();
			delete m_thread;
			m_thread = nullptr;
		}
	}

protected:
	virtual void threadFunc() = 0;

	std::thread* m_thread;
	std::atomic<bool> m_runFlag;
};
