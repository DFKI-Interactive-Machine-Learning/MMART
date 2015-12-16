/*
 * This file is part of the AV-NUI project.
 * Copyright (C) 2012 DFKI GmbH. All rights reserved.
 *
 * Disclaimer:
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND
 * CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include "core/common.hpp"
#include "core/errorcodes.hpp"
#include "core/NUIDLLexport.hpp"
#include "eventmanager/Events.hpp"

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <queue>

/*************  Callback functions *******************************************/
#include <boost/function.hpp>
/*
 * Callback Functions:
 * -------------------
 *
 *  EventFunc:
 *  
 * [Parameters] 
 *   - event (NUIEvent) - Event triggered modality
 */
typedef boost::function1<void, const nui::events::NUIEventPtr> EventFunc;
typedef std::queue<nui::events::NUIEvent*> EventQueue;

/**
 * Sampling Rates
 */
const executiontime_t SAMPLING_100_HZ = 10;
const executiontime_t SAMPLING_50_HZ  = 20; 
const executiontime_t SAMPLING_30_HZ  = 33;

/**
 * \brief Current run-time state of the modality.
 */
enum RunTimeState {
	NOT_INITIALIZED = 0x0,
	INITIALIZED = 0x1,
	RUNNING = 0x2,
	PAUSED = 0x3,
	STOPPED = 0x4,
	WAITING = 0x5
};

enum ModalityMode {
	NONBLOCKING = 0x0,
	BLOCKING    = 0x1
};

/**
 * \brief Interface for input modalities. 
 *
 * Each modality is running in its own thread having a maximum exectution time of each cycle. 
 * Furthermore, the modality is a defined state.
 */
class COMMON_EXPORT IModality {
public:
	/**
	 * \brief Constructor for modality
	 *
	 * \param[in] name of modality 
	 * \param[in] maximum time needed for execution (default=0)
	 */
	IModality(const std::string name, executiontime_t time=0, ModalityMode mode=NONBLOCKING) :
			m_name(name), m_executiontime(time), m_state(NOT_INITIALIZED),
			m_waiting(true), m_updates(0), m_thread(NULL), m_mode(mode) {
	};

	/**
	 * \brief Destructor.
	 */
	virtual ~IModality() {
		if (m_state == RUNNING || m_state == PAUSED) // Abort
		{
			stop();
		}
		PtrRelease(m_thread);
	};

	/**
	 * \brief Reconfigure modality.
	 */
	virtual void
	reconfigure() = 0;

	/**
	 * \brief State of the modality.
	 * \return current state of modality
	 */
	const RunTimeState state() const {
		return m_state;
	}

	/**
	 * \brief Mode of the modality.
	 * \return current mode of modality
	 */
	const ModalityMode mode() const {
		return m_mode;
	}

	/**
	 * \brief Reconfigure modality
	 */
	NUISTATUS start();

	/**
	 * \brief Pause modality
	 */
	NUISTATUS
	pause();

	/**
	 * \brief Resume modality
	 */
	NUISTATUS
	resume();

	/**
	 * \brief stop modality
	 */
	NUISTATUS
	stop();
	/**
	 * \brief Connects callback function
	 */
	void
	connectListener(const EventFunc listener)
	{
		event_sig = listener;
	};
	/**
	 * \brief Number of updates so far.
	 */
	const long updates() const
	{
		return m_updates;
	}
	/**
	 * \brief Average frames per second.
	 */
	const float fps() const;
	/**
	 * \brief Name of modality
	 */
	const std::string
	name() const
	{
		return m_name;
	};

protected:
	/**
	 * \brief Called when modality starts.
	 */
	virtual NUISTATUS
	onStart()
	{
		return NUI_SUCCESS;
	};
	/**
	 * \brief Called when modality stops.
	 */
	virtual NUISTATUS
	onStop()
	{
		return NUI_SUCCESS;
	};
	/**
	 * \brief Initialize modality.
	 */
	virtual NUISTATUS
	initialize()
	{
		return NUI_SUCCESS;
	};

	/**
	 * \brief Unintialize modality.
	 */
	virtual NUISTATUS
	unInitialize()
	{
		return NUI_SUCCESS;
	};
	/** 
	 * \brief NUI updates.
	 */
	unsigned long m_updates;
	/**
	 * \brief Starting time.
	 */
	timestamp_t m_start;
	/**
	 * \brief State of the modality.
	 */
	RunTimeState m_state;
	/**
	 * \brief Mode of the modality.
	 */
	ModalityMode m_mode;
	/**
	 * \brief Process method is called in the thread.
	 */
	virtual void
	process(EventQueue&) {};
	/**
	 * \brief Process method is called in the thread.
	 */
	virtual void
	process() {};
	/**
	 * \brief Fires a nui event.
	 */
	void
	fireEvent(nui::events::NUIEventPtr evt)
	{
		event_sig(evt);
	}
	/**
	 * \brief Mutex for state locks
	 */
	boost::mutex m_state_mutex;
private:
	/**
	 * \brief Signal to trigger events
	 */
	EventFunc event_sig;
	/**
	 * \brief Pointer to thread.
	 */
	boost::thread* m_thread;
	/**
	 * \brief Mutex for locks 
	 */
	boost::mutex m_mutex;
	/** 
	 * \brief Waiting flag. 
	 */
	bool m_waiting;
	/** 
	 * \brief Waiting condition variable. 
	 */
	boost::condition_variable m_wait_condition_variable;
	/**
	 * \brief Maximum time to update the modality, measured in milliseconds.
	 */
	executiontime_t m_executiontime;
	/**
	 * \brief Name of the modality 
	 */
	std::string m_name;
	/**
	 * \brief Event queue
	 */
	EventQueue m_eventqueue;
	/**
	 * \brief Pauses the thread.
	 */
	void
	wait();
	/**
	 * \brief Wakes up the thread.
	 */
	void
	wakeup();
	/*
	 * Run
	 */
	void
	run();
};
