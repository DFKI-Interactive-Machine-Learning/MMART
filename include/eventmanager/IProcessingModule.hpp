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
#include "core/NUIDLLexport.hpp"
#include "core/TripleBuffer.hxx"

#include "eventmanager/Modality.hpp"
#include "eventmanager/Events.hpp"

enum ProcessingyMode {
	LASTEST_UPDATE = 0x0,
	QUEUEING = 0x1
};

/**
 * \brief Listerner Interface for nuievents
 * Each listener is running in its own thread.
 */
class COMMON_EXPORT IProcessingModule {
public:
	IProcessingModule(std::string name, ProcessingyMode mode = LASTEST_UPDATE) :
			m_state(NOT_INITIALIZED), m_name(name), m_updates(0), m_milliseconds(0), m_mode (mode), m_buffer() {
	}
	virtual ~IProcessingModule() {};

	/**
	 * \brief Reconfigure listener.
	 */
	virtual void reconfigure() {
	}

	/**
	 * \brief Registers the listener callback event function.
	 * Whenever the event listener analysis triggers an event the callback function will be called.
	 */
	void registerListener(EventFunc listener) {
		listenerFunc = listener;
	}

	/**
	 * \brief Starts the listener in an own thread.
	 */
	NUISTATUS start() {
		m_thread = boost::shared_ptr<boost::thread>(
				new boost::thread(boost::bind(&IProcessingModule::run, this)));
		m_updates = 0;
		m_milliseconds = 0;
		return NUI_SUCCESS;
	}

	/**
	 * \brief Stop listner
	 */
	NUISTATUS stop();

	/**
	 * \brief Updates the current posture.
	 */
	void update(const nui::events::NUIEventPtr evt);

	executiontime_t averageExecutiontime() {
		if (m_updates == 0) return 0;
		return m_milliseconds / m_updates;
	}

	/**
	 * \brief Name of listener.
	 */
	const std::string name() const {
		return m_name;
	}

protected:
	EventFunc listenerFunc;

	/**
	 *	Current state of listener.
	 */
	RunTimeState m_state;

	/**
	*	Current mode of processing module.
	*/
	ProcessingyMode m_mode;
	/**
	 * \brief Initialize posture listener.
	 */
	virtual NUISTATUS initialize() {
		m_state = INITIALIZED;
		return NUI_SUCCESS;
	}

	/**
	 * \brief Unintialize posture listener.
	 */
	virtual NUISTATUS unInitialize() {
		return NUI_SUCCESS;
	}

	/**
	 * \brief Fires a nui event.
	 */
	void fireEvent(nui::events::NUIEvent* evt) {
		if(!listenerFunc.empty())
		{
			listenerFunc(NUIEVENT_PTR(evt));
		}
	}
	virtual void process(nui::events::LeapActionGestureEvent&) {};
	virtual void process(nui::events::HandPostureGestureEvent&) {};
	virtual void process(nui::events::CircleGestureEvent&)      {};
	virtual void process(nui::events::LeapTrackingEvent&)       {};
	virtual void process(nui::events::SwipeGestureEvent&)       {};
	virtual void process(nui::events::KeyTapGestureEvent&)      {};
	virtual void process(nui::events::ScreenTapGestureEvent&)   {};
	virtual void process(nui::events::ScreenPointerEvent&)      {};
	virtual void process(nui::events::CamEvent&)                {};
    virtual void process(nui::events::HeadTrackingEvent&)       {};
	virtual void process(nui::events::UserAttentionEvent&)      {};
    virtual void process(nui::events::PMDNanoEvent&)            {};
    virtual void process(nui::events::Kinectv2Event&)            {};

private:
	/** \brief Counting the updates of the listener. */
	size_t m_updates;

	/** \brief Name of the Body posture listener */
	std::string m_name;

	/** \brief Execution time */
	executiontime_t m_milliseconds;

	/** \brief Pointer to thread. */
	boost::shared_ptr<boost::thread> m_thread;
	/**
	 * Non-blocking buffer.
	 */
	TripleBuffer<nui::events::NUIEvent> m_buffer;
	/**
	* \brief Mutex for buffer locks
	*/
	boost::mutex m_buffer_mutex;
	/**
	* \brief Mutex for locks
	*/
	boost::mutex m_mutex;

	/** \brief Waiting condition */
	boost::condition_variable m_wait_condition_variable;

	/** \brief Checks if the event is valid. */
	/*virtual bool check(BodyPostureEvent& evt) {
		return true;
	}*/

	void wait();

	void wakeup();

	/** \brief run() */
	void run();
};
