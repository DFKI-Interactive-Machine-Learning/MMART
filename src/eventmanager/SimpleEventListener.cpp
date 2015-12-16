#include "eventmanager/SimpleEventListener.hpp"
#include <boost/log/trivial.hpp>

using namespace nui::events;

void ISimpleEventListener::update(const NUIEventPtr evt) {
	m_updates += 1;
	timestamp_t t1(boost::posix_time::microsec_clock::universal_time());
	EventType type = evt.get()->eventType();
	switch (type) {
	case HAND_GESTURE_EVENT:
		this->process(*NUIEVENTPTR_TO_HANDGESTUREPTR(evt));
		break;
	case CIRCLE_GESTURE_EVENT:
		this->process(*NUIEVENTPTR_TO_CIRCLEGESTUREPTR(evt));
		break;
	case SWIPE_GESTURE_EVENT:
		this->process(*NUIEVENTPTR_TO_SWIPEGESTUREPTR(evt));
		break;
	case KEYTAP_GESTURE_EVENT:
		this->process(*NUIEVENTPTR_TO_KEYTAPGESTUREPTR(evt));
		break;
	case SCREEN_TAP_GESTURE_EVENT:
		this->process(*NUIEVENTPTR_TO_SCREENTAPGESTUREPTR(evt));
		break;
	case SCREEN_POINTER_EVENT:
		this->process(*NUIEVENTPTR_TO_SCREENPOINTERPTR(evt));
		break;
	case LEAP_TRACKING_EVENT:
		this->process(*NUIEVENTPTR_TO_LEAPTRACKINGPTR(evt));
		break;
	case MYO_EVENT:
		this->process(*NUIEVENTPTR_TO_MYOPTR(evt));
		break;
	case CAM_EVENT:
		this->process(*NUIEVENTPTR_TO_WEBCAMPTR(evt));
		break;
	case USER_ATTENTION_EVENT:
		this->process(*NUIEVENTPTR_TO_USERATTENTIONPTR(evt));
		break;
	case PMDNANO_EVENT:
		this->process(*NUIEVENTPTR_TO_PMDNANOPTR(evt));
		break;
	case REALSENSE_EVENT:
		this->process(*NUIEVENTPTR_TO_REALSENSEPTR(evt));
		break;
	case DEPTHSENSE_EVENT:
		this->process(*NUIEVENTPTR_TO_DEPTHSENSEPTR(evt));
		break;
	case KINECTV2_EVENT:
		this->process(*NUIEVENTPTR_TO_KINECTV2PTR(evt));
		break;
	}
	timestamp_t t2(boost::posix_time::microsec_clock::universal_time());
	duration_t exe_time = t2 - t1;
	m_milliseconds += exe_time.total_milliseconds();
}

NUISTATUS ISimpleEventListener::stop() {
	m_buffer_mutex.lock(); // prevent any new updates
	m_state = STOPPED;     // set state to STOPPED
	m_wait_condition_variable.notify_one(); // wake up
	if (m_thread) m_thread.get()->join();
	m_buffer_mutex.unlock();
	BOOST_LOG_TRIVIAL(info) << name() << " stopped. [AVERAGE PROCESSING TIME] : " << I2Str(averageExecutiontime()) << " ms "
		<< " [UPDATES] : " << I2Str(m_updates);
	return unInitialize();
}

void ISimpleEventListener::run() {
	if (!m_state == INITIALIZED) {
		NUISTATUS r = initialize();
		if (NUI_FAILED(r)) {
			BOOST_LOG_TRIVIAL(error) << "Initialization of " << m_name << " failed.";
			return;
		}
		m_state = RUNNING;
	}
}

void ISimpleEventListener::wait() {
	boost::unique_lock<boost::mutex> lock(m_mutex);
	m_state = WAITING;
	while (m_state == WAITING) {
		m_wait_condition_variable.wait(lock);
	}
}

void ISimpleEventListener::wakeup() {
	m_state = RUNNING;
	m_wait_condition_variable.notify_one();
}
