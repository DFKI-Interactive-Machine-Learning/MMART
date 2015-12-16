/*
 * This file is part of the CAR-NUI project.
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

#include "controller/Controller.hpp"

#include <iostream>

using boost::asio::ip::tcp;
using namespace nui::events;
using namespace nui::stream;

Controller::Controller() :
		m_stream_server(NULL), m_control_server(NULL) {
	m_startup = new timestamp_t(
			boost::posix_time::microsec_clock::local_time());
	reconfigure();
	nui::config()->registerForReconfigure("Controller",
			boost::bind(&Controller::reconfigure, this));
}

Controller::~Controller() {
	PtrRelease(m_control_server);
	PtrRelease(m_stream_server);
	PtrRelease(m_startup);
}

void Controller::reconfigure() {
	bool streaming = nui::config()->get("Streaming.Activated", true);
	bool cnt = nui::config()->get("Controller.Activated", true);
	int port = nui::config()->get("Streaming.Port", 50111);

	if (streaming) {
		if (m_stream_server) {
			m_stream_server->close();
			delete m_stream_server;
		}
		m_stream_server = new TCPStreamChannel(port);
		BOOST_LOG_TRIVIAL(info)<< "Streaming is enabled";
		BOOST_LOG_TRIVIAL(info)<< "Serving on port : " << I2Str(port);
	}
	if (cnt) {
		if (m_control_server) {
			m_control_server->close();
			delete m_control_server;
		}
		m_control_server = new TCPControlChannel(
				nui::config()->get("Controller.Port", 50112));
	}
}

void Controller::sendEvent(NUIEventPtr& evt) {
	HandPostureGestureEventPtr he;
	CircleGestureEventPtr ce;
	LeapTrackingEventPtr le;
	SwipeGestureEventPtr se;
	KeyTapGestureEventPtr ke;
	ScreenTapGestureEventPtr ste;
	ScreenPointerEventPtr spe;
	PinchGestureEventPtr pe;
	GrabGestureEventPtr ge;
	CamEventPtr came;
	LeapActionGestureEventPtr lae;
	SystemEventPtr sye;
	UserAttentionEventPtr uae;
	MyoEventPtr me;
	EventType type = evt.get()->eventType();
	switch (type) {
	case CIRCLE_GESTURE_EVENT:
		ce = NUIEVENTPTR_TO_CIRCLEGESTUREPTR(evt);
		this->sendCircleGesture(*ce);
		break;
	case SWIPE_GESTURE_EVENT:
		se = NUIEVENTPTR_TO_SWIPEGESTUREPTR(evt);
		this->sendSwipeGestureEvent(*se);
		break;
	case KEYTAP_GESTURE_EVENT:
		ke = NUIEVENTPTR_TO_KEYTAPGESTUREPTR(evt);
		this->sendKeyTapGestureEvent(*ke);
		break;
	case SCREEN_TAP_GESTURE_EVENT:
		ste = NUIEVENTPTR_TO_SCREENTAPGESTUREPTR(evt);
		this->sendScreenTapGestureEvent(*ste);
		break;
	case SCREEN_POINTER_EVENT:
		spe = NUIEVENTPTR_TO_SCREENPOINTERPTR(evt);
		this->sendScreenPointerEvent(*spe);
		break;
	case PINCH_EVENT:
		pe = NUIEVENTPTR_TO_PINCHGESTUREPTR(evt);
		this->sendPinchEvent(*pe);
		break;
	case GRAB_EVENT:
		ge = NUIEVENTPTR_TO_GRABGESTUREPTR(evt);
		this->sendGrabEvent(*ge);
		break;
	case LEAP_TRACKING_EVENT:
		le = NUIEVENTPTR_TO_LEAPTRACKINGPTR(evt);
		this->sendLeapTrackingEvent(*le);
		break;
	case LEAP_ACTION_GESTURE_EVENT:
		lae = NUIEVENTPTR_TO_LEAPACTIONEVENTPTR(evt);
		this->sendLeapActionEvent(*lae);
		break;
	case CAM_EVENT:
		came = NUIEVENTPTR_TO_WEBCAMPTR(evt);
		this->sendCamEvent(*came);
		break;
	case SYSTEM_EVENT:
		sye = NUIEVENTPTR_TO_SYSTEMEVENTPTR(evt);
		this->sendSystemEvent(*sye);
		break;
	case USER_ATTENTION_EVENT:
		uae = NUIEVENTPTR_TO_USERATTENTIONPTR(evt);
		this->sendUserAttentionEvent(*uae);
		break;
	case MYO_EVENT:
		me = NUIEVENTPTR_TO_MYOPTR(evt);
		this->sendMyoEvent(*me);
		break;
	}
}

void Controller::sendLeapTrackingEvent(LeapTrackingEvent& evt) {
	Frame f;
	leap2frame(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_LEAP_TRACKING, f);
}

void Controller::sendLeapActionEvent(LeapActionGestureEvent& evt) {
	Gesture f;
	action2gesture(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_LEAP_ACTION_GESTURE, f);
}

void Controller::sendCircleGesture(CircleGestureEvent& evt) {
	Gesture f;
	circle2gesture(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_LEAP_CIRCLE, f);
}

void Controller::sendPinchEvent(nui::events::PinchGestureEvent& evt) {
	Gesture f;
	pinch2gesture(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_LEAP_PINCH, f);
}

void Controller::sendGrabEvent(nui::events::GrabGestureEvent& evt) {
	Gesture f;
	grab2gesture(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_LEAP_GRAB, f);
}

void Controller::sendSwipeGestureEvent(SwipeGestureEvent& evt) {
	Gesture f;
	swipe2gesture(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_LEAP_SWIPE, f);
}

void Controller::sendMyoEvent(MyoEvent& evt) {
	MyoFrame f;
	myo2myoframe(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_MYO_EVENT, f);
}


void Controller::sendKeyTapGestureEvent(KeyTapGestureEvent& evt) {

}

void Controller::sendScreenTapGestureEvent(ScreenTapGestureEvent& evt) {
	Gesture f;
	screentap2gesture(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_LEAP_TAP, f);
}
void Controller::sendScreenPointerEvent(ScreenPointerEvent& evt) {

}

void Controller::sendCamEvent(CamEvent& evt) {

}
void Controller::sendSystemEvent(SystemEvent& evt) {
	SystemMessage f;
	system2system(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_SYSTEM_EVENT, f);

}

void Controller::sendUserAttentionEvent(UserAttentionEvent& evt) {
	UserAttention f;
	attention2attention(evt, &f);
	m_stream_server->send(TRANSMISSION_TYPE_ATTENTION_EVENT, f);

}
void Controller::open() {
	if (m_stream_server) {
		m_stream_server->open();
	}
	if (m_control_server) {
		m_control_server->open();
	}
}

void Controller::close() {
	if (m_stream_server) {
		m_stream_server->close();
	}
	if (m_control_server) {
		m_control_server->close();
	}
}
