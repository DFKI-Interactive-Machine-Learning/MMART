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
#include "controller/ControlChannel.hpp"
#include "controller/TCPChannel.hpp"
#include "core/common.hpp"
#include "core/NUIDLLexport.hpp"
#include "eventmanager/Events.hpp"

#include <boost/thread.hpp>
#include <queue>


/**
 * \brief Controller for Natural user interface.
 *
 *	Controller is responsible for the communication 
 */
class EVENTMANAGER_EXPORT Controller {
public:
	explicit Controller();

	~Controller();

	/**
	 * \brief Reconfigures the settings of the controller.
	 * \parameter NUIConfig configuration of application
	 */
	void reconfigure();
	/**
	 * \brief Received command register function for callback.
	 * \param[in] function pointer
	 */
	void RegisterReceivedCommandFunc(ReceivedCommandFunc func) {
		m_control_server->RegisterReceivedCommandFunc(func);
	}

	/**
	 * \brief Sends event.
	 */
	void sendEvent(nui::events::NUIEventPtr&);

	/**
	 * \brief Sends Leap motion tracking event.
	 */
	void sendLeapTrackingEvent(nui::events::LeapTrackingEvent&);

	/**
	 * \brief Sends Leap motion tracking event.
	 */
	void sendLeapActionEvent(nui::events::LeapActionGestureEvent&);

	/**
	 * \brief Sends specific Leap Motion gesture to application.
	 * \parameter Leap Motion event
	 */
	void sendCircleGesture(nui::events::CircleGestureEvent&);

	/**
	 * \brief Sends specific Leap Motion gesture to application.
	 * \parameter Leap Motion event
	 */
	void sendSwipeGestureEvent(nui::events::SwipeGestureEvent&);

	/**
	 * \brief Sends specific Leap Motion gesture to application.
	 * \parameter Leap Motion event
	 */
	void sendKeyTapGestureEvent(nui::events::KeyTapGestureEvent&);

	/**
	 * \brief Sends specific Leap Motion gesture to application.
	 * \parameter Leap Motion event
	 */
	void sendScreenTapGestureEvent(nui::events::ScreenTapGestureEvent&);

	/**
	 * \brief Sends specific Leap Motion pointer event.
	 * \parameter Leap Motion pointer event
	 */
	void sendScreenPointerEvent(nui::events::ScreenPointerEvent& evt);

	/**
	 * \brief Sends specific Leap Motion pointer event.
	 * \parameter Leap Motion pointer event
	 */
	void sendPinchEvent(nui::events::PinchGestureEvent& evt);

	/**
	 * \brief Sends specific Leap Motion pointer event.
	 * \parameter Leap Motion pointer event
	 */
	void sendGrabEvent(nui::events::GrabGestureEvent& evt);

	/**
	 * \brief Sends specific Leap Motion pointer event.
	 * \parameter Leap Motion pointer event
	 */
	void sendCamEvent(nui::events::CamEvent& evt);

	/**
	* \brief Sends specific Myo pointer event.
	* \parameter Myo pointer event
	*/
	void sendMyoEvent(nui::events::MyoEvent& evt);

	/**
	 * \brief Sends system events.
	 * \parameter Leap Motion pointer event
	 */
	void sendSystemEvent(nui::events::SystemEvent& evt);

	/**
	 * \brief Sends user attention.
	 * \parameter Leap Motion pointer event
	 */
	void sendUserAttentionEvent(nui::events::UserAttentionEvent& evt);

	/**
	 * \brief Opens the connection with the application.
	 */
	void open();

	/**
	 * \brief Closes the connection with the application.
	 */
	void close();

private:
	timestamp_t* m_startup;

	/** \brief Configured streaming server format. */
	std::string streaming_format;

	/** \brief Control channel for body tracker. */
	StreamChannel* m_stream_server;

	/** \brief Feedback channel. */
	ControlChannel* m_control_server;

	bool m_stream_position;
};
