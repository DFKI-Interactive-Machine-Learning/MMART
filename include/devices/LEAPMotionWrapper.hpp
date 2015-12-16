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
#include "eventmanager/Modality.hpp"
#include "Leap.h"

/**
 * \brief Interface for hand tracking of LEAP Motion device.
 */
class LEAPMOTION_EXPORT LEAPMotionWrapper: public IModality {
public:
	/** Enum with native supported gestures. */
	typedef enum _LEAP_GESTURES {
		CIRCLE_GESTURE = 0x1,
		SWIPE_GESTURE = (CIRCLE_GESTURE << 1),
		KEY_TAP_GESTURE = (SWIPE_GESTURE << 1),
		SCREEN_TAP_GESTURE = (KEY_TAP_GESTURE << 1),
	} LeapGesture;

	LEAPMotionWrapper() :
			IModality(LEAP_MOTION_MODALITY, SAMPLING_50_HZ), m_gesture_setup(0x0) {
		reconfigure();
	};
	virtual ~LEAPMotionWrapper() {

	};
protected:
	/** ----------- IModality virtual function ------------- **/
	virtual void reconfigure();
	virtual void process(EventQueue&);
	virtual NUISTATUS initialize();
	virtual NUISTATUS unInitialize();
	/** ----------- Leap Callback functions ------------- **/
private:
	/** Type to store information about the gesture setup. */
	typedef boost::uint8_t GestureSetup;
	/** Leap controller */
	Leap::Controller m_controller;
	/** Gesture setup. */
	GestureSetup m_gesture_setup;
	/**
	 * Stream raw tracking data.
	 */
	bool m_stream_raw;
	/**
	 * Flag if streams should be streamed.
	 */
	bool m_stream_gestures;
	/** 
	 * \brief Check function if gesture is active.
	 * \param gesture config
	 * \param type of the gesture
	 */
	bool
	isActive(GestureSetup&, LeapGesture);
	/** 
	 * \brief Functions to process gestures.
	 * \param list of gestures
	 */
	void
	processGestures(EventQueue&, const Leap::GestureList&);
	/** 
	 * \brief Functions to process hands.
	 * \param queue of events
	 * \param list of hands
	 */
	void
	processHands(EventQueue&, const Leap::HandList&);
	/** 
	 * \brief Functions to process calibrated screen
	 * \param queue of events
	 * \param list of calibrated screens
	 * \param list of pointables
	 */
	void
	processCalibratedScreens(EventQueue&, const Leap::ScreenList&,
			const Leap::PointableList&);

	/**
	 * \brief No hand tracked event.
	 */
	void createNoneEvent(EventQueue&);
};
