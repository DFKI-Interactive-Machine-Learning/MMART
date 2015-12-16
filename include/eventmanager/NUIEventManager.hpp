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

#include "controller/Controller.hpp"
#include "devices/Webcam.hpp"
#ifdef WITH_DEVICE_PMD_NANO
	#include "devices/PMDNano.hpp"
#endif
#if WITH_DEVICE_KINECT_V2
	#include "devices/KinectV2.hpp"
#endif
#if WITH_DEVICE_INTEL_REALSENSE
	#include "devices/IntelRealSenseCamera.hpp"
#endif
#if WITH_DEVICE_DEPTHSENSE
	#include "devices/DepthSenseCamera.hpp"
#endif
#if WITH_DEVICE_LEAP_MOTION
	#include "devices/LEAPMotionWrapper.hpp"
#endif
#if WITH_DEVICE_MYO
	#include "devices/MyoWrapper.hpp"
#endif
#if WITH_GESTURE_RECOGNITION
	#include "GestureRecognition.hpp"
	#include "HandGestureRecognition.hpp"
	#include "ModelClassifier.hpp"
#endif
#include "eventmanager/SimpleEventListener.hpp"
#include "eventmanager/IProcessingModule.hpp"
#include "eventmanager/Modality.hpp"
#include "eventmanager/Events.hpp"

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/ptr_container/ptr_deque.hpp>
#include <boost/signals2.hpp>
#include <vector>

/* ------------------------ TYPES ----------------------------------*/
typedef std::vector<IModality*> ModalityVector;
typedef std::vector<nui::events::IFeedbackListener*> FeedBackListeners;
typedef std::vector<ISimpleEventListener*> EventListeners;
typedef std::vector<IProcessingModule*> ProcessingModules;

/**
 * \brief Event manager. 
 */
class EVENTMANAGER_EXPORT NUIEventManager {
public:
	/**
	 * \brief Destructor
	 */
	~NUIEventManager();

	static NUIEventManager* instance();

	/**
	 * \brief Starts the event managing system
	 */
	NUISTATUS start();

	/**
	 * \brief Pauses the event managing system.
	 */
	NUISTATUS pause();

	/**
	 * \brief Resumes the pause event managing system.
	 */
	NUISTATUS resume();

	/**
	 * \brief Stops the event managing system.
	 */
	NUISTATUS stop();

	/**
	 * \brief Register a listener component.
	 * \param listener
	 */
	void registerListener(ISimpleEventListener*);

	/**
	 * Callback function which is called, when a NUI event is received.
	 */
	void eventReceived(nui::events::NUIEventPtr);

	/**
	 * Callback function which is called, when a Feedback event is received.
	 */
	void feedbackReceived(nui::events::FeedbackEventPtr);

	/**
	 * Directly sends a system event.
	 */
	void sendSystemEvent(nui::events::SystemEventPtr evt);

	/**
	 * \brief Reconfigure method.
	 */
	void reconfigure();
private:
	/**
	 * \brief Constructor
	 */
	NUIEventManager() :
			m_thread(NULL), m_Controller(NULL), m_state(NOT_INITIALIZED) {
		m_efunc = std::bind1st(std::mem_fun(&NUIEventManager::eventReceived),
				this);
	}

	static NUIEventManager* eventManager_;

	/**
	 * \brief Pointer to thread.
	 */
	boost::thread* m_thread;
	/**
	 * \brief Returns an instance of a modality. If it not exists it will be created and automatically added to the vector.
	 * \param name -- name of the modality
	 */
	template<typename T>
	T* getInstanceModality(const std::string name) {
		for (std::vector<IModality*>::iterator it = m_modalities.begin();
				it != m_modalities.end(); ++it) {
			if ((*it)->name() == name) {
				return static_cast<T*>(*it);
			}
		}
		T* instance = new T();
		m_modalities.push_back(instance);
		return instance;
	};
	template<typename T>
	T* getInstanceListener(const std::string name) {
		for (std::vector<ISimpleEventListener*>::iterator it = m_event_listener.begin();
				it != m_event_listener.end(); ++it) {
			if ((*it)->name() == name) {
				return static_cast<T*>(*it);
			}
		}
		T* instance = new T();
		m_event_listener.push_back(instance);
		return instance;
	};
	template<typename T>
	T* getInstanceProcessingModule(const std::string name) {
		for (std::vector<IProcessingModule*>::iterator it = m_processing_modules.begin();
				it != m_processing_modules.end(); ++it) {
			if ((*it)->name() == name) {
				return static_cast<T*>(*it);
			}
		}
		T* instance = new T();
		m_processing_modules.push_back(instance);
		return instance;
	};
	/**
	 * \brief Checks for an instance of a modality. 
	 * \param name -- name of the modality
	 * \return flag if the modality is in list and active
	 */
	bool containsModality(const std::string name) {
		for (std::vector<IModality*>::iterator it = m_modalities.begin();
				it != m_modalities.end(); ++it) {
			if ((*it)->name() == name) {
				return true;
			}
		}
		return false;
	}
	/**
	 * \brief Checks for an instance of a listener.
	 * \param name -- name of the listener
	 * \return flag if the listener is in list and active
	 */
	bool containsListener(const std::string name) {
		for (std::vector<ISimpleEventListener*>::iterator it = m_event_listener.begin();
				it != m_event_listener.end(); ++it) {
			if ((*it)->name() == name) {
				return true;
			}
		}
		return false;
	}
	/**
	 * \brief Removes modality from the list.
	 * \param name -- name of the modality
	 */
	bool removeModality(const std::string name);
	/**
	 * \brief Removes listener from the list.
	 * \param name -- name of the listener
	 */
	bool removeListener(const std::string name);
	/**
	 * \brief Removes processing from the list.
	 * \param name -- name of the listener
	 */
	bool removeProcessingModule(const std::string name);
	/**
	 * \brief Feedback function.
	 */
	void fireEvent(nui::events::FeedbackEventPtr);

	/** Controller which sends the events */
	Controller* m_Controller;

	/** Collector with registered interaction modalities. */
	ModalityVector m_modalities;

	FeedBackListeners m_feedback_listner;
	/**
	 * Event listeners.
	 */
	EventListeners m_event_listener;
	/**
	 * Processing modules.
	 */
	ProcessingModules m_processing_modules;
	/**
	 * Currently used tracking device.
	 */
	std::string m_currentTrackingDevice;
	/**
	 * \brief State of the event manager.
	 */
	RunTimeState m_state;
	/**
	 * \brief Event function
	 */
	EventFunc m_efunc;

};

