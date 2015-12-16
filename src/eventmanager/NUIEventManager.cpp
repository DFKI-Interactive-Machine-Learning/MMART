/*
 * This file is part of the NUI project.
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

#include <iostream>

#include "eventmanager/NUIEventManager.hpp"
#include "eventmanager/DataDumper.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

using namespace nui::events;

NUIEventManager* NUIEventManager::eventManager_ = NULL;

NUIEventManager* NUIEventManager::instance() {
	if (!eventManager_) {
		eventManager_ = new NUIEventManager();
		eventManager_->reconfigure();
		nui::config()->registerForReconfigure("*.Activated",
				boost::bind(&NUIEventManager::reconfigure, eventManager_));
	}
	return eventManager_;
}

inline SystemEventPtr shutdownEvent()
{
	SystemEvent* st = new SystemEvent();
	st->device = "SYSTEM";
	st->timestamp = timestamp_now();
	st->type = MESSAGETYPE_INFO;
	st->category = "EVENTMANAGER";
	st->message  = "SHUTDOWN";
	return SystemEventPtr(st);
}

void NUIEventManager::reconfigure() {
	BOOST_LOG_TRIVIAL(info)<< "Reconfiguring NUIEventManager...";
	// ----------------------------------- Enabled techniques -----------------------------------
	bool webcamEnabled       = nui::config()->get("WebCam.Activated", false);
	bool gesturesEnabled     = nui::config()->get("Gestures.Activated", false);
	bool dump                = nui::config()->get("General.Dump.Activated", false);
	bool viz                 = nui::config()->get("General.Viz.Activated", false);
	bool handGesturesEnabled = nui::config()->get("Processing.HandGestureRecognition.Activated", false);
	bool modelEnabled        = nui::config()->get("Processing.ModelClassifier.Activated", false);
	bool streamEnabled       = nui::config()->get("Streaming.Activated", false);
	bool constrollerEnabled  = nui::config()->get("Controller.Activated", false);
	// ----------------------------------- Logging ---------------------------------------------
	LOG_IF(info, dump) << "Data dumping activated.";
	LOG_IF(info, viz)  << "Data visualization activated.";
	// ------------------------------------------------------------------------------------------
#if WITH_DEVICE_MYO
	bool myoEnabled   = nui::config()->get("Myo.Activated", false);
	if(myoEnabled) {
		if(!containsModality(MYO_MODALITY)) {
			MyoWrapper* myo = getInstanceModality<MyoWrapper>(MYO_MODALITY);
			myo->connectListener(m_efunc);
			if(m_state == RUNNING) myo->start();
		}
	} else {
		LOG_IF(INFO, removeModality(MYO_MODALITY)) << "Myo removed";
	}
#endif
#ifdef WITH_DEVICE_LEAP_MOTION
	bool leapmotionEnabled   = nui::config()->get("Leap.Activated", false);
	if(leapmotionEnabled) {
		if(!containsModality(LEAP_MOTION_MODALITY)) {
			LEAPMotionWrapper* leap = getInstanceModality<LEAPMotionWrapper>(LEAP_MOTION_MODALITY);
			leap->connectListener(m_efunc);
			if(m_state == RUNNING) leap->start();
		}
	} else {
		LOG_IF(INFO, removeModality(LEAP_MOTION_MODALITY)) << "LEAP Motion removed";
	}
#endif
	if(webcamEnabled) {
		if(!containsModality(WEBCAM_DEVICE)) {
			WebCamWrapper* cam = getInstanceModality<WebCamWrapper>(WEBCAM_DEVICE);
			cam->connectListener(m_efunc);
			if(m_state == RUNNING) cam->start();
		}
	} else {
		LOG_IF(INFO, removeModality(WEBCAM_DEVICE)) << "LEAP Motion removed";
	}
#ifdef WITH_DEVICE_PMD_NANO
	bool pmdEnabled          = nui::config()->get("PMDNano.Activated", false);
	if(pmdEnabled) {
		if(!containsModality(PMDNANO_DEVICE)) {
			PMDNanoWrapper* cam = getInstanceModality<PMDNanoWrapper>(PMDNANO_DEVICE);
			cam->connectListener(m_efunc);
			if(m_state == RUNNING) cam->start();
		}
	} else {
		LOG_IF(INFO, removeModality(PMDNANO_DEVICE)) << "PMD Nano removed";
	}
#endif
#ifdef WITH_DEVICE_INTEL_REALSENSE
	bool realsenseEnabled   = nui::config()->get("RealSense.Activated", false);
	if(realsenseEnabled) {
		if(!containsModality(INTELREALSENSE_DEVICE)) {
			IntelRealSenseCameraWrapper* cam = getInstanceModality<IntelRealSenseCameraWrapper>(INTELREALSENSE_DEVICE);
			cam->connectListener(m_efunc);
			if(m_state == RUNNING) cam->start();
		}
	} else {
		LOG_IF(INFO, removeModality(INTELREALSENSE_DEVICE)) << "Intel RealSense removed";
	}
#endif
#ifdef WITH_DEVICE_DEPTHSENSE
	bool depthsenseEnabled   = nui::config()->get("DepthSense.Activated", false);
	if(depthsenseEnabled) {
		if(!containsModality(DEPTHLSENSE_DEVICE)) {
			DepthSenseCameraWrapper* cam = getInstanceModality<DepthSenseCameraWrapper>(DEPTHLSENSE_DEVICE);
			cam->connectListener(m_efunc);
			if(m_state == RUNNING) cam->start();
		}
	} else {
		LOG_IF(INFO, removeModality(DEPTHLSENSE_DEVICE)) << "DepthSense removed";
	}
#endif
#if WITH_DEVICE_KINECT_V2
	bool kinectEnabled       = nui::config()->get("Kinect.Activated", false);
	if(kinectEnabled) {
		if(!containsModality(KINECTV2_DEVICE)) {
			KinectWrapper* kin = getInstanceModality<KinectWrapper>(KINECTV2_DEVICE);
			kin->connectListener(m_efunc);
			if(m_state == RUNNING) kin->start();
		}
	} else {
		LOG_IF(INFO, removeModality(KINECTV2_DEVICE)) << "Kinect removed";
	}
#endif 
	if(dump)
	{
		DataDumper* dump = getInstanceListener<DataDumper>(DUMPER_MODULE);
		dump->registerListener(m_efunc);
		if(m_state == RUNNING) dump->start();
	}
	else
	{
		LOG_IF(INFO, removeListener(DUMPER_MODULE)) << "Data dumper removed";
	}
	/**
	if(viz)
	{
		DataViz* viz = getInstanceListener<DataViz>(VIZ_MODULE);
		viz->registerListener(m_efunc);
		if(m_state == RUNNING) viz->start();
	}
	else
	{
		LOG_IF(INFO, removeListener(VIZ_MODULE)) << "Data visualization removed";
	} */
#if	WITH_GESTURE_RECOGNITION
	if(gesturesEnabled)
	{
		if(!containsListener("gesture"))
		{
			GestureRecognition* gest = getInstanceListener<GestureRecognition>(GESTURE_RECOGNITON_MODULE);
			gest->registerListener(m_efunc);
			if(m_state == RUNNING) gest->start();
		}
	}
	else
	{
		LOG_IF(INFO, removeListener(GESTURE_RECOGNITON_MODULE)) << "Gesture recognition removed";
	}
	if(handGesturesEnabled)
	{
		if(!containsListener(HANDGESTURE_RECOGNITON_MODULE))
		{
			HandGestureRecognition* gest = getInstanceProcessingModule<HandGestureRecognition>(HANDGESTURE_RECOGNITON_MODULE);
			gest->registerListener(m_efunc);
			if(m_state == RUNNING) gest->start();
		}
	}
	else
	{
		LOG_IF(INFO, removeListener(GESTURE_RECOGNITON_MODULE)) << "Gesture recognition removed";
	}
	if (modelEnabled)
	{
		if (!containsListener(MODELCLASSIFIER_MODULE))
		{
			ModelClassifier* gest = getInstanceProcessingModule<ModelClassifier>(MODELCLASSIFIER_MODULE);
			gest->registerListener(m_efunc);
			if (m_state == RUNNING) gest->start();
		}
	}
	else
	{
		LOG_IF(INFO, removeListener(GESTURE_RECOGNITON_MODULE)) << "Gesture recognition removed";
	}
	
#endif
	if(m_Controller == NULL && constrollerEnabled) {
		m_Controller = new Controller();
		m_Controller->RegisterReceivedCommandFunc(std::bind1st(std::mem_fun(&NUIEventManager::feedbackReceived), this));
	}
	if(m_state == NOT_INITIALIZED) m_state = INITIALIZED;
}

bool NUIEventManager::removeModality(const std::string name) {
	ModalityVector::iterator pos;
	bool found = false;
	for (ModalityVector::iterator it = m_modalities.begin();
			it != m_modalities.end(); ++it) {
		if ((*it)->name() == name) {
			pos = it;
			found = true;
		}
	}
	if(found)
	{
		(*pos)->stop();
		m_modalities.erase(pos);
		PtrRelease((*pos));
		return true;
	}
	return false;
}


bool NUIEventManager::removeListener(const std::string name) {
	EventListeners::iterator pos;
	bool found = false;
	for (EventListeners::iterator it = m_event_listener.begin();
			it != m_event_listener.end(); ++it) {
		if ((*it)->name() == name) {
			pos = it;
			found = true;
		}
	}
	if(found)
	{
		(*pos)->stop();
		m_event_listener.erase(pos);
		PtrRelease((*pos));
		return true;
	}
	return false;
}


bool NUIEventManager::removeProcessingModule(const std::string name) {
	ProcessingModules::iterator pos;
	bool found = false;
	for (ProcessingModules::iterator it = m_processing_modules.begin();
			it != m_processing_modules.end(); ++it) {
		if ((*it)->name() == name) {
			pos = it;
			found = true;
		}
	}
	if(found)
	{
		(*pos)->stop();
		m_processing_modules.erase(pos);
		PtrRelease((*pos));
		return true;
	}
	return false;
}

void NUIEventManager::registerListener(ISimpleEventListener* listener) {
	m_event_listener.push_back(listener);
}

NUIEventManager::~NUIEventManager() {
	PtrRelease(m_Controller);
	for (ModalityVector::iterator it = m_modalities.begin();
			it != m_modalities.end(); ++it) {
		PtrRelease(*it);
	}
}

NUISTATUS NUIEventManager::start() {
	if(m_Controller)
	{
		m_Controller->open();
	}
	for (ModalityVector::iterator it = m_modalities.begin();
			it != m_modalities.end(); ++it) {
		NUISTATUS status = (*it)->start();
		BOOST_LOG_TRIVIAL(info) << "STATUS : " << I2Str(status);
	}
	for (EventListeners::iterator it = m_event_listener.begin(); it != m_event_listener.end(); ++it) {
		(*it)->start();
	}
	m_state = RUNNING;
	return NUI_SUCCESS;
}

NUISTATUS NUIEventManager::pause() {
	for (ModalityVector::iterator it = m_modalities.begin();
			it != m_modalities.end(); ++it) {
		(*it)->pause();
	}
	m_state = PAUSED;
	return 0;
}

NUISTATUS NUIEventManager::resume() {
	for (ModalityVector::iterator it = m_modalities.begin();
			it != m_modalities.end(); ++it) {
		(*it)->resume();
	}
	m_state = RUNNING;
	return 0;
}

NUISTATUS NUIEventManager::stop() {
	sendSystemEvent(shutdownEvent());
	BOOST_LOG_TRIVIAL(info)<< "Stopping Modalities...";
	for (ModalityVector::iterator it = m_modalities.begin(); it != m_modalities.end(); ++it)
	{
		(*it)->stop();
	}
	for (EventListeners::iterator it = m_event_listener.begin(); it != m_event_listener.end(); ++it) {
		(*it)->stop();
	}
	
	if(m_Controller)
	{
		BOOST_LOG_TRIVIAL(info) << "Stopping Application Connector...";
		m_Controller->close();
	}
	m_state = STOPPED;
	return NUI_SUCCESS;
}

void NUIEventManager::sendSystemEvent(nui::events::SystemEventPtr evt)
{
	if(m_Controller)
	{
		m_Controller->sendSystemEvent(*evt);
	}
}

void NUIEventManager::eventReceived(NUIEventPtr evt) {
	// first perform further processing
	for (ProcessingModules::iterator it = m_processing_modules.begin(); it != m_processing_modules.end(); ++it) {
		(*it)->update(evt); // might update the event
	}
	//send event directly to the registered listeners
	for (EventListeners::iterator it = m_event_listener.begin(); it != m_event_listener.end(); ++it) {
		(*it)->update(evt);
	}
	if((evt.get()->processType() & OUTPUT_EVENT) != 0 && m_Controller) // only send output events
	{
		m_Controller->sendEvent(evt);
	}
}

void NUIEventManager::feedbackReceived(FeedbackEventPtr evt) {
	fireEvent(evt);
}

void NUIEventManager::fireEvent(FeedbackEventPtr evt) {
	for (FeedBackListeners::iterator it = m_feedback_listner.begin(); it != m_feedback_listner.end(); ++it) {
		(*it)->process(evt);
	}
}
