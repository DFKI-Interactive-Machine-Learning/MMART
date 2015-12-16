#include "GestureRecognition.hpp"

using namespace nui::events;

inline void createEvent(NUIEvent* evt, bool stream) {
	evt->device = LEAP_MOTION_MODALITY;
	evt->timestamp = timestamp_now();
	if(stream)
	{
		evt->processingtype = OUTPUT_EVENT;
	}
	else
	{
		evt->processingtype = INTERNAL_EVENT;
	}
}

void GestureRecognition::reconfigure() {
	m_stream_gestures = nui::config()->get("Gestures.Stream.raw", false);
}

void GestureRecognition::process(nui::events::CamEvent&)
{

}
void GestureRecognition::process(nui::events::UserAttentionEvent&)
{

}

NUISTATUS GestureRecognition::initialize() {
	return NUI_SUCCESS;
}

NUISTATUS GestureRecognition::unInitialize() {
	return NUI_SUCCESS;
}

void GestureRecognition::process(LeapTrackingEvent& evt) {
	// Check of easy to detect gestures
	checkForPinch(evt);
	checkForGrab(evt);
}

void GestureRecognition::checkForPinch(nui::events::LeapTrackingEvent& evt) {
	for (HandVector::iterator it = evt.hands.begin(); it != evt.hands.end();
			++it) {
		HandTypePtr h = static_cast<HandTypePtr>(*it);
		if (h.get()->confidence > 0.8f && h.get()->pinchStrength > 0.5f) {
			PinchGestureEvent* evt = new PinchGestureEvent();
			createEvent(evt, m_stream_gestures);
			evt->handside = h.get()->side;
			evt->pinchStrength = h.get()->pinchStrength;
			fireEvent(evt);
		}
	}
}
void GestureRecognition::checkForGrab(nui::events::LeapTrackingEvent& evt) {
	bool g = false;
	for (HandVector::iterator it = evt.hands.begin(); it != evt.hands.end();
			++it) {
		HandTypePtr h = static_cast<HandTypePtr>(*it);
		if (h.get()->confidence > 0.8f && h.get()->grabStrength > 0.5f) {
			GrabGestureEvent* evt = new GrabGestureEvent();
			createEvent(evt, m_stream_gestures);
			evt->handside = h.get()->side;
			evt->grabStrength = h.get()->grabStrength;
			fireEvent(evt);
		}
	}
}
