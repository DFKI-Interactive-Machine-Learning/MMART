#include "devices/LEAPMotionWrapper.hpp"

#include <Eigen/Core>
#include <bitset>
using namespace nui::events;

#define LEAP_TO_EIGEN(V) Eigen::Vector3f(V.x, V.y, V.z)
#define FINGER_NAME(T) static_cast<nui::events::FingerName>(T)
#define BONE_NAME(T)   static_cast<nui::events::BoneName>(T)


const Eigen::Vector3f DEFAULT_VECTOR(0.f, 0.f, 0.f);

bool LEAPMotionWrapper::isActive(GestureSetup& setup, LeapGesture gesture) {
	return 0 < (setup & gesture);
}

void LEAPMotionWrapper::reconfigure() {
	m_gesture_setup |= (
			nui::config()->get("Leap.Gestures.Circle", false) ?
					CIRCLE_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Leap.Gestures.Circle", false))<< "Circle gesture enabled";
	m_gesture_setup |= (
			nui::config()->get("Leap.Gestures.Swipe", false) ?
					SWIPE_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Leap.Gestures.Swipe", false))<< "Swipe gesture enabled";
	m_gesture_setup |= (
			nui::config()->get("Leap.Gestures.Keytap", false) ?
					KEY_TAP_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Leap.Gestures.Keytap", false))<< "Key tap gesture enabled";
	m_gesture_setup |= (
			nui::config()->get("Leap.Gestures.Screentap", false) ?
					SCREEN_TAP_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Leap.Gestures.Screentap", false))<< "Screen tap gesture enabled";
	m_stream_raw = nui::config()->get("Leap.Stream.raw", false);
	LOG_IF(INFO, m_stream_raw)<< "Stream raw LEAP frame.";
	m_stream_gestures = nui::config()->get("Gestures.Stream.raw", false);
	LOG_IF(INFO, m_stream_gestures)<< "Stream raw gesture LEAP frame.";
}

NUISTATUS LEAPMotionWrapper::initialize() {
	m_controller.enableGesture(Leap::Gesture::TYPE_SWIPE,
			isActive(m_gesture_setup, SWIPE_GESTURE));
	m_controller.enableGesture(Leap::Gesture::TYPE_CIRCLE,
			isActive(m_gesture_setup, CIRCLE_GESTURE));
	m_controller.enableGesture(Leap::Gesture::TYPE_KEY_TAP,
			isActive(m_gesture_setup, KEY_TAP_GESTURE));
	m_controller.enableGesture(Leap::Gesture::TYPE_SCREEN_TAP,
			isActive(m_gesture_setup, SCREEN_TAP_GESTURE));
	return NUI_SUCCESS;
}

NUISTATUS LEAPMotionWrapper::unInitialize() {
	return NUI_SUCCESS;
}

void LEAPMotionWrapper::process(EventQueue& evtQue) {
	if (m_controller.isConnected()) {
		// Get the most recent frame and report some basic information
		const Leap::Frame frame = m_controller.frame();
		if (!frame.hands().isEmpty()) {
			processHands(evtQue, frame.hands());
		}
		else
		{
			// None
			createNoneEvent(evtQue);
		}
		// Get gestures
		if (!frame.gestures().isEmpty()) {
			processGestures(evtQue, frame.gestures());
		}
		if (!frame.pointables().isEmpty()) {
			processCalibratedScreens(evtQue, m_controller.locatedScreens(),
					frame.pointables());
		}
	}
}

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


void LEAPMotionWrapper::createNoneEvent(EventQueue& que)
{
	LeapTrackingEvent* evt = new LeapTrackingEvent();
	evt->isStreaming = false;
	createEvent(evt, m_stream_raw);
	HandType* hevt = new HandType();
	hevt->id = -1;
	hevt->direction = DEFAULT_VECTOR;
	hevt->palmNormal = DEFAULT_VECTOR;
	hevt->palmPosition = DEFAULT_VECTOR;
	hevt->palmVelocity = DEFAULT_VECTOR;
	hevt->sphereCenter = DEFAULT_VECTOR;
	hevt->sphereRadius = 0.f;
	hevt->confidence   = 0.f;

	for (int32_t i = 0; i < 5; i++) {
		FingerType* fevt = new FingerType();
		fevt->name = FINGER_NAME(i);
		fevt->direction = DEFAULT_VECTOR;
		fevt->tipPosition = DEFAULT_VECTOR;
		fevt->tipVelocity = DEFAULT_VECTOR;
		fevt->width = 0.f;
		fevt->length = 0.f;
		// Get finger bones
		for (int b = 0; b < 4; b++) {
			BoneType* bptr = new BoneType();
			bptr->length = 0.f;
			bptr->width  = 0.f;
			bptr->center = DEFAULT_VECTOR;
			bptr->direction = DEFAULT_VECTOR;
			bptr->name = BONE_NAME(b);
			fevt->bones[BONE_NAME(b)] = BONETYPE_PTR(bptr);

		}
		hevt->fingers[FINGER_NAME(i)] = FINGERTYPE_PTR(fevt);
	}
	evt->hands.push_back(HANDTYPE_PTR(hevt));
	que.push(evt);
}

void LEAPMotionWrapper::processHands(EventQueue& que, const Leap::HandList& hands) {
	LeapTrackingEvent* evt = new LeapTrackingEvent();
	evt->isStreaming = true;
	if(m_stream_raw)
	{
		evt->processingtype = OUTPUT_EVENT;
	}
	createEvent(evt, m_stream_raw);
	for (int h = 0; h < hands.count(); h++) {
		const Leap::Hand hand = hands[h];
		HandType* hevt = new HandType();
		hevt->id            = hand.id();
		hevt->grabStrength  = hand.grabStrength();
		hevt->pinchStrength = hand.pinchStrength();
		hevt->direction     = LEAP_TO_EIGEN(hand.direction());
		hevt->palmNormal    = LEAP_TO_EIGEN(hand.palmNormal());
		hevt->palmPosition  = LEAP_TO_EIGEN(hand.stabilizedPalmPosition());
		hevt->palmVelocity  = LEAP_TO_EIGEN(hand.palmVelocity());
		hevt->sphereCenter  = LEAP_TO_EIGEN(hand.sphereCenter());
		hevt->sphereRadius  = hand.sphereRadius();
		hevt->confidence    = hand.confidence();
		hevt->side = hand.isRight() ? RIGHT : LEFT;
		// Check if the hand has any fingers
		const Leap::FingerList fingers = hand.fingers();

		for (Leap::FingerList::const_iterator fl = fingers.begin();
				fl != fingers.end(); ++fl) {
			const Leap::Finger finger = *fl;
			FingerType* fevt = new FingerType();
			fevt->name = FINGER_NAME(finger.type());
			fevt->direction = LEAP_TO_EIGEN(finger.direction());
			fevt->tipPosition = LEAP_TO_EIGEN(finger.tipPosition());
			fevt->tipVelocity = LEAP_TO_EIGEN(finger.tipVelocity());
			fevt->width = finger.width();
			fevt->length = finger.length();
			// Get finger bones
			Leap::Bone bone;
			Leap::Bone::Type boneType;
			for (int b = 0; b < 4; b++) {
				boneType = static_cast<Leap::Bone::Type>(b);
				bone = finger.bone(boneType);
				BoneType* bptr = new BoneType();
				bptr->length = bone.length();
				bptr->width = bone.width();
				bptr->center = LEAP_TO_EIGEN(bone.center());
				bptr->direction = LEAP_TO_EIGEN(bone.direction());
				bptr->name = BONE_NAME(boneType);
				fevt->bones[BONE_NAME(boneType)] = BONETYPE_PTR(bptr);

			}
			hevt->fingers[FINGER_NAME(finger.type())] = FINGERTYPE_PTR(fevt);
		}
		evt->hands.push_back(HANDTYPE_PTR(hevt));
	}
	que.push(evt);
}

void LEAPMotionWrapper::processGestures(EventQueue& que, const Leap::GestureList& gestures) {

	for (int g = 0; g < gestures.count(); ++g) {
		Leap::Gesture gesture = gestures[g];
		switch (gesture.type()) {
		case Leap::Gesture::TYPE_CIRCLE: {
			Leap::CircleGesture circle = gesture;
			// Calculate angle swept since last frame
			float sweptAngle = 0;
			if (circle.state() != Leap::Gesture::STATE_START) {
				Leap::CircleGesture previousUpdate = Leap::CircleGesture(
						m_controller.frame(1).gesture(circle.id()));
				sweptAngle = (circle.progress() - previousUpdate.progress()) * 2
						* Leap::PI;
			}
			CircleGestureEvent* cg = new CircleGestureEvent();
			createEvent(cg, m_stream_gestures);
			cg->id = gesture.id();
			cg->radius = circle.radius();
			cg->progress = circle.progress();
			cg->state = static_cast<LeapGestureState>(circle.state());
			cg->angle = sweptAngle * Leap::RAD_TO_DEG;
			cg->clockwise = (circle.pointable().direction().angleTo(
					circle.normal()) <= Leap::PI / 4);
			que.push(cg);
			break;
		}
		case Leap::Gesture::TYPE_SWIPE: {
			Leap::SwipeGesture swipe = gesture;
			SwipeGestureEvent* sg = new SwipeGestureEvent();
			createEvent(sg, m_stream_gestures);
			sg->id = gesture.id();
			sg->state = static_cast<LeapGestureState>(gesture.state());
			Leap::Vector v = swipe.direction();
			sg->direction = Eigen::Vector3f(v.x, v.y, v.z);
			sg->speed = swipe.speed();
			que.push(sg);
			break;
		}
		case Leap::Gesture::TYPE_KEY_TAP: {
			Leap::KeyTapGesture tap = gesture;
			KeyTapGestureEvent* kte = new KeyTapGestureEvent();
			createEvent(kte, m_stream_gestures);
			kte->id = gesture.id();
			kte->state = static_cast<LeapGestureState>(gesture.state());
			Leap::Vector v = tap.position();
			kte->position = Eigen::Vector3f(v.x, v.y, v.z);
			Leap::Vector v2 = tap.direction();
			kte->direction = Eigen::Vector3f(v2.x, v2.y, v2.z);
			que.push(kte);
			break;
		}
		case Leap::Gesture::TYPE_SCREEN_TAP: {
			Leap::ScreenTapGesture screentap = gesture;
			ScreenTapGestureEvent* ste = new ScreenTapGestureEvent();
			createEvent(ste, m_stream_gestures);
			ste->id = gesture.id();
			ste->state = static_cast<LeapGestureState>(gesture.state());
			Leap::Vector v = screentap.position();
			ste->position = Eigen::Vector3f(v.x, v.y, v.z);
			Leap::Vector v2 = screentap.direction();
			ste->direction = Eigen::Vector3f(v2.x, v2.y, v2.z);
			que.push(ste);
			break;
		}
		default:
			BOOST_LOG_TRIVIAL(error)<< "Unknown gesture type.";
			break;
		}
	}
}

void LEAPMotionWrapper::processCalibratedScreens(EventQueue& que,
		const Leap::ScreenList& screens, const Leap::PointableList& points) {
	ScreenPointerEvent* evt = new ScreenPointerEvent();
	createEvent(evt, m_stream_raw);
	bool trigger = false;
	for (int g = 0; g < points.count(); ++g) {
		Leap::Pointable p = points[g];
		if (p.isValid()) {
			ScreenPointType* pointer = new ScreenPointType();

			Leap::Screen screen = screens.closestScreenHit(p);
			if (screen.id() != -1) {
				Leap::Vector normalizedCoordinates = screen.intersect(p, true);
				pointer->id = p.id();
				pointer->screenid = screen.id();
				pointer->type = p.isFinger() ? FINGER_POINTER : TOOL_POINTER;
				pointer->x = (int) (normalizedCoordinates.x
						* screen.widthPixels());
				pointer->y =
						screen.heightPixels()
								- (int) (normalizedCoordinates.y
										* screen.heightPixels());
				Leap::Vector intersection = screen.intersect(p, false);
				Leap::Vector tipToScreen = intersection - p.tipPosition();
				pointer->distance = tipToScreen.magnitude();
				evt->pointers.push_back(SCREENPOINTTYPEPTR_PTR(pointer));
				trigger = true;
			}
		}
	}
	if (trigger) {
		que.push(evt);
	}
}

