/*
 * This file is part of the AV-NUI project.
 * Copyright (C) 2012-2014 DFKI GmbH. All rights reserved.
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
#include "interface.pb.h"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <queue>          // std::priority_queue
#include <vector>

namespace nui {
namespace events {

/**
 * Checks if a bit is set.
 */
inline bool is_bit_set(unsigned value, unsigned bitindex) {
	return (value & (1 << bitindex)) != 0;
}

/**
 *	\brief Supported gesture types
 */
enum EventType {
	NUI_EVENT,
	DEPTHSENSE_EVENT,
	HAND_GESTURE_EVENT,
	CIRCLE_GESTURE_EVENT,
	GRAB_EVENT,
	KEYTAP_GESTURE_EVENT,
	SWIPE_GESTURE_EVENT,
	SCREEN_TAP_GESTURE_EVENT,
	SCREEN_POINTER_EVENT,
	PINCH_EVENT,
	LEAP_ACTION_GESTURE_EVENT,
	LEAP_TRACKING_EVENT,
	MYO_EVENT,
	CAM_EVENT,
	PMDNANO_EVENT,
	REALSENSE_EVENT,
	KINECTV2_EVENT,
    HEAD_TRACKING_EVENT,
	SYSTEM_EVENT,
	USER_ATTENTION_EVENT
};

/**
 *	\brief Event types
 */
enum ProcessingType {
	INTERNAL_EVENT = 0x1,
	RAW_DATA_EVENT = INTERNAL_EVENT << 1,
	OUTPUT_EVENT = RAW_DATA_EVENT << 1 // Events which are send to connected to NUI
};

/**
 * \brief General NUI event.
 */
struct NUIEvent {
	NUIEvent(const ProcessingType type = INTERNAL_EVENT, const EventType evttype = NUI_EVENT) :
			processingtype(type),
			evtType(evttype),
			timestamp(timestamp_now()),
			id(0) {
	};
	virtual ~NUIEvent() {
	};
	/**
	 * \brief Timestamp of event
	 */
	time_t timestamp;
	/**
	 * \brief Device which captured the event.
	 */
	std::string device;
	/**
	 * id.
	 */
	int32_t id;
	/**
	 * Processing type.
	 */
	ProcessingType processingtype;
	/**
	 * Event type.
	 */
	EventType evtType;
	/**
	 * ID.
	 */
	int32_t event_id() const {
		return id;
	}
	/**
	 * Event timestamp.
	 */
	time_t event_timestamp() const {
		return timestamp;
	}
	/**
	 * Event device.
	 */
	std::string event_device() const {
		return device;
	}
	/**
	 * \brief Returns the type of event.
	 */
	virtual EventType eventType() const
	{
		return evtType;
	}
	/**
	 * \brief Returns the type of event.
	 */
	virtual ProcessingType processType() const {
		return processingtype;
	}
};

/** Shared pointer*/
typedef boost::shared_ptr<nui::events::NUIEvent> NUIEventPtr;

/**
 * \brief General NUI event.
 */
struct FeedbackEvent {
	/**
	 * \brief Timestamp of event
	 */
	time_t timestamp;
	/**
	 * \brief Command event.
	 */
	std::string command;
};

typedef boost::shared_ptr<nui::events::FeedbackEvent> FeedbackEventPtr;
#define FEEDBACKEVENT_PTR(E) nui::events::FeedbackEventPtr(E)

/** 
 *	\brief Listener for feedback events coming from application.
 * 
 */
class COMMON_EXPORT IFeedbackListener {
public:
	/*
	 * \brief Virtual destructor.
	 */
	virtual ~IFeedbackListener() {};
	/**
	 * \brief Processing event.
	 */
	virtual void
	process(FeedbackEventPtr) = 0;
};

/**
 * \brief Side.
 */
enum Side {
	RIGHT = 0x0, LEFT = 0x1, NONE = 0x2
};
/*************  Myo Device   **************************/

/**
 * The possible myo gesture.
 */
enum MyoPose {
	UNKNOWN                = 0x0,
	REST_GESTURE           = 0x1,
	FIST_GESTURE           = (REST_GESTURE << 1),
	WAVE_IN_GESTURE        = (FIST_GESTURE << 1),
	WAVE_OUT_GESTURE       = (WAVE_IN_GESTURE << 1),
	FINGERS_SPREAD_GESTURE = (WAVE_OUT_GESTURE << 1),
	DOUBLE_TAB_GESTURE     = (FINGERS_SPREAD_GESTURE << 1)
};


/**
 * The possible myo gesture.
 */
enum MyoState {
	NOT_CONNECT            = 0x0,
	CONNECTION_ESTABLISHED = 0x1,
	CONNECTION_LOST        = 0x2,
	BATTERY_STATUS_LOW     = 0x3,
	CONNECTION_GOOD        = 0x4,
    CONNECTION_POOR        = 0x5
};

/**
 * \brief Event from Myo Device
 */
struct MyoEvent: NUIEvent {
	/** 
	 * \brief These values are set by onArmSync() and onArmUnsync() above.
	 */
    bool onArm;
	/** 
	 * \brief The values of this array is set by onEmgData() above.
	 */
    std::array<int8_t, 8> emgSamples;
	/** 
	 * \brief Info on which arm the myo is mounted.
	 */
    Side whichArm;
    /** 
	 * \brief  This is set by onUnlocked() and onLocked() above.
	 */
    bool isUnlocked;
	
	/**
	 * \brief Battery level.
	 */
	uint8_t batteryLevel;
	
	/**
	 * \brief RSSI.
	 */
	int8_t rssi;
	
    /** 
	 * \brief These values are set by onOrientationData() and onPose() above.
	 */
    int roll_w, pitch_w, yaw_w;

	/** 
	 * \brief Accelerometer data.
	 */
    float acc_x, acc_y, acc_z;
	
	/** 
	 * \brief Gyroscope data. 
	 */
    float gyro_x, gyro_y, gyro_z;
	/** 
	 * \brief Current pose of the hand.
	 */
    MyoPose currentPose;

	/** 
	 * \brief Current state.
	 */
	MyoState currentState;

	virtual EventType eventType() const {
		return MYO_EVENT;
	}
};

typedef boost::shared_ptr<MyoEvent> MyoEventPtr;
#define NUIEVENTPTR_TO_MYOPTR(E) boost::static_pointer_cast<MyoEvent>(E)

/** --------------------------------------------- LEAP ------------------------------------------------------- */
/**
 * The possible gesture states.
 */
enum LeapGestureState {
	STATE_INVALID = -1, /* An invalid state */
	STATE_START = 1, /* The gesture is starting. Just enough has happened to recognize it. */
	STATE_UPDATE = 2, /* The gesture is in progress. (Note: not all gestures have updates). */
	STATE_STOP = 3, /* The gesture has completed or stopped. */
};

/**
 * The possible pointer type of LEAP device.
 */
enum PointerType {
	TOOL_POINTER = 0, FINGER_POINTER = 1
};

enum FingerName {
	TYPE_THUMB = 0, /**< The thumb */
	TYPE_INDEX = 1, /**< The index or fore-finger */
	TYPE_MIDDLE = 2, /**< The middle finger */
	TYPE_RING = 3, /**< The ring finger */
	TYPE_PINKY = 4 /**< The pinky or little finger */
};

enum BoneName {
	TYPE_METACARPAL = 0, /** Bone connected to the wrist inside the palm */
	TYPE_PROXIMAL = 1, /** Bone connecting to the palm */
	TYPE_INTERMEDIATE = 2, /** Bone between the tip and the base*/
	TYPE_DISTAL = 3 /** Bone at the tip of the finger */
};

/** Hand Gesture events */
struct LeapGestureEvent : NUIEvent {
	int32_t id;
	LeapGestureState state;
	Side handside;
	LeapGestureEvent() : NUIEvent(OUTPUT_EVENT)
	{
		id = 0;
		state = STATE_INVALID;
		handside = RIGHT;
	};
};

/** Shared pointer*/
typedef boost::shared_ptr<nui::events::LeapGestureEvent> LeapGestureEventPtr;

/** LEAP Motion Gesture events */
struct CircleGestureEvent: LeapGestureEvent {
	float radius;
	float progress;
	float angle;
	bool clockwise;
	virtual EventType eventType() const {
		return CIRCLE_GESTURE_EVENT;
	};
};

struct SwipeGestureEvent: LeapGestureEvent {
	Eigen::Vector3f direction;
	float speed;
	virtual EventType eventType() const {
		return SWIPE_GESTURE_EVENT;
	};
};

struct KeyTapGestureEvent: LeapGestureEvent {
	Eigen::Vector3f position;
	Eigen::Vector3f direction;
	virtual EventType eventType() const {
		return KEYTAP_GESTURE_EVENT;
	};
};

struct PinchGestureEvent: LeapGestureEvent {
	int32_t id;
	LeapGestureState state;
	float pinchStrength;
	virtual EventType eventType() const {
		return PINCH_EVENT;
	};
};

struct GrabGestureEvent: LeapGestureEvent {
	float grabStrength;
	virtual EventType eventType() const {
		return GRAB_EVENT;
	};
};

struct ScreenTapGestureEvent: LeapGestureEvent {
	Eigen::Vector3f position;
	Eigen::Vector3f direction;
	virtual EventType eventType() const {
		return SCREEN_TAP_GESTURE_EVENT;
	};
};

struct LeapActionGestureEvent: LeapGestureEvent {
	std::string action;
	virtual EventType eventType() const {
		return LEAP_ACTION_GESTURE_EVENT;
	};
};

struct BoneType {
	BoneName name;
	float width;
	float length;
	Eigen::Vector3f center;
	Eigen::Vector3f direction;
};

typedef boost::shared_ptr<nui::events::BoneType> BoneTypePtr;

typedef std::map<BoneName, nui::events::BoneTypePtr> BoneMap;

struct FingerType {
	FingerName name;
	float length; // The length of the visible portion of the object (from where it extends out of the hand to the tip).
	float width;      // The average width of the visible portion of the object.
	Eigen::Vector3f direction; // A unit direction vector pointing in the same direction as the object (i.e. from base to tip).
	Eigen::Vector3f tipPosition; // The position of the tip in millimeters from the Leap origin.
	Eigen::Vector3f tipVelocity; // The speed of the tip in millimeters per second.
	BoneMap bones;
};

typedef boost::shared_ptr<FingerType> FingerTypePtr;
typedef std::map<FingerName, FingerTypePtr> FingerMap;

struct HandType {
	Side side;
	int32_t id;
	float confidence;
	float pinchStrength;
	float grabStrength;
	Eigen::Vector3f palmPosition; // The center of the palm measured in millimeters from the Leap origin.
	Eigen::Vector3f palmVelocity; // The speed of the palm in millimeters per second.
	Eigen::Vector3f palmNormal; // A vector perpendicular to the plane formed by the palm of the hand. The vector points downward out of the palm.
	Eigen::Vector3f direction; // A vector pointing from the center of the palm toward the fingers.
	Eigen::Vector3f sphereCenter; // The center of a sphere fit to the curvature of the hand (as if it were holding a ball).
	float sphereRadius; // The radius of a sphere fit to the curvature of the hand. The radius changes with the shape of the hand.
	FingerMap fingers; //
};

typedef boost::shared_ptr<nui::events::HandType> HandTypePtr;

typedef std::vector<nui::events::HandTypePtr> HandVector;

struct LeapTrackingEvent: NUIEvent {
	bool isStreaming;
	HandVector hands;
	virtual EventType eventType() const {
		return LEAP_TRACKING_EVENT;
	};
};

struct ScreenPointType {
	int32_t id;
	int32_t screenid;
	PointerType type;
	float distance;
	int x;
	int y;
};

typedef boost::shared_ptr<ScreenPointType> ScreenPointTypePtr;
typedef std::vector<ScreenPointTypePtr> ScreenPointerVector;

struct ScreenPointerEvent: LeapGestureEvent {
	int32_t id;
	ScreenPointerVector pointers;
	virtual EventType eventType() const {
		return SCREEN_POINTER_EVENT;
	}
	;
	virtual ProcessingType processType() const {
		return OUTPUT_EVENT;
	}
	;
};

typedef boost::shared_ptr<LeapTrackingEvent> LeapTrackingEventPtr;
#define NUIEVENTPTR_TO_LEAPTRACKINGPTR(E) boost::static_pointer_cast<LeapTrackingEvent>(E)

typedef boost::shared_ptr<LeapActionGestureEvent> LeapActionGestureEventPtr;
#define NUIEVENTPTR_TO_LEAPACTIONEVENTPTR(E) boost::static_pointer_cast<LeapActionGestureEvent>(E)

typedef boost::shared_ptr<CircleGestureEvent> CircleGestureEventPtr;
#define NUIEVENTPTR_TO_CIRCLEGESTUREPTR(E) boost::static_pointer_cast<CircleGestureEvent>(E)

typedef boost::shared_ptr<SwipeGestureEvent> SwipeGestureEventPtr;
#define NUIEVENTPTR_TO_SWIPEGESTUREPTR(E) boost::static_pointer_cast<SwipeGestureEvent>(E)

typedef boost::shared_ptr<KeyTapGestureEvent> KeyTapGestureEventPtr;
#define NUIEVENTPTR_TO_KEYTAPGESTUREPTR(E) boost::static_pointer_cast<KeyTapGestureEvent>(E)

typedef boost::shared_ptr<PinchGestureEvent> PinchGestureEventPtr;
#define NUIEVENTPTR_TO_PINCHGESTUREPTR(E) boost::static_pointer_cast<PinchGestureEvent>(E)

typedef boost::shared_ptr<GrabGestureEvent> GrabGestureEventPtr;
#define NUIEVENTPTR_TO_GRABGESTUREPTR(E) boost::static_pointer_cast<GrabGestureEvent>(E)

typedef boost::shared_ptr<ScreenTapGestureEvent> ScreenTapGestureEventPtr;
#define NUIEVENTPTR_TO_SCREENTAPGESTUREPTR(E) boost::static_pointer_cast<ScreenTapGestureEvent>(E)

typedef boost::shared_ptr<ScreenPointerEvent> ScreenPointerEventPtr;
#define NUIEVENTPTR_TO_SCREENPOINTERPTR(E) boost::static_pointer_cast<ScreenPointerEvent>(E)

/*************  Hand gestures **************************/

/**
 * \brief Event for body pose gesture.
 * Each event has a type, a confidence calculated by the classifier, 
 * and an intensity of the gesture.
 */
struct HandPostureGestureEvent: NUIEvent {
	/*
	 * Name of the posture
	 */
	std::string event_name;
	/*
	 * Hand
	 */
	Side hand;
	/*
	 * Intensity level of gesture. Normalized value from [0.0 - 1.0]
	 */
	float intensity;
	/*
	 * Confidence level of gesture. Normalized value from [0.0 - 1.0]
	 */
	float confidence;

	virtual EventType eventType() const {
		return HAND_GESTURE_EVENT;
	}
	;
	virtual ProcessingType processType() const {
		return OUTPUT_EVENT;
	}
	;
};

typedef boost::shared_ptr<HandPostureGestureEvent> HandPostureGestureEventPtr;
#define NUIEVENTPTR_TO_HANDGESTUREPTR(E) boost::static_pointer_cast<HandPostureGestureEvent>(E)

/*************  WebCam   **************************/
/**
 * \brief Event from Camera
 */
struct CamEvent: NUIEvent {
	uint32_t id;   // id of web cam
	double dWidth; //get the width of frames of the video
	double dHeight; //get the height of frames of the  video
	cv::Mat frame;
	virtual EventType eventType() const {
		return CAM_EVENT;
	}
};

typedef boost::shared_ptr<CamEvent> CamEventPtr;
#define NUIEVENTPTR_TO_WEBCAMPTR(E) boost::static_pointer_cast<CamEvent>(E)
/*************  PMDNano   **************************/
/**
 * \brief Event from PMDNano Camera
 */
struct PMDNanoEvent: NUIEvent {
	uint32_t id;           // id of web cam
	double dWidth;         // get the width of frames of the video
	double dHeight;        // get the height of frames of the  video
	cv::Mat depth;         // depth
	cv::Mat amplitude;     // amplitude
	PointCloud::Ptr cloud; // point cloud
	PointCloud::Ptr filteredCloud; // point cloud
	virtual EventType eventType() const {
		return PMDNANO_EVENT;
	}
};

typedef boost::shared_ptr<PMDNanoEvent> PMDNanoEventPtr;
#define NUIEVENTPTR_TO_PMDNANOPTR(E) boost::static_pointer_cast<PMDNanoEvent>(E)
/*************  RealSense   **************************/
/**
 * \brief Event from RealSense Camera
 */
struct RealSenseEvent: NUIEvent {
	uint32_t id;               // id of web cam
	PointCloudRGBA::Ptr cloud; // point cloud
	cv::Mat depth;             // depth
	cv::Mat rgb;               // Color
	cv::Mat ir;                // IR
	virtual EventType eventType() const {
		return REALSENSE_EVENT;
	}
};

typedef boost::shared_ptr<RealSenseEvent> RealSenseEventPtr;
#define NUIEVENTPTR_TO_REALSENSEPTR(E) boost::static_pointer_cast<RealSenseEvent>(E)
/*************  DepthSense   **************************/
/**
 * \brief Event from DepthSense Camera
 */
struct DepthSenseEvent: NUIEvent {
	uint32_t id;               // id of web cam
	PointCloudRGBA::Ptr cloud; // point cloud
	cv::Mat depth;             // depth
	cv::Mat rgb;               // Color
	virtual EventType eventType() const {
		return DEPTHSENSE_EVENT;
	}
};

typedef boost::shared_ptr<DepthSenseEvent> DepthSenseEventPtr;
#define NUIEVENTPTR_TO_DEPTHSENSEPTR(E) boost::static_pointer_cast<DepthSenseEvent>(E)
/*************  Kinectv2 Device   **************************/
/**
 * \brief Event from Kinect v2 Camera
 */
struct Kinectv2Event: NUIEvent {
	uint32_t id;           // id of web cam
	double dWidth;         // get the width of frames of the video
	double dHeight;        // get the height of frames of the  video
	cv::Mat depth;         // depth
	cv::Mat amplitude;     // amplitude
	PointCloud::Ptr cloud; // point cloud
	PointCloud::Ptr filteredCloud; // point cloud
	virtual EventType eventType() const {
		return KINECTV2_EVENT;
	}
};

typedef boost::shared_ptr<Kinectv2Event> Kinectv2EventPtr;
#define NUIEVENTPTR_TO_KINECTV2PTR(E) boost::static_pointer_cast<Kinectv2Event>(E)

/**
 * \brief Event containing current head and eye position
 */
struct HeadTrackingEvent: NUIEvent {
    /** Current translation vector */
    cv::Vec3d translation;
    /** Current rotation vector */
    cv::Vec3d rotation;

    cv::Vec2d eye_displacement_left;
    cv::Vec2d eye_displacement_right;

    virtual EventType eventType() const {
        return HEAD_TRACKING_EVENT;
    }
};

typedef boost::shared_ptr<HeadTrackingEvent> HeadTrackingEventPtr;
#define NUIEVENTPTR_TO_HEADTRACKINGPTR(E) boost::static_pointer_cast<HeadTrackingEvent>(E)

/*************  System Events **********************/

enum MessageType {
	MESSAGETYPE_COMMAND,
	MESSAGETYPE_INFO,
	MESSAGETYPE_CONFIGURATION
};
/**
 * \brief Event from system
 */
struct SystemEvent : NUIEvent {
	uint32_t id;
	MessageType type;
	std::string category;
	std::string message;

	virtual ProcessingType processType() const {
		return OUTPUT_EVENT;
	};

	virtual EventType eventType() const {
		return SYSTEM_EVENT;
	};
};

typedef boost::shared_ptr<SystemEvent> SystemEventPtr;
#define NUIEVENTPTR_TO_SYSTEMEVENTPTR(E) boost::static_pointer_cast<SystemEvent>(E)

struct AttentionArea {
	/** label for this area */
	std::string label;

	/** The point defining the area (bottom left corner when looking at the plane)*/
	Eigen::Vector3f point;

	/** vector pointing from mPoint to top left corner */
	Eigen::Vector3f vector_top;

	/** vector pointing from mPoint to bottom right corner */
	Eigen::Vector3f vector_right;

	/** the normal of the plane (is calculated in constructor */
	Eigen::Vector3f plane_normal;

	/** whether this target area is active */
	bool active;

	/** Distance to area. */
	float distance;

	/** Probability, that the user has his/her attention in this area. */
	float probability;
};

/**
 * \brief Event from Camera.
 */
struct UserAttentionEvent: NUIEvent {

	std::vector<AttentionArea> areas;

	virtual EventType eventType() const {
		return USER_ATTENTION_EVENT;
	}

	virtual ProcessingType processType() const {
		return OUTPUT_EVENT;
	};
};

typedef boost::shared_ptr<UserAttentionEvent> UserAttentionEventPtr;
#define NUIEVENTPTR_TO_USERATTENTIONPTR(E) boost::static_pointer_cast<UserAttentionEvent>(E)

//}
// ----------------------------------- Conversion MACROS -----------------------------------
#define NUIEVENT_PTR(E)           boost::shared_ptr<nui::events::NUIEvent>(static_cast<nui::events::NUIEvent*>(E))
#define BONETYPE_PTR(E)           nui::events::BoneTypePtr(E)
#define FINGERTYPE_PTR(E)         nui::events::FingerTypePtr(E)
#define HANDTYPE_PTR(E)           nui::events::HandTypePtr(E)
#define SCREENPOINTTYPEPTR_PTR(E) nui::events::ScreenPointTypePtr(E)
// ----------------------------------- Conversion Functions ---------------------------------

COMMON_EXPORT void leap2frame(LeapTrackingEvent& evt, nui::stream::Frame* frame);

COMMON_EXPORT void circle2gesture(CircleGestureEvent& evt, nui::stream::Gesture* g);
COMMON_EXPORT void screentap2gesture(ScreenTapGestureEvent& evt, nui::stream::Gesture* g);
COMMON_EXPORT void swipe2gesture(SwipeGestureEvent& evt, nui::stream::Gesture* g);
COMMON_EXPORT void pinch2gesture(PinchGestureEvent& evt, nui::stream::Gesture* g);
COMMON_EXPORT void grab2gesture(GrabGestureEvent& evt, nui::stream::Gesture* g);
COMMON_EXPORT void action2gesture(LeapActionGestureEvent& evt, nui::stream::Gesture* g);
COMMON_EXPORT void system2system(SystemEvent& evt, nui::stream::SystemMessage* g);
COMMON_EXPORT void attention2attention(UserAttentionEvent& evt, nui::stream::UserAttention* g);
COMMON_EXPORT void myo2myoframe(MyoEvent& evt, nui::stream::MyoFrame* g);
}
}
