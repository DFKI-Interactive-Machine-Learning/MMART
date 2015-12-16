#include "devices/MyoWrapper.hpp"

#include <Eigen/Core>
#include <bitset>
using namespace nui::events;

const Eigen::Vector3f DEFAULT_VECTOR(0.f, 0.f, 0.f);

inline void createEvent(nui::events::MyoEvent* evt, bool stream) {
	evt->device         = MYO_EVENT;
	evt->timestamp      = timestamp_now();
	evt->currentState   = NOT_CONNECT;
	evt->processingtype = (stream) ? OUTPUT_EVENT : INTERNAL_EVENT;
	evt->currentPose    = UNKNOWN;
	evt->isUnlocked     = false;
	evt->onArm          = false;
	evt->whichArm       = NONE;
	// ------------------------------------------------------------
	evt->roll_w         = 0.f;
	evt->pitch_w        = 0.f;
	evt->yaw_w          = 0.f;
	// ------------------------------------------------------------
	evt->batteryLevel   = 0;
	evt->rssi           = 0;
	// ------------------------------------------------------------
	evt->acc_x          = 0.f;
	evt->acc_y          = 0.f;
	evt->acc_z          = 0.f;
	// ------------------------------------------------------------
	evt->gyro_x         = 0.f;
	evt->gyro_y         = 0.f;
	evt->gyro_z         = 0.f;
}

void MyoDataCollector::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
{
	boost::unique_lock<boost::mutex> lock(m_emgdata_mutex);
    for (int i = 0; i < 8; i++) {
        emgSamples[i] = emg[i];
    }
	lock.release()->unlock();
}

 void MyoDataCollector::onUnpair(myo::Myo* myo, uint64_t timestamp)
{
	boost::unique_lock<boost::mutex> lock(m_data_mutex);
    // We've lost a Myo.
    // Let's clean up some leftover state.
    roll_w = 0;
    pitch_w = 0;
    yaw_w = 0;
    onArm = false;
    isUnlocked = false;
	lock.release()->unlock();
}

void MyoDataCollector::onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
{
    using std::atan2;
    using std::asin;
    using std::sqrt;
    using std::max;
    using std::min;
    // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
    float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                        1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
    float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
    float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                    1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
    // Convert the floating point angles in radians to a scale from 0 to 18.
	boost::unique_lock<boost::mutex> lock(m_data_mutex);
    roll_w  = static_cast<int>((roll + (float)  M_PI)/ (M_PI * 2.0f) * 18);
    pitch_w = static_cast<int>((pitch + (float) M_PI / 2.0f) / M_PI * 18);
    yaw_w   = static_cast<int>((yaw + (float)  M_PI) / (M_PI * 2.0f) * 18);
	lock.release()->unlock();
}

void MyoDataCollector::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
{
	boost::unique_lock<boost::mutex> lock(m_data_mutex);
	switch(pose.type()) {
	case libmyo_pose_rest :
		currentPose = REST_GESTURE;
		break;
	case libmyo_pose_fist :
		currentPose = FIST_GESTURE;
		break;
	case libmyo_pose_wave_in :
		currentPose = WAVE_IN_GESTURE;
		break;
	case libmyo_pose_wave_out :
		currentPose = WAVE_OUT_GESTURE;
		break;
	case libmyo_pose_fingers_spread :
		currentPose = FINGERS_SPREAD_GESTURE;
		break;
	case libmyo_pose_double_tap :
		currentPose = DOUBLE_TAB_GESTURE;
		break;
			
	}
	lock.release()->unlock();
    if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
        // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
        // Myo becoming locked.
        myo->unlock(myo::Myo::unlockHold);
        // Notify the Myo that the pose has resulted in an action, in this case changing
        // the text on the screen. The Myo will vibrate.
        myo->notifyUserAction();
    } else {
        // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
        // are being performed, but lock after inactivity.
        myo->unlock(myo::Myo::unlockTimed);
    }
}

void MyoDataCollector::onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel)
{
	boost::unique_lock<boost::mutex> lock(m_data_mutex);
	acc_x = accel.x();
	acc_y = accel.y();
	acc_z = accel.z();
	lock.release()->unlock();
}

void MyoDataCollector::onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro)
{
	boost::unique_lock<boost::mutex> lock(m_data_mutex);
	gyro_x = gyro.x();
	gyro_y = gyro.y();
	gyro_z = gyro.z();
	lock.release()->unlock();
}

void MyoDataCollector::onRssi(myo::Myo* myo, uint64_t timestamp, int8_t _rssi)
{
	boost::unique_lock<boost::mutex> lock(m_data_mutex);
	rssi = _rssi;
	lock.release()->unlock();
}

void MyoDataCollector::onBatteryLevelReceived(myo::Myo* myo, uint64_t timestamp, uint8_t level)
{
	boost::unique_lock<boost::mutex> lock(m_data_mutex);
	batteryLevel = level;
	lock.release()->unlock();
}

void MyoDataCollector::onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                myo::WarmupState warmupState)
{
	boost::unique_lock<boost::mutex> lock(m_data_mutex);
	onArm = true;
	whichArm = (arm == myo::armRight) ? RIGHT : LEFT;
	lock.release()->unlock();
}

void MyoDataCollector::onArmUnsync(myo::Myo* myo, uint64_t timestamp)
{
    onArm = false;
}

void MyoDataCollector::onUnlock(myo::Myo* myo, uint64_t timestamp)
{
    isUnlocked = true;
}

void MyoDataCollector::onLock(myo::Myo* myo, uint64_t timestamp)
{
    isUnlocked = false;
}

void MyoDataCollector::enqueData(EventQueue& evtQue, MyoEvent* evt) {
	boost::unique_lock<boost::mutex> lock(m_data_mutex);
	evt->currentState = CONNECTION_GOOD;
	evt->currentPose = currentPose;
	evt->isUnlocked = isUnlocked;
	evt->onArm = onArm;
	evt->whichArm = whichArm;
	// ------------------------------------------------------------
	evt->roll_w = roll_w;
	evt->pitch_w = pitch_w;
	evt->yaw_w = yaw_w;
	// ------------------------------------------------------------
	evt->batteryLevel = batteryLevel;
	evt->rssi = rssi;
	// ------------------------------------------------------------
	evt->acc_x = acc_x;
	evt->acc_y = acc_y;
	evt->acc_z = acc_z;
	// ------------------------------------------------------------
	evt->gyro_x = gyro_x;
	evt->gyro_y = gyro_y;
	evt->gyro_z = gyro_z;
	// ------------------------------------------------------------
	lock.release()->unlock();
	boost::unique_lock<boost::mutex> lockEMG(m_emgdata_mutex);	
	for (int i = 0; i < 8; i++) {
		evt->emgSamples[i] = emgSamples[i];
    }
	lockEMG.release()->unlock();
}

void MyoWrapper::reconfigure() {
	m_gesture_setup |= (nui::config()->get("Myo.Gestures.Rest", false) ? REST_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Myo.Gestures.Rest", false))<< "Rest gesture enabled";

	m_gesture_setup |= (nui::config()->get("Myo.Gestures.Fist", false) ? FIST_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Myo.Gestures.Fist", false))<< "Fist gesture enabled";

	m_gesture_setup |= (nui::config()->get("Myo.Gestures.WaveIn", false) ? WAVE_IN_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Myo.Gestures.WaveIn", false))<< "Wave In gesture enabled";

	m_gesture_setup |= (nui::config()->get("Myo.Gestures.WaveOut", false) ? WAVE_OUT_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Myo.Gestures.WaveOut", false))<< "Wave Out gesture enabled";

	m_gesture_setup |= (nui::config()->get("Myo.Gestures.FingersSpread", false) ? FINGERS_SPREAD_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Myo.Gestures.FingersSpread", false))<< "Fingers spread gesture enabled";

	m_gesture_setup |= (nui::config()->get("Myo.Gestures.DoubleTab", false) ? DOUBLE_TAB_GESTURE : 0x0);
	LOG_IF(INFO, nui::config()->get("Myo.Gestures.DoubleTab", false))<< "Double tab gesture enabled";

	m_stream_raw = nui::config()->get("Myo.Stream.raw", false);
	LOG_IF(INFO, m_stream_raw)<< "Stream raw MYO frame.";
}

NUISTATUS MyoWrapper::initialize() {
	// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    m_hub.addListener(&m_collector);
	m_hub.setLockingPolicy(myo::Hub::lockingPolicyNone);
	return NUI_SUCCESS;
}

NUISTATUS MyoWrapper::unInitialize() {
	m_hub.removeListener(&m_collector);
	return NUI_SUCCESS;
}

void MyoWrapper::connect(nui::events::MyoEvent* evt)
{
	myo::Myo* myo = m_hub.waitForMyo(10);
	if (myo) 
	{
		m_myostate = CONNECTED;
		// Next we enable EMG streaming on the found Myo.
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
		evt->currentState = CONNECTION_ESTABLISHED;
	}
}


void MyoWrapper::process(EventQueue& evtQue) {
	// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
    // immediately.
    // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
    // if that fails, the function will return a null pointer.
	MyoEvent* evt = new MyoEvent();
	evt->id = updates();
	// configure event
	createEvent(evt, m_stream_raw);
	switch(m_myostate) {
	case CONNECTED:
		m_hub.run(1000/50);
		m_collector.enqueData(evtQue, evt);	
		break;
	case WAITING_FOR_CONNECTION:
		connect(evt);
		break;
	}
	evtQue.push(evt);
}




