/*
 * This file is part of the AV-NUI project.
 * Copyright (C) 2015 DFKI GmbH. All rights reserved.
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
#include <myo/myo.hpp>

/**
 * Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
 * provides several virtual functions for handling different kinds of events. If you do not override an event, the
 * default behavior is to do nothing.
 */
class MyoDataCollector : public myo::DeviceListener {

public:
	/**
	 * \brief Mutex for data locks
	 */
	boost::mutex m_data_mutex;
	
	/**
	 * \brief Mutex for emg data locks
	 */
	boost::mutex m_emgdata_mutex;
	
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
    nui::events::Side whichArm;
    
	/** 
	 * \brief  This is set by onUnlocked() and onLocked() above.
	 */
    bool isUnlocked;
    
	/** 
	 * \brief These values are set by onOrientationData() and onPose() above.
	 */
    int roll_w, pitch_w, yaw_w;
	
	/**
	 * \brief Battery level.
	 */
	uint8_t batteryLevel;
	
	/**
	 * \brief RSSI.
	 */
	int8_t rssi;
	
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
    nui::events::MyoPose currentPose;

	/**
	 * \brief Constructor.
	 */
    MyoDataCollector()
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose(), emgSamples()
    {
    }
    /** 
	 * \brief onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	 */
    virtual void onUnpair(myo::Myo* myo, uint64_t timestamp);

    /**  
	 * \brief onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
     * as a unit quaternion.
	 */
    virtual void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat);

    /**  
	 * \brief onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
     * making a fist, or not making a fist anymore.
	 */
    virtual void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose);

    /**  
	 * \brief onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
     * arm. This lets Myo know which arm it's on and which way it's facing.
	 */
    virtual void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState);

    /**  
	 * \brief onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
     * it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
     * when Myo is moved around on the arm.
	 */
    virtual void onArmUnsync(myo::Myo* myo, uint64_t timestamp);

	/**  
	 * \brief Called when a paired Myo has provided new EMG data.
	 */
	virtual void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg);

    /** 
	 * \brief onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
	 */
    virtual void onUnlock(myo::Myo* myo, uint64_t timestamp);

    /**  
	 * \brief onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
	 */
    virtual void onLock(myo::Myo* myo, uint64_t timestamp);

	/**  
	 * \brief Called when a paired Myo has provided new accelerometer data in units of g.
	 */
	virtual void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel);

	/**
	 * \brief Called when a paired Myo has provided new gyroscope data in units of deg/s.
	 */
	virtual void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro);

	/**
	 * \brief Called when a paired Myo has provided a new RSSI value
	 */
    virtual void onRssi(myo::Myo* myo, uint64_t timestamp, int8_t rssi);

	/**
	 * \brief Called when a paired Myo receives an battery level update.
	 */
    virtual void onBatteryLevelReceived(myo::Myo* myo, uint64_t timestamp, uint8_t level);

  	/**
	 * \brief Grabbing data from collector and trigger event.
	 * \param queue - event queue
	 * \param evt   - myo event to configure
	 */
	void enqueData(EventQueue&,  nui::events::MyoEvent*);
};

/**
 * \brief Interface for hand tracking of LEAP Motion device.
 */
class MYO_EXPORT MyoWrapper: public IModality {
public:

	typedef enum InternalState {
		WAITING_FOR_CONNECTION,
		CONNECTED
	} InternalState;

	MyoWrapper() : 
			IModality(MYO_MODALITY, SAMPLING_50_HZ), m_gesture_setup(0x0), m_hub("de.dfki.myo"), m_myostate(WAITING_FOR_CONNECTION) {
		reconfigure();
	};
	virtual ~MyoWrapper() {

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

	/** Gesture setup. */
	GestureSetup m_gesture_setup;
	/** 
	 * Collector for Myo data.
	 */
	MyoDataCollector m_collector;
	/**
	 * \brief State of Myo.
	 */
	InternalState m_myostate;
	/**
	 * \brief Hub.
	 */
	myo::Hub m_hub;
	/**
	 * Stream raw tracking data.
	 */
	bool m_stream_raw;
	/**
	 * Flag if streams should be streamed.
	 */
	bool m_stream_gestures;
	
	/**
	 * \brief Connects Myo device and handles the internal state.
	 */
	void connect(nui::events::MyoEvent* );

	/**
	 * \brief No hand tracked event.
	 */
	void createNoneEvent(EventQueue&);
};
