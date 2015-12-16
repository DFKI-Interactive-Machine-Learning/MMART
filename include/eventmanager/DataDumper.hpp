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
#include "eventmanager/SimpleEventListener.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <queue>
#include <map>
#include <string>
#include <stdint.h>
#include <opencv2/core/core.hpp>


/**
 * \brief Dumping mechanism for data.
 */
class EVENTMANAGER_EXPORT DataDumper: public ISimpleEventListener {
public:
	DataDumper() :
		    m_dump_dir("."), m_realsenseCacheInterval(0),
			m_dumper_thread(NULL), ISimpleEventListener(DUMPER_MODULE) {
		reconfigure();
		nui::config()->registerForReconfigure("General.Dump",
				boost::bind(&DataDumper::reconfigure, this));
	};
	virtual ~DataDumper();
	virtual void reconfigure();
protected:
	virtual void process(nui::events::LeapActionGestureEvent&);
	virtual void process(nui::events::HandPostureGestureEvent&);
	virtual void process(nui::events::CircleGestureEvent&);
	virtual void process(nui::events::LeapTrackingEvent&);
	virtual void process(nui::events::SwipeGestureEvent&);
	virtual void process(nui::events::KeyTapGestureEvent&);
	virtual void process(nui::events::ScreenTapGestureEvent&);
	virtual void process(nui::events::ScreenPointerEvent&);
	virtual void process(nui::events::CamEvent&);
    virtual void process(nui::events::UserAttentionEvent&);
    virtual void process(nui::events::PMDNanoEvent&);
	virtual void process(nui::events::RealSenseEvent&);
	virtual void process(nui::events::DepthSenseEvent&);
	virtual void process(nui::events::MyoEvent&);
	/** ----------------------- Virtual methods ----------------------- */
	virtual NUISTATUS initialize();
	virtual NUISTATUS unInitialize();
	/**
	 * \brief Writes a session summary file.
	 */
	void writeSessionSummary();
	/**
	 * \brief Generates HDF5 session files.
	 */
	void generateSessionFiles();
private:
	/** \brief Configuration values. */
	std::map<std::string, bool> m_configuration;

	/** \brief Pointer to thread. */
	boost::thread* m_dumper_thread;

	/** \brief Mutex for locks  */
	boost::mutex m_dumper_mutex;
	/**
	 * \brief Dump step for color frames.
	 */
	uint32_t m_colorFrameDumpStep;
	/**
	 * \brief Dump counter.
	 */
	std::map<std::string, ushort> m_counter;
	/**
	 * \brief Statistics counter.
	 */
	std::map<std::string, size_t> m_statistics;
	/**
	 * \brief Path where to dumped data.
	 */
	boost::filesystem::path m_dump_dir;
	/**
	 * \brief Path to session dir.
	 */
	boost::filesystem::path m_session_dir;
	/**
	 * \brief Path to session images.
	 */
	boost::filesystem::path m_session_images_dir;

	/**
	 * \brief Path to session images.
	 */
	boost::filesystem::path m_session_pmdnano_dir;
	/**
	 * \brief Path to session amplitude images.
	 */
	boost::filesystem::path m_session_pmdnano_amp_dir;
	/**
	 * \brief Path to session depth images.
	 */
	boost::filesystem::path m_session_pmdnano_depth_dir;
	/**
	 * \brief Path to session point cloud.
	 */
	boost::filesystem::path m_session_pmdnano_pcloud_dir;
		/**
	 * \brief Path to session images.
	 */
	boost::filesystem::path m_session_depthsense_dir;
	/**
	 * \brief Path to session point cloud.
	 */
	boost::filesystem::path m_session_depthsense_pcloud_dir;
	/**
	 * \brief Path to color.
	 */
	boost::filesystem::path m_session_depthsense_color_dir;
	/**
	 * \brief Path to depth.
	 */
	boost::filesystem::path m_session_depthsense_depth_dir;
	/**
	 * \brief Path to session realsense data .
	 */
	boost::filesystem::path m_session_realsense_dir;
	/**
	 * \brief Path to session point cloud.
	 */
	boost::filesystem::path m_session_realsense_pcloud_dir;
	/**
	 * \brief Path to color.
	 */
	boost::filesystem::path m_session_realsense_color_dir;
	/**
	 * \brief Path to depth.
	 */
	boost::filesystem::path m_session_realsense_depth_dir;
	/**
	 * \brief Path to ir images.
	 */
	boost::filesystem::path m_session_realsense_ir_dir;
	/**
	 * \brief Path to myo dir.
	 */
	boost::filesystem::path m_session_myo_dir;
	/**
	 * \brief Path to Leap dir.
	 */
	boost::filesystem::path m_session_leap_dir;
    /**
	 * \brief Queue of myo events.
	 */
	std::queue<nui::events::MyoEvent> m_myo_queue;
    boost::mutex m_myo_queue_mutex;
	/**
	 * \brief Queue of leap events.
	 */
	std::queue<nui::events::LeapTrackingEvent> m_leap_queue;
    boost::mutex m_leap_queue_mutex;
	/**
	 * \brief Queue of leap gestures.
	 */
    std::queue<nui::events::LeapActionGestureEvent> m_leap_gesture_queue_action;
    boost::mutex m_leap_gesture_queue_action_mutex;
	/**
	 * \brief Queue of circle gestures.
	 */
    std::queue<nui::events::CircleGestureEvent> m_leap_gesture_queue_circle;
    boost::mutex m_leap_gesture_queue_circle_mutex;
	/**
	 * \brief Queue of swipe gestures.
	 */
    std::queue<nui::events::SwipeGestureEvent> m_leap_gesture_queue_swipe;
    boost::mutex m_leap_gesture_queue_swipe_mutex;
	/**
	 * \brief Queue of key tap gestures.
	 */
    std::queue<nui::events::KeyTapGestureEvent> m_leap_gesture_queue_key_tap;
    boost::mutex m_leap_gesture_queue_key_tap_mutex;
	/**
	 * \brief Queue of screen tap gestures.
	 */
    std::queue<nui::events::ScreenTapGestureEvent> m_leap_gesture_queue_screen_tap;
    boost::mutex m_leap_gesture_queue_screen_tap_mutex;
	/**
	 * \brief Queue of PMD Nano frames.
	 */
    std::queue<nui::events::PMDNanoEvent> m_pmd_nano_queue;
    boost::mutex m_pmd_nano_queue_mutex;
	/**
	 * \brief Queue of RealSense frames.
	 */
    std::queue<nui::events::RealSenseEvent> m_realsense_queue;
    boost::mutex m_realsense_queue_mutex;
		/**
	 * \brief Queue of PMD Nano frames.
	 */
    std::queue<nui::events::DepthSenseEvent> m_depthsense_queue;
    boost::mutex m_depthsense_queue_mutex;

	timestamp_t m_realsense_lastupdate;
	long m_realsenseCacheInterval;
	/**
	 * \brief Queue of video images.
	 */
    std::queue<cv::Mat> m_cam_0_queue;
    std::queue<time_t> m_cam_0_timestamp_queue;
    boost::mutex m_cam_0_queue_mutex;

    std::queue<cv::Mat> m_cam_1_queue;
    std::queue<time_t> m_cam_1_timestamp_queue;
    boost::mutex m_cam_1_queue_mutex;

	 /**
     * \brief Session files.
     */
	std::queue<bfs::path> m_myo_session_cache_queue;
    /**
     * \brief Queue of attention events.
     */
    std::queue<nui::events::UserAttentionEvent> m_attention_queue;
    boost::mutex m_attention_queue_mutex;

	/**
	 * \brief Renders the video frames.
	 */
	void dumper();
};

