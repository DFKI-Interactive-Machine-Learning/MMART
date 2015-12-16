#include "eventmanager/DataDumper.hpp"
#include "serialize/HDF5Serializer.hpp"
#include "serialize/GPufferSerializer.hpp"
#include "serialize/PointCloudSerializer.hpp"
#include "serialize/OpenCVSerializer.hpp"
#include "core/common.hpp"
#include "core/errorcodes.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <cv.h>
#include <fstream>

#define DUMP_SIZE 1000
using namespace nui::events;
namespace bfs = boost::filesystem;

/** Helper method. */
static bfs::path relativeTo(bfs::path from, bfs::path to)
{
   // Start at the root path and while they are the same then do nothing then when they first
   // diverge take the remainder of the two path and replace the entire from path with ".."
   // segments.
   bfs::path::const_iterator fromIter = from.begin();
   bfs::path::const_iterator toIter = to.begin();
   // Loop through both
   while (fromIter != from.end() && toIter != to.end() && (*toIter) == (*fromIter))
   {
      ++toIter;
      ++fromIter;
   }

   bfs::path finalPath;
   while (fromIter != from.end())
   {
      finalPath /= "..";
      ++fromIter;
   }

   while (toIter != to.end())
   {
      finalPath /= *toIter;
      ++toIter;
   }
   return finalPath;
}

void DataDumper::reconfigure()
{
	BOOST_LOG_TRIVIAL(info)<< "Reconfiguring DataDumber...";
	m_dump_dir = boost::filesystem::absolute(nui::config()->getString("General.Dump.DumpDir", "."));
	BOOST_LOG_TRIVIAL(info) << "Dumping data to folder " << m_dump_dir;
	m_configuration["webcam"]          = nui::config()->get("WebCam.Activated", true) &&  nui::config()->get("General.Dump.WebCam.Dump", true);
	m_configuration["pmd"]             = nui::config()->get("PMDNano.Activated",  true);
	m_configuration["pmd-amplitude"]   = m_configuration["pmd"] && nui::config()->get("General.Dump.PMDNano.DumpAmplitude",  true);
	m_configuration["pmd-depth"]       = m_configuration["pmd"] && nui::config()->get("General.Dump.PMDNano.DumpDepth",      true);
	m_configuration["pmd-cloud"]       = m_configuration["pmd"] && nui::config()->get("General.Dump.PMDNano.DumpPointCloud", true);
	m_configuration["realsense"]       = nui::config()->get("RealSense.Activated", true);
	m_configuration["realsense-cloud"] = m_configuration["realsense"]  && nui::config()->get("General.Dump.Realsense.DumpPointCloud", true);
	m_configuration["realsense-color"] = m_configuration["realsense"]  && nui::config()->get("General.Dump.Realsense.DumpColor", true);
	m_configuration["realsense-depth"] = m_configuration["realsense"]  && nui::config()->get("General.Dump.Realsense.DumpDepth", true);
	m_configuration["realsense-ir"]    = m_configuration["realsense"]  && nui::config()->get("General.Dump.Realsense.DumpIR", true);
	m_configuration["depthsense"]       = nui::config()->get("DepthSense.Activated", true);
	m_configuration["depthsense-cloud"] = nui::config()->get("General.Dump.Depthsense.DumpPointCloud", true);
	m_configuration["depthsense-color"] = nui::config()->get("General.Dump.Depthsense.DumpColor", true);
	m_configuration["depthsense-depth"] = nui::config()->get("General.Dump.Depthsense.DumpDepth", true);
	m_configuration["myo"]             = nui::config()->get("Myo.Activated", true)  && nui::config()->get("General.Dump.Myo.Dump", true);
	m_configuration["leap"]            = nui::config()->get("Leap.Activated", true) && nui::config()->get("General.Dump.Leap.Dump", true);
	int fps                            = nui::config()->get("General.Dump.Realsense.CacheFramerate", 10);
	// --------------------------------------- Cache counter ------------------------------------------------
	m_counter["leap-motion"]         = 0;
	m_counter["myo"]                 = 0;
	m_counter["leap-motion-gesture"] = 0;
	m_counter["attention"]           = 0;
	// ---------------------------------------- Statistics --------------------------------------------------
	m_statistics["leap"]                = 0;
	m_statistics["myo"]                 = 0;
	m_statistics["pmd"]                 = 0;
	m_statistics["leap-motion-gesture"] = 0;
	m_statistics["attention"]           = 0;
	m_statistics["realsense"]           = 0;
	m_statistics["depthsense"]          = 0;
	// ------------------------------------------------------------------------------------------------------
	m_realsenseCacheInterval = 1000 / fps;
	m_realsense_lastupdate = boost::posix_time::microsec_clock::universal_time(); 
	m_session_dir                   = m_dump_dir / date_string();
	m_session_images_dir            = m_session_dir /"webcam";
	m_session_pmdnano_dir           = m_session_dir /"pmd";
	m_session_pmdnano_amp_dir       = m_session_pmdnano_dir /"amplitude";
	m_session_pmdnano_depth_dir     = m_session_pmdnano_dir /"depth";
	m_session_pmdnano_pcloud_dir    = m_session_pmdnano_dir /"pointcloud";
	m_session_realsense_dir         = m_session_dir / "realsense";
	m_session_realsense_pcloud_dir  = m_session_realsense_dir / "pointcloud";
	m_session_realsense_color_dir   = m_session_realsense_dir / "color";
	m_session_realsense_depth_dir   = m_session_realsense_dir / "depth";
	m_session_realsense_ir_dir      = m_session_realsense_dir / "ir";
	m_session_depthsense_dir        = m_session_dir / "depthsense";
	m_session_depthsense_pcloud_dir = m_session_depthsense_dir / "pointcloud";
	m_session_depthsense_color_dir  = m_session_depthsense_dir / "color";
	m_session_depthsense_depth_dir  = m_session_depthsense_dir / "depth";
	m_session_myo_dir               = m_session_dir / "myo";
	m_session_leap_dir              = m_session_dir / "leap-motion";
	BOOST_LOG_TRIVIAL(info) << "Dumping session to folder " << m_session_dir;
	// configuration of serializer
	reconfigure_serializer(nui::config());
}
DataDumper::~DataDumper()
{
	PtrRelease(m_dumper_thread);
}

NUISTATUS DataDumper::initialize()
{
	NUISTATUS st = NUI_SUCCESS;
	if (!is_directory(m_session_dir))
	{
		if (!create_directories(m_session_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		} 
	}
	if (m_configuration["webcam"] && !is_directory(m_session_images_dir))
	{
		if (!create_directories(m_session_images_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["pmd-amplitude"] && !is_directory(m_session_pmdnano_amp_dir))
	{
		if (!create_directories(m_session_pmdnano_amp_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["pmd-depth"] && !is_directory(m_session_pmdnano_depth_dir))
	{
		if (!create_directories(m_session_pmdnano_depth_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["pmd-cloud"] && !is_directory(m_session_pmdnano_pcloud_dir))
	{
		if (!create_directories(m_session_pmdnano_pcloud_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["realsense-cloud"] && !is_directory(m_session_realsense_pcloud_dir))
	{
		if (!create_directories(m_session_realsense_pcloud_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["realsense-color"] && !is_directory(m_session_realsense_color_dir))
	{
		if (!create_directories(m_session_realsense_color_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["realsense-depth"] && !is_directory(m_session_realsense_depth_dir))
	{
		if (!create_directories(m_session_realsense_depth_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["realsense-ir"] && !is_directory(m_session_realsense_ir_dir))
	{
		if (!create_directories(m_session_realsense_ir_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["depthsense-cloud"] && !is_directory(m_session_depthsense_pcloud_dir))
	{
		if (!create_directories(m_session_depthsense_pcloud_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["depthsense-color"] && !is_directory(m_session_depthsense_color_dir))
	{
		if (!create_directories(m_session_depthsense_color_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["depthsense-depth"] && !is_directory(m_session_depthsense_depth_dir))
	{
		if (!create_directories(m_session_depthsense_depth_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["myo"] && !is_directory(m_session_myo_dir))
	{
		if (!create_directories(m_session_myo_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	if (m_configuration["leap"] && !is_directory(m_session_leap_dir))
	{
		if (!create_directories(m_session_leap_dir))
		{
			return ERR_FAILED_TO_CREATE_DIRECTORY;
		}
	}
	m_dumper_thread = new boost::thread(boost::bind(&DataDumper::dumper, this));
	return st;
}

NUISTATUS DataDumper::unInitialize()
{
	if (m_dumper_thread)
		m_dumper_thread->join();
	return NUI_SUCCESS;
}


void DataDumper::process(LeapTrackingEvent& evt)
{
	m_leap_queue_mutex.lock();
	m_leap_queue.push(evt);
	m_statistics["leap"] += 1;
	m_leap_queue_mutex.unlock();
}

void DataDumper::process(HandPostureGestureEvent& evt)
{

}

void DataDumper::process(nui::events::LeapActionGestureEvent& evt)
{
	m_leap_gesture_queue_action_mutex.lock();
	m_leap_gesture_queue_action.push(evt);
	m_leap_gesture_queue_action_mutex.unlock();
}

void DataDumper::process(CircleGestureEvent& evt)
{
	m_leap_gesture_queue_circle_mutex.lock();
	m_leap_gesture_queue_circle.push(evt);
	m_leap_gesture_queue_circle_mutex.unlock();
}

void DataDumper::process(SwipeGestureEvent& evt)
{
	m_leap_gesture_queue_swipe_mutex.lock();
	m_leap_gesture_queue_swipe.push(evt);
	m_leap_gesture_queue_swipe_mutex.unlock();
}

void DataDumper::process(KeyTapGestureEvent& evt)
{
	m_leap_gesture_queue_key_tap_mutex.lock();
	m_leap_gesture_queue_key_tap.push(evt);
	m_leap_gesture_queue_key_tap_mutex.unlock();
}

void DataDumper::process(ScreenTapGestureEvent& evt)
{
	m_leap_gesture_queue_screen_tap_mutex.lock();
	m_leap_gesture_queue_screen_tap.push(evt);
	m_leap_gesture_queue_screen_tap_mutex.unlock();
}

void DataDumper::process(nui::events::PMDNanoEvent& evt)
{
	m_pmd_nano_queue_mutex.lock();
	m_pmd_nano_queue.push(evt);
	m_statistics["pmd"] += 1;
	m_pmd_nano_queue_mutex.unlock();
}

void DataDumper::process(nui::events::DepthSenseEvent& evt)
{
	m_depthsense_queue_mutex.lock();
	m_depthsense_queue.push(evt);
	m_statistics["depthsense"] += 1;
	m_depthsense_queue_mutex.unlock();
}

void DataDumper::process(nui::events::RealSenseEvent& evt)
{
	timestamp_t tEvent =  boost::posix_time::from_time_t(evt.event_timestamp());
	duration_t update_time = tEvent - m_realsense_lastupdate;
	m_realsense_queue_mutex.lock();
	m_realsense_queue.push(evt);
	m_statistics["realsense"] += 1;
	m_realsense_lastupdate = tEvent; 
	m_realsense_queue_mutex.unlock();

}

void DataDumper::process(ScreenPointerEvent& evt)
{
	//TODO
}


void DataDumper::process(MyoEvent& evt)
{
	m_myo_queue_mutex.lock();
	m_myo_queue.push(evt);
	m_statistics["myo"] += 1;
	m_myo_queue_mutex.unlock();
}

void DataDumper::process(CamEvent& evt)
{
	if (evt.id == 0)
	{
		m_cam_0_queue_mutex.lock();
		m_cam_0_queue.push(evt.frame.clone());
		m_cam_0_timestamp_queue.push(evt.timestamp);
		m_cam_0_queue_mutex.unlock();
	}
	else
	{
		m_cam_1_queue_mutex.lock();
		m_cam_1_queue.push(evt.frame.clone());
		m_cam_1_timestamp_queue.push(evt.timestamp);
		m_cam_1_queue_mutex.unlock();
	}
}

void DataDumper::process(nui::events::UserAttentionEvent& evt)
{
	m_attention_queue_mutex.lock();
	m_attention_queue.push(evt);
	m_attention_queue_mutex.unlock();
}

void DataDumper::dumper()
{
	bool run = true;
	bool dumped = false;
	while (run)
	{
		// ------------------------------ Save image events -----------------------------
		if (!m_cam_0_timestamp_queue.empty())
		{
			std::stringstream fname;
			cv::Mat color_frame;
			m_cam_0_queue_mutex.lock();
			dumped = !m_cam_0_queue.empty();
			time_t t = m_cam_0_timestamp_queue.front();
			m_cam_0_timestamp_queue.pop();
			if (dumped)
			{
				color_frame = m_cam_0_queue.front();
				m_cam_0_queue.pop();
			}
			m_cam_0_queue_mutex.unlock();
			fname << "0_" << t << ".png";
			if (dumped)
			{
				cv::imwrite((m_session_images_dir / fname.str()).string(),
						color_frame);
			}

		}
		if (!m_cam_1_timestamp_queue.empty())
		{
			std::stringstream fname;
			cv::Mat color_frame;
			m_cam_1_queue_mutex.lock();
			dumped = !m_cam_1_queue.empty();
			time_t t = m_cam_1_timestamp_queue.front();
			m_cam_1_timestamp_queue.pop();
			if (dumped)
			{
				color_frame = m_cam_1_queue.front();
				m_cam_1_queue.pop();
			}
			m_cam_1_queue_mutex.unlock();
			fname << "1_" << t << ".png";
			if (dumped)
			{
				cv::imwrite((m_session_images_dir / fname.str()).string(),
						color_frame);
			}

		}
		// ------------------------------ PMD Nano events -------------------------------
		if (!m_pmd_nano_queue.empty())
		{
			std::stringstream fname;
			PMDNanoEvent evt;
			m_pmd_nano_queue_mutex.lock();
			evt = m_pmd_nano_queue.front();
			m_pmd_nano_queue.pop();
			m_pmd_nano_queue_mutex.unlock();
			dumped = true;
			// file name
			fname << evt.timestamp << ".png";
			if (m_configuration["pmd-amplitude"])
			{
				// Amplitude
				cv::imwrite((m_session_pmdnano_amp_dir / fname.str()).string(),
						evt.amplitude);
			}
			if (m_configuration["pmd-depth"])
			{
				// Depth
				cv::imwrite((m_session_pmdnano_depth_dir / fname.str()).string(),
						evt.depth);
			}
			// Point cloud
			if (m_configuration["pmd-cloud"])
			{
				fname.str( std::string() );
				fname.clear();
				fname << evt.timestamp << ".RAW.pcd";
				pcl_serialize(evt.cloud,
					m_session_pmdnano_pcloud_dir / fname.str());
				fname.str( std::string() );
				fname.clear();
				fname << evt.timestamp << ".FILTERED.pcd";
				pcl_serialize(evt.filteredCloud,
					m_session_pmdnano_pcloud_dir / fname.str());
			}
		}
		// ------------------------------ Save Realsense events ------------------------
		// Point cloud
		if (!m_realsense_queue.empty())
		{
			std::stringstream fname;
			RealSenseEvent evt;
			m_realsense_queue_mutex.lock();
			evt = m_realsense_queue.front();
			m_realsense_queue.pop();
			m_realsense_queue_mutex.unlock();
			if (m_configuration["realsense-cloud"])
			{
				fname.str( std::string() );
				fname.clear();
				fname << evt.timestamp << ".RGB.compressed.pcd";
				pcl_serialize(evt.cloud,
					m_session_realsense_pcloud_dir / fname.str());
				dumped = true;
				BOOST_LOG_TRIVIAL(info) << "Data dumped. [queue:=" << I2Str(m_realsense_queue.size()) << "]";
			} 
			if (m_configuration["realsense-color"])
			{
				fname.str( std::string() );
				fname.clear();
				fname << evt.timestamp << ".png";
				cv::imwrite((m_session_realsense_color_dir / fname.str()).string(),
						evt.rgb);
				dumped = true;
			}
			if (m_configuration["realsense-depth"])
			{
				fname.str( std::string() );
				fname.clear();
				fname << evt.timestamp;
				// Save png image
				opencv_serialize_depth(evt.depth, (m_session_realsense_depth_dir / fname.str()).string() + ".png");
				// Save depth map image
				save_depth_pnm(evt.depth, (m_session_realsense_depth_dir / fname.str()).string() + ".pnm");
				dumped = true;
			}
			if (m_configuration["realsense-ir"])
			{
				fname.str( std::string() );
				fname.clear();
				fname << evt.timestamp << ".png";
				/*cv::imwrite((m_session_realsense_ir_dir / fname.str()).string(),
						evt.ir);*/
				dumped = true;
			}
		}
		// ------------------------------ Save DepthSense events ------------------------
		// Point cloud
		if (!m_depthsense_queue.empty())
		{
			std::stringstream fname;
			DepthSenseEvent evt;
			m_depthsense_queue_mutex.lock();
			evt = m_depthsense_queue.front();
			m_depthsense_queue.pop();
			m_depthsense_queue_mutex.unlock();
			if (m_configuration["depthsense-cloud"])
			{
				fname.str( std::string() );
				fname.clear();
				fname << evt.timestamp << ".RGB.compressed.pcd";
				pcl_serialize(evt.cloud,
					m_session_depthsense_pcloud_dir / fname.str());
				dumped = true;
				BOOST_LOG_TRIVIAL(info) << "Data dumped. [queue:=" << I2Str(m_depthsense_queue.size()) << "]";
			} 
			if (m_configuration["depthsense-color"])
			{
				fname.str( std::string() );
				fname.clear();
				fname << evt.timestamp << ".png";
				cv::imwrite((m_session_depthsense_color_dir / fname.str()).string(),
						evt.rgb);
				dumped = true;
			}
			if (m_configuration["depthsense-depth"])
			{
				fname.str( std::string() );
				fname.clear();
				fname << evt.timestamp;
				// Save png image
				opencv_serialize_depth(evt.depth, (m_session_depthsense_depth_dir / fname.str()).string() + ".png");
				// Save depth map image
				save_depth_pnm(evt.depth, (m_session_depthsense_depth_dir / fname.str()).string() + ".pnm");
				dumped = true;
			}
		}
		// ------------------------------ Save Leap events ------------------------------
		if (!m_leap_queue.empty())
		{
			m_leap_queue_mutex.lock();
			if (m_leap_queue.size() > DUMP_SIZE || m_state == STOPPED)
			{

				std::stringstream ss;
				ss << "leap-motion." << std::setw(4) << std::setfill('0')
						<< m_counter["leap-motion"] << ".cache";
				gbuff_serialize(m_leap_queue, m_session_dir / ss.str());
				m_counter["leap-motion"] += 1;
				dumped = true;
			}
			m_leap_queue_mutex.unlock();
		}
		// ------------------------------ Save Myo events ----------------------------------
		if (!m_myo_queue.empty())
		{
			m_myo_queue_mutex.lock();
			size_t size = m_myo_queue.size();
			if (size > DUMP_SIZE || m_state == STOPPED)
			{
				std::stringstream ss;
				ss << "myo." << m_counter["myo"] <<".cache";
				bfs::path cFile = m_session_myo_dir / ss.str();
				gbuff_serialize(m_myo_queue,
						cFile);
				m_counter["myo"] += 1;
				m_myo_session_cache_queue.push(cFile);
			}
			m_myo_queue_mutex.unlock();
		}
		// ------------------------------ Save Gesture events ------------------------------
		//action gestures
		if (!m_leap_gesture_queue_action.empty())
		{
			m_leap_gesture_queue_action_mutex.lock();
			size_t size = m_leap_gesture_queue_action.size();
			if (size > DUMP_SIZE || m_state == STOPPED)
			{
				std::stringstream ss;
				ss << "leap-motion.gestures." << std::setw(4)
						<< std::setfill('0') << m_counter["leap-motion"] << ".cache";
				gbuff_serialize(m_leap_gesture_queue_action,
						m_session_dir / ss.str());
				m_counter["leap-motion"] += 1;
			}
			m_leap_gesture_queue_action_mutex.unlock();
		}
		//circle gestures
		if (!m_leap_gesture_queue_circle.empty())
		{
			m_leap_gesture_queue_circle_mutex.lock();
			size_t size = m_leap_gesture_queue_circle.size();
			if (size > DUMP_SIZE || m_state == STOPPED)
			{
				std::stringstream ss;
				ss << "leap-motion.gestures." << std::setw(4)
						<< std::setfill('0') << m_counter["leap-motion-gesture"] << ".cache";
				gbuff_serialize(m_leap_gesture_queue_circle,
						m_session_dir / ss.str());
				m_counter["leap-motion-gesture"] += 1;
			}
			m_leap_gesture_queue_circle_mutex.unlock();
		}
		//swipe
		if (!m_leap_gesture_queue_swipe.empty())
		{
			m_leap_gesture_queue_swipe_mutex.lock();
			size_t size = m_leap_gesture_queue_swipe.size();
			if (size > DUMP_SIZE || m_state == STOPPED)
			{
				std::stringstream ss;
				ss << "leap-motion.gestures." << std::setw(4)
						<< std::setfill('0') << m_counter["leap-motion-gesture"] << ".cache";
				gbuff_serialize(m_leap_gesture_queue_swipe,
						m_session_dir / ss.str());
				m_counter["leap-motion-gesture"] += 1;
			}
			m_leap_gesture_queue_swipe_mutex.unlock();
		}
		//key tap
		if (!m_leap_gesture_queue_key_tap.empty())
		{
			m_leap_gesture_queue_key_tap_mutex.lock();
			size_t size = m_leap_gesture_queue_key_tap.size();
			if (size > DUMP_SIZE || m_state == STOPPED)
			{
				std::stringstream ss;
				ss << "leap-motion.gestures." << std::setw(4)
						<< std::setfill('0') << m_counter["leap-motion-gesture"] << ".cache";
				gbuff_serialize(m_leap_gesture_queue_key_tap,
						m_session_dir / ss.str());
				m_counter["leap-motion-gesture"] += 1;
			}
			m_leap_gesture_queue_key_tap_mutex.unlock();
		}
		//screen tap
		if (!m_leap_gesture_queue_screen_tap.empty())
		{
			m_leap_gesture_queue_screen_tap_mutex.lock();
			size_t size = m_leap_gesture_queue_screen_tap.size();
			if (size > DUMP_SIZE || m_state == STOPPED)
			{
				std::stringstream ss;
				ss << "leap-motion.gestures." << std::setw(4)
						<< std::setfill('0') << m_counter["leap-motion-gesture"] << ".cache";
				gbuff_serialize(m_leap_gesture_queue_screen_tap,
						m_session_dir / ss.str());
				m_counter["leap-motion-gesture"] += 1;
			}
			m_leap_gesture_queue_screen_tap_mutex.unlock();
		}
		// ------------------------------ Save Attention events -----------------------------
		if (!m_attention_queue.empty())
		{
			m_attention_queue_mutex.lock();
			if (m_attention_queue.size() > DUMP_SIZE || m_state == STOPPED)
			{

				std::stringstream ss;
				ss << "attention-events." << std::setw(4) << std::setfill('0')
						<< m_counter["attention"] << ".cache";
				gbuff_serialize(m_attention_queue, m_session_dir / ss.str());
				m_counter["attention"] += 1;

			}
			m_attention_queue_mutex.unlock();

		}
		if (!dumped)
		{
			SLEEP(100);
		}
		dumped = false;
		// EXIT condition: queue has to be empty and the state should be stopped
		run = !(m_state == STOPPED && m_cam_0_timestamp_queue.empty()
				&& m_cam_1_timestamp_queue.empty() 
				&& m_leap_queue.empty()
				&& m_myo_queue.empty()
				&& m_leap_gesture_queue_action.empty()
				&& m_leap_gesture_queue_circle.empty()
				&& m_leap_gesture_queue_swipe.empty()
				&& m_leap_gesture_queue_key_tap.empty()
				&& m_leap_gesture_queue_screen_tap.empty()
				&& m_attention_queue.empty()
				&& m_pmd_nano_queue.empty()
				&& m_realsense_queue.empty()
				&& m_depthsense_queue.empty());
	}
	// Writes the summary of session 
	writeSessionSummary();
	// Writes the session files in HDF5 format
	generateSessionFiles();
	BOOST_LOG_TRIVIAL(info)<< "Data dump finished";
}

void DataDumper::writeSessionSummary()
{
	boost::property_tree::ptree root, device, depthCam;
	if(m_configuration["leap"])
	{
		boost::property_tree::ptree leap;
		leap.put<std::string>("type", "leap");
		leap.put<std::string>("sessionfile", relativeTo(m_session_dir, m_session_leap_dir / "leap.h5").string());
		device.push_back(std::make_pair("", leap));
		
	}
	if(m_configuration["myo"])
	{
		boost::property_tree::ptree myo;
		myo.put<std::string>("type", "myo");
		myo.put<std::string>("sessionfile", relativeTo(m_session_dir, m_session_myo_dir / "myo.h5").string());
		myo.put<size_t>("samples", m_statistics["myo"]);
		device.push_back(std::make_pair("", myo));
	}
	root.put_child("device", device);
	if(m_configuration["realsense"])
	{
		boost::property_tree::ptree realsense;
		realsense.put<std::string>("type", "realsense");
		realsense.put<std::string>("depth", relativeTo(m_session_dir, m_session_realsense_depth_dir).string());
		realsense.put<std::string>("color", relativeTo(m_session_dir, m_session_realsense_color_dir).string());
		realsense.put<size_t>("samples", m_statistics["realsense"]);
		depthCam.push_back(std::make_pair("", realsense));
	}
	if(m_configuration["pmd"])
	{
		boost::property_tree::ptree pmd;
		pmd.put<std::string>("type", "pmd");
		pmd.put<std::string>("depth", relativeTo(m_session_dir, m_session_pmdnano_depth_dir).string());
		pmd.put<std::string>("amplitude", relativeTo(m_session_dir, m_session_pmdnano_amp_dir).string());
		pmd.put<std::string>("cloud", relativeTo(m_session_dir, m_session_pmdnano_pcloud_dir).string());

		depthCam.push_back(std::make_pair("", pmd));
	}
	if (m_configuration["depthsense"])
	{
		boost::property_tree::ptree depthsense;
		depthsense.put<std::string>("type", "depthsense");
		depthsense.put<std::string>("depth", relativeTo(m_session_dir, m_session_depthsense_color_dir).string());
		depthsense.put<std::string>("color", relativeTo(m_session_dir, m_session_depthsense_depth_dir).string());
		depthsense.put<size_t>("samples", m_statistics["depthsense"]);
		depthCam.push_back(std::make_pair("", depthsense));
	}
	root.put_child("depth-camera", depthCam);
	if(m_configuration["webcam"])
	{
		boost::property_tree::ptree webcam;
		webcam.put<std::string>("type", "rbg-camera");
		webcam.put<std::string>("color", relativeTo(m_session_dir, m_session_images_dir).string());
	}
	std::ofstream os((m_session_dir / "recording.session").string());
	write_json(os, root);
	os.close();
}


void DataDumper::generateSessionFiles()
{
	// ---------------------------------- Copy myo frames ------------------------------------
	hdf5_gbuff_cache_files(m_myo_session_cache_queue, (m_session_myo_dir / "myo.h5"));
}
