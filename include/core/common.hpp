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

#define QT_NO_KEYWORDS
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	#pragma warning( disable : 4005 ) // ignore warning C4005: macro redefinition 
	#define WIN32_LEAN_AND_MEAN 
	#ifdef _UNICODE
	#if defined _M_IX86
	#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
	#elif defined _M_X64
	#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
	#else
	#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
	#endif
	#endif
	#include <windows.h>
#endif

#define BOOST_HAS_STDINT_H
#include <boost/property_tree/ptree.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
#ifdef _WIN32 
	typedef HRESULT NUISTATUS;
#else
	typedef long NUISTATUS;
#endif
static const NUISTATUS NUI_SUCCESS = 0L;
static const NUISTATUS NUI_FAILED = -1L;
#define NUI_SUCCEEDED(STATUS) STATUS == NUI_SUCCESS
#define NUI_FAILED(STATUS) STATUS != NUI_SUCCESS

/** Config */
#include "config/NUIconfig.hpp"

/** Typedef for configuration of NUI. */
typedef boost::property_tree::ptree BodyPoseXML;
typedef boost::posix_time::ptime timestamp_t;
typedef boost::posix_time::time_duration duration_t;
static const timestamp_t T0(boost::gregorian::date(1970, boost::gregorian::Jan, 1));
#ifdef _WIN32 
	#ifdef  _WIN64
		typedef long long ssize_t;
	#else
		typedef int ssize_t;
	#endif
#else
	typedef long int ssize_t; // ssize_t only available on posix
#endif
typedef unsigned long executiontime_t;
/** Constant for config file name. */
static const std::string configFilename = "NUI.xml";
/*************  Supported modalities ******************************************/
static const std::string MOTION_TRACKING_MODALITY       = "Motion Tracking";
static const std::string LEAP_MOTION_MODALITY           = "LEAP Motion";
static const std::string MYO_MODALITY                   = "Myo";
static const std::string WEBCAM_DEVICE                  = "WebCam";
static const std::string PMDNANO_DEVICE                 = "PMDNano";
static const std::string INTELREALSENSE_DEVICE          = "Intel RealSense";
static const std::string DEPTHLSENSE_DEVICE             = "DepthSense";
static const std::string KINECTV2_DEVICE                = "KinectV2";
/*************  Supported modules *********************************************/
static const std::string DUMPER_MODULE                  = "Dumper";
static const std::string VIZ_MODULE                     = "Visualizer";
static const std::string GESTURE_RECOGNITON_MODULE      = "Gesture Recognition";
static const std::string EYE_TRACKING_MODULE            = "Eye Tracking";
/*************  Processing modules *********************************************/
static const std::string HANDGESTURE_RECOGNITON_MODULE  = "HandGestureRecognition";
static const std::string MODELCLASSIFIER_MODULE         = "ModelClassifier";
/*************  Conversion CONSTANTS ******************************************/
static const int32_t TRANSMISSION_TYPE_LEAP_TRACKING       = 1;
static const int32_t TRANSMISSION_TYPE_LEAP_SWIPE          = TRANSMISSION_TYPE_LEAP_TRACKING + 1;
static const int32_t TRANSMISSION_TYPE_LEAP_CIRCLE         = TRANSMISSION_TYPE_LEAP_SWIPE + 1;
static const int32_t TRANSMISSION_TYPE_LEAP_TAP            = TRANSMISSION_TYPE_LEAP_CIRCLE + 1;
static const int32_t TRANSMISSION_TYPE_LEAP_PINCH          = TRANSMISSION_TYPE_LEAP_TAP + 1;
static const int32_t TRANSMISSION_TYPE_LEAP_GRAB           = TRANSMISSION_TYPE_LEAP_PINCH + 1;
static const int32_t TRANSMISSION_TYPE_LEAP_ACTION_GESTURE = TRANSMISSION_TYPE_LEAP_GRAB + 1;
static const int32_t TRANSMISSION_TYPE_SYSTEM_EVENT        = TRANSMISSION_TYPE_LEAP_ACTION_GESTURE + 1;
static const int32_t TRANSMISSION_TYPE_ATTENTION_EVENT     = TRANSMISSION_TYPE_SYSTEM_EVENT + 1;
static const int32_t TRANSMISSION_TYPE_MYO_EVENT           = TRANSMISSION_TYPE_ATTENTION_EVENT + 1;
// ---------- Namespaces ---------------------
namespace bfs = boost::filesystem;

#define SLEEP(DUR) boost::this_thread::sleep(boost::posix_time::milliseconds(DUR))
#define QUOTE(str) #str
#define STR(N) #N
#define I2Str(V) std::to_string(V)
// -------- Custom logging macros ----------------
#include <boost/log/trivial.hpp>
#define LOG_IF(X, B) BOOST_LOG_TRIVIAL(info)
#define LOG_GUI(X, B) BOOST_LOG_TRIVIAL(info)
// ------------- Templates ------------------------
// Safe release for pointer
template<class T>
inline void PtrRelease(T * ptr) {
	if (ptr != NULL) {
		delete ptr;
		ptr = NULL;
	}
}
template<class T>
inline void PtrReleaseArray(T * ptr) {
	if (ptr != NULL) {
		delete[] ptr;
		ptr = NULL;
	}
}

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease) {
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
/**
 * \brief Retrieve current timestamp.
 * \return timestamp
 */
inline time_t timestamp_now() {
	timestamp_t t(boost::posix_time::microsec_clock::universal_time());
	duration_t td = t - T0;
	return static_cast<time_t>(td.total_milliseconds());
}

/**
 * \brief Retrieve current timestamp as string.
 * \return timestamp
 */
inline std::string date_string()
{
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	localtime_s(&tstruct, &now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d_%H.%M.%S", &tstruct);
	return buf;
}
