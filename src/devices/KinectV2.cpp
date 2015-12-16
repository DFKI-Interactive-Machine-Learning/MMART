#include "devices/KinectV2.hpp"
#include <iostream>

using namespace nui::events;

void KinectWrapper::reconfigure()
{
	m_minDistance = nui::config()->get("Kinect.filter.MinDistance", 0.05f);
	m_maxDistance = nui::config()->get("Kinect.filter.MaxDistance", 0.6f);
}

NUISTATUS KinectWrapper::initialize()
{
	NUISTATUS status = NUI_SUCCESS;
	if (m_freenect2.enumerateDevices() == 0)
	{
		LOG(ERROR)<< "No Kinect sensor detected.";
		return NUI_FAILED;
	}
	std::string serial = m_freenect2.getDefaultDeviceSerialNumber();
	m_ptrPipeline = new libfreenect2::CpuPacketPipeline();
	m_ptrDev = m_freenect2.openDevice(serial, m_ptrPipeline);
	if (m_ptrDev == 0)
	{
		LOG(ERROR)<< "Open Kinect sensor failed.";
		return NUI_FAILED;
	}
	m_ptrDev->setColorFrameListener(&m_listener);
	m_ptrDev->setIrAndDepthFrameListener(&m_listener);
	return status;
}

NUISTATUS KinectWrapper::onStart()
{
	m_ptrDev->start();
	m_ptrRegistration = new libfreenect2::Registration(m_ptrDev->getIrCameraParams(),
													   m_ptrDev->getColorCameraParams());
	return NUI_SUCCESS;
}
NUISTATUS KinectWrapper::onStop()
{

	return NUI_SUCCESS;
}
NUISTATUS KinectWrapper::unInitialize()
{
	boost::mutex::scoped_lock lock(_mutex);
	m_ptrDev->stop();
	m_ptrDev->close();
	return NUI_SUCCESS;
}

inline void createEvent(NUIEvent* evt)
{
	evt->device = KINECTV2_DEVICE;
	evt->timestamp = timestamp_now();
}

cv::Size KinectWrapper::depthSize() const
{
	return cv::Size(_width, _height);
}

int KinectWrapper::captureDepth(cv::Mat& buffer)
{
	int res = 0;

	return res;
}

int KinectWrapper::captureAmplitude(cv::Mat& buffer)
{
	int res = 0;

	return res;
}

int KinectWrapper::capturePointCloud(PointCloud::Ptr buffer,
		PointCloud::Ptr filtered)
{
	int res = 0;

	return res;
}

void KinectWrapper::handleError(const std::string& msg)
{

}

void KinectWrapper::process(EventQueue& evtQueue)
{
	int res;
	Kinectv2Event* evt = new Kinectv2Event();
	createEvent(evt);
	m_listener.waitForNewFrame(m_frames);
	libfreenect2::Frame *rgb = m_frames[libfreenect2::Frame::Color];
	libfreenect2::Frame *ir = m_frames[libfreenect2::Frame::Ir];
	libfreenect2::Frame *depth = m_frames[libfreenect2::Frame::Depth];

	int key = cv::waitKey(1);

	m_listener.release(m_frames);
	// check if we have to sleep
	// publish event
	evtQueue.push(evt);
}
