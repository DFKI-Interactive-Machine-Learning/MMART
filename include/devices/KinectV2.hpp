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
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

/**
 * \brief Interface for hand tracking of LEAP Motion device.
 */
class KINECT_EXPORT KinectWrapper: public IModality
{
public:
	KinectWrapper() :
		m_ptrDev(NULL),
		m_ptrPipeline(NULL),
		m_ptrRegistration(NULL),
		m_listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth),
		m_Undistorted(512, 424, 4),
		m_Registered(512, 424, 4),
		IModality(KINECTV2_DEVICE, 30)
	{
		reconfigure();
	};

	virtual ~KinectWrapper()
	{
		PtrRelease(m_ptrDev);
		PtrRelease(m_ptrPipeline);
		PtrRelease(m_ptrRegistration);
	};

	/** ----------- IModality virtual function ------------- **/
	virtual void
	process(EventQueue&);
	virtual void reconfigure();
	virtual NUISTATUS initialize();
	virtual NUISTATUS unInitialize();
protected:
	/**
	 * \brief Called when modality starts.
	 */
	virtual NUISTATUS
	onStart();
	/**
	 * \brief Called when modality stops.
	 */
	virtual NUISTATUS
	onStop();
private:
	/**
	 * \brief Mutex for data access.
	 */
	boost::mutex _mutex;
	/**
	 * Kinect device reference.
	 */
	libfreenect2::Freenect2Device *m_ptrDev;
	/**
	 * Pipeline.
	 */
	libfreenect2::PacketPipeline *m_ptrPipeline;
	/**
	 * Registration.
	 */
	libfreenect2::Registration* m_ptrRegistration;
	/**
	 * Freenect.
	 */
	libfreenect2::Freenect2 m_freenect2;
	/**
	 * Listner
	 */
	libfreenect2::SyncMultiFrameListener m_listener;
	/**
	 * Frames.
	 */
	libfreenect2::FrameMap m_frames;
	/**
	 * Distorted and registered.
	 */
	libfreenect2::Frame m_Undistorted, m_Registered;
	/**
	 * \brief Width of the camear image.
	 */
	size_t _width;
	/**
	 * \brief Height of the camera image.
	 */
	size_t _height;
	/**
	 * \brief Size.
	 */
	size_t _size;
	/**
	 * \brief Minimum distance.
	 */
	float m_minDistance;
	/**
	 * \brief Minimum distance.
	 */
	float m_maxDistance;
	/**
	 * \brief Depth size.
	 * \return size
	 */
	cv::Size depthSize() const;
	/**
	 * \brief Capture the depth data.
	 * \param[out] - buffer of depth image
	 */
	int captureDepth(cv::Mat& buffer);
	/**
	 * \brief Capture the depth data.
	 * \param[out] - buffer of depth image
	 */
	int captureAmplitude(cv::Mat& buffer);
	/**
	 * \brief Capture the depth data.
	 * \param[out] - buffer of point cloud
	 * \param[out] - buffer of filtered point cloud
	 */
	int capturePointCloud(PointCloud::Ptr buffer, PointCloud::Ptr filtered);
	/**
	 * \brief Handle error msg.
	 * \param[in] - msg
	 */
	void handleError(const std::string& );
};
