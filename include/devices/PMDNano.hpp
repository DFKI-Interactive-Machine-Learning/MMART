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
#include <pmdsdk2.h>
#define SOURCE_PLUGIN "camboardnano"
#define SOURCE_PARAM ""
#define PROC_PLUGIN "camboardnanoproc"
#define PROC_PARAM ""


/**
 * \brief Interface for hand tracking of LEAP Motion device.
 */
class PMDNANO_EXPORT PMDNanoWrapper: public IModality {
public:
	PMDNanoWrapper() :
			IModality(PMDNANO_DEVICE, 50),  
			_source(NULL), _buffer(NULL), _vbuffer(NULL){
		reconfigure();
	};

	virtual ~PMDNanoWrapper() {
	};

	/** ----------- IModality virtual function ------------- **/
	virtual void
	process(EventQueue&);
	virtual void reconfigure();
	virtual NUISTATUS initialize();
	virtual NUISTATUS unInitialize();
private:
	/**
	 * \brief Mutex for data access.
	 */
	boost::mutex _mutex;
	/**
	 * \brief Capturing device.
	 */
	PMDHandle _handle;
	/**
	 * \brief Description.
	 */
	PMDDataDescription _description;
	/**
	 * \brief Filter.
	 */
	pcl::PassThrough<pcl::PointXYZ> pass;
	/**
	 * \brief Source.
	 */
	char* _source;
	/**
	 * \brief buffer.
	 */
	float* _buffer;
	/**
	 * \brief vbuffer.
	 */
	float* _vbuffer;
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
