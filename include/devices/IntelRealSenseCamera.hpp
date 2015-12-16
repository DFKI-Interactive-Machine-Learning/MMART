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
#include <pcl/io/grabber.h>

#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <pcl/pcl_exports.h>

#include <pxcsession.h>
#include <pxccapture.h>
#include <pxccapturemanager.h>
#include <pxcprojection.h>

/** ================================= Templates ================================== */
/* Helper function to convert a PXCPoint3DF32 point into a PCL point.
 * Takes care of unit conversion (PXC point coordinates are in millimeters)
 * and invalid points. */
template <typename T> inline void
convertPoint (const PXCPoint3DF32& src, T& tgt)
{
  static const float nan = std::numeric_limits<float>::quiet_NaN ();
  if (src.z == 0)
  {
    tgt.x = tgt.y = tgt.z = nan;
  }
  else
  {
    tgt.x = src.x / 1000.0;
    tgt.y = src.y / 1000.0;
    tgt.z = src.z / 1000.0;
  }
}

/** Helper function to release a RealSense resource.
    Useful as a deleter for a shared pointer holding RealSense resource. */
template <typename T> void
releasePXCResource (T* resource)
{
  std::cout << "Releasing PXC resource " << typeid(T).name() << std::endl;
  if (resource)
  {
    resource->Release ();
    resource = 0;
  }
}
template <typename T> boost::shared_ptr<T>
makePXCSharedPtr (T* resource)
{
  return boost::shared_ptr<T> (resource, releasePXCResource<T>);
}


/** ================================= Helper classes ================================== */
 class INTELREALSENSE_EXPORT RealSenseDevice 
{

public:

    typedef boost::shared_ptr<RealSenseDevice> Ptr;

    inline const std::string&
    getSerialNumber () { return m_device_id; }

    inline PXCCapture::Device&
    getPXCDevice () { return *m_device; }

private:

    friend class IntelRealSenseCameraWrapper;

    std::string                            m_device_id;
    boost::shared_ptr<PXCCapture>          m_capture;
    boost::shared_ptr<PXCCapture::Device>  m_device;

    RealSenseDevice (const std::string& id) : m_device_id (id) { };

};

 struct DeviceInfo
{
	pxcUID iuid;
	pxcI32 didx;
	std::string serial;
	boost::weak_ptr<RealSenseDevice> device_ptr;
	inline bool isCaptured () { return (!device_ptr.expired ()); }
};

 namespace pcl
{

  namespace io
  {

    template <typename T>
    class Buffer
    {

      public:

        typedef T value_type;

        virtual
        ~Buffer ();

        virtual T
        operator[] (size_t idx) const = 0;

        virtual void
        push (std::vector<T>& data) = 0;

        inline size_t
        size () const
        {
          return (size_);
        }

      protected:

        Buffer (size_t size);

        const size_t size_;

    };

    template <typename T>
    class SingleBuffer : public Buffer<T>
    {

      public:

        SingleBuffer (size_t size);

        virtual
        ~SingleBuffer ();

        virtual T
        operator[] (size_t idx) const;

        virtual void
        push (std::vector<T>& data);

      private:

        std::vector<T> data_;

        using Buffer<T>::size_;

    };

    template <typename T>
    class MedianBuffer : public Buffer<T>
    {

      public:

        MedianBuffer (size_t size, size_t window_size);

        virtual
        ~MedianBuffer ();

        virtual T
        operator[] (size_t idx) const;

        virtual void
        push (std::vector<T>& data);

      private:

        /** Compare two data elements.
          *
          * Invalid value is assumed to be larger than everything else. If both values
          * are invalid, they are assumed to be equal.
          *
          * \return -1 if \c a < \c b, 0 if \c a == \c b, 1 if \c a > \c b */
        static int compare (T a, T b);

        const size_t window_size_;
        const size_t midpoint_;

        /// Data pushed into the buffer (last window_size_ chunks), logically
        /// organized as a circular buffer
        std::vector<std::vector<T> > data_;

        /// Index of the last pushed data chunk in the data_ circular buffer
        size_t data_current_idx_;

        /// Indices that the argsort function would produce for data_ (with
        /// dimensions swapped)
        std::vector<std::vector<unsigned char> > data_argsort_indices_;

        /// Number of invalid values in the buffer
        std::vector<unsigned char> data_invalid_count_;

        using Buffer<T>::size_;

    };

    template <typename T>
    class AverageBuffer : public Buffer<T>
    {

      public:

        AverageBuffer (size_t size, size_t window_size);

        virtual
        ~AverageBuffer ();

        virtual T
        operator[] (size_t idx) const;

        virtual void
        push (std::vector<T>& data);

      private:

        const size_t window_size_;

        /// Data pushed into the buffer (last window_size_ chunks), logically
        /// organized as a circular buffer
        std::vector<std::vector<T> > data_;

        /// Index of the last pushed data chunk in the data_ circular buffer
        size_t data_current_idx_;

        /// Current sum of the buffer
        std::vector<T> data_sum_;

        /// Number of invalid values in the buffer
        std::vector<unsigned char> data_invalid_count_;

        using Buffer<T>::size_;

    };

  }

}

/** ================================= Wrapper class ================================== */
/**
 * \brief Interface for an Intel Realsense device.
 */
class INTELREALSENSE_EXPORT IntelRealSenseCameraWrapper: public IModality {
public:
	enum TemporalFilteringType
	{
		RealSense_None = 0,
		RealSense_Median = 1,
		RealSense_Average = 2,
	};
	IntelRealSenseCameraWrapper() :
			IModality(INTELREALSENSE_DEVICE, 50), m_ptrProjection(NULL) {
		reconfigure();
	};

	virtual ~IntelRealSenseCameraWrapper() {
	};
	
	void
    enableTemporalFiltering (TemporalFilteringType type, size_t window_size);

    void
    disableTemporalFiltering ();
	/** ----------- IModality virtual function ------------- **/
	virtual void
	process(EventQueue&);
	virtual void reconfigure();
	virtual NUISTATUS initialize();
	virtual NUISTATUS unInitialize();
private:
    static const int FRAMERATE = 30;
    static const int WIDTH = 640;
    static const int HEIGHT = 480;
    static const int SIZE = WIDTH * HEIGHT;
    static const int COLOR_WIDTH = 640;
    static const int COLOR_HEIGHT = 480;
    static const int COLOR_SIZE = COLOR_WIDTH * COLOR_HEIGHT;
	/**
	 * \brief Projection of camera. 
	 */
	TemporalFilteringType m_temporal_filtering_type;
	/**
	 * \brief Projection of camera. 
	 */
	PXCProjection* m_ptrProjection;
	/**
	 * \brief Capturing sample.
	 */
	PXCCapture::Sample sample;
	/**
	 * \brief Camera device.
	 */
	boost::shared_ptr<RealSenseDevice> m_device;

	/**
	 * \brief Depth buffer to perform temporal filtering of the depth images.
	 */
    boost::shared_ptr<pcl::io::Buffer<unsigned short> > m_depth_buffer;
	/**
	 * Session.
	 */
	boost::shared_ptr<PXCSession> m_session;
	
	/**
	 * Capture manager.
	 */
    boost::shared_ptr<PXCCaptureManager> m_capture_manager;
	/**
	 * Device list.
	 */ 
	std::vector<DeviceInfo> m_device_list;
	/**
	 * \brief Windowsize filtering.
	 */
	float m_windowsize;

	/**
	 * Capture device.
	 */
	RealSenseDevice::Ptr
	captureDevice ();
	
	boost::shared_ptr<RealSenseDevice>
    capture (DeviceInfo& device_info);
	/**
	 * \brief Create device list.
	 */
	void
	createDeviceList ();
	/**
	 * \brief Handle error msg.
	 * \param[in] - msg
	 */
    void handleError(const std::string& );
};
