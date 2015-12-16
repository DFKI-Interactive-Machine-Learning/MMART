#include "devices/IntelRealSenseCamera.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <pcl/pcl_macros.h>


template <typename T>
struct buffer_traits
{
  static T invalid () { return 0; }
  static bool is_invalid (T value) { return value == invalid (); };
};

template <>
struct buffer_traits <float>
{
  static float invalid () { return std::numeric_limits<float>::quiet_NaN (); };
  static bool is_invalid (float value) { return static_cast<bool>(pcl_isnan (value)); };
};

template <typename T>
pcl::io::Buffer<T>::Buffer (size_t size)
: size_ (size)
{
}

template <typename T>
pcl::io::Buffer<T>::~Buffer ()
{
}

template <typename T>
pcl::io::SingleBuffer<T>::SingleBuffer (size_t size)
: Buffer<T> (size)
, data_ (size, buffer_traits<T>::invalid ())
{
}

template <typename T>
pcl::io::SingleBuffer<T>::~SingleBuffer ()
{
}

template <typename T> T
pcl::io::SingleBuffer<T>::operator[] (size_t idx) const
{
  assert (idx < size_);
  return (data_[idx]);
}

template <typename T> void
pcl::io::SingleBuffer<T>::push (std::vector<T>& data)
{
  assert (data.size () == size_);
  data_.swap (data);
  data.clear ();
}

template <typename T>
pcl::io::MedianBuffer<T>::MedianBuffer (size_t size,
                                        size_t window_size)
: Buffer<T> (size)
, window_size_ (window_size)
, midpoint_ (window_size_ / 2)
, data_current_idx_ (window_size_ - 1)
{
  assert (size_ > 0);
  assert (window_size_ > 0 &&
          window_size_ <= std::numeric_limits<unsigned char>::max ());

  data_.resize (window_size_);
  for (size_t i = 0; i < window_size_; ++i)
    data_[i].resize (size_, buffer_traits<T>::invalid ());

  data_argsort_indices_.resize (size_);
  for (size_t i = 0; i < size_; ++i)
  {
    data_argsort_indices_[i].resize (window_size_);
    for (size_t j = 0; j < window_size_; ++j)
      data_argsort_indices_[i][j] = j;
  }

  data_invalid_count_.resize (size_, window_size_);
}

template <typename T>
pcl::io::MedianBuffer<T>::~MedianBuffer ()
{
}

template <typename T> T
pcl::io::MedianBuffer<T>::operator[] (size_t idx) const
{
  assert (idx < size_);
  int midpoint = (window_size_ - data_invalid_count_[idx]) / 2;
  return (data_[data_argsort_indices_[idx][midpoint]][idx]);
}

template <typename T> void
pcl::io::MedianBuffer<T>::push (std::vector<T>& data)
{
  assert (data.size () == size_);

  if (++data_current_idx_ >= window_size_)
    data_current_idx_ = 0;

  // New data will replace the column with index data_current_idx_. Before
  // overwriting it, we go through all the new-old value pairs and update
  // data_argsort_indices_ to maintain sorted order.
  for (size_t i = 0; i < size_; ++i)
  {
    const T& new_value = data[i];
    const T& old_value = data_[data_current_idx_][i];
    bool new_is_nan = buffer_traits<T>::is_invalid (new_value);
    bool old_is_nan = buffer_traits<T>::is_invalid (old_value);
    if (compare (new_value, old_value) == 0)
      continue;
    std::vector<unsigned char>& argsort_indices = data_argsort_indices_[i];
    // Rewrite the argsort indices before or after the position where we insert
    // depending on the relation between the old and new values
    if (compare (new_value, old_value) == 1)
    {
      for (int j = 0; j < window_size_; ++j)
        if (argsort_indices[j] == data_current_idx_)
        {
          int k = j + 1;
          while (k < window_size_ && compare (new_value, data_[argsort_indices[k]][i]) == 1)
          {
            std::swap (argsort_indices[k - 1], argsort_indices[k]);
            ++k;
          }
          break;
        }
    }
    else
    {
      for (int j = window_size_ - 1; j >= 0; --j)
        if (argsort_indices[j] == data_current_idx_)
        {
          int k = j - 1;
          while (k >= 0 && compare (new_value, data_[argsort_indices[k]][i]) == -1)
          {
            std::swap (argsort_indices[k], argsort_indices[k + 1]);
            --k;
          }
          break;
        }
    }

    if (new_is_nan && !old_is_nan)
      ++data_invalid_count_[i];
    else if (!new_is_nan && old_is_nan)
      --data_invalid_count_[i];
  }

  // Finally overwrite the data
  data_[data_current_idx_].swap (data);
  data.clear ();
}

template <typename T> int
pcl::io::MedianBuffer<T>::compare (T a, T b)
{
  bool a_is_nan = buffer_traits<T>::is_invalid (a);
  bool b_is_nan = buffer_traits<T>::is_invalid (b);
  if (a_is_nan && b_is_nan)
    return 0;
  if (a_is_nan)
    return 1;
  if (b_is_nan)
    return -1;
  if (a == b)
    return 0;
  return a > b ? 1 : -1;
}

template <typename T>
pcl::io::AverageBuffer<T>::AverageBuffer (size_t size,
                                          size_t window_size)
: Buffer<T> (size)
, window_size_ (window_size)
, data_current_idx_ (window_size_ - 1)
{
  assert (size_ > 0);
  assert (window_size_ > 0 &&
          window_size_ <= std::numeric_limits<unsigned char>::max ());

  data_.resize (window_size_);
  for (size_t i = 0; i < window_size_; ++i)
    data_[i].resize (size_, buffer_traits<T>::invalid ());

  data_sum_.resize (size_, 0);
  data_invalid_count_.resize (size_, window_size_);
}

template <typename T>
pcl::io::AverageBuffer<T>::~AverageBuffer ()
{
}

template <typename T> T
pcl::io::AverageBuffer<T>::operator[] (size_t idx) const
{
  assert (idx < size_);
  if (data_invalid_count_[idx] == window_size_)
    return (buffer_traits<T>::invalid ());
  else
    return (data_sum_[idx] / (window_size_ - data_invalid_count_[idx]));
}

template <typename T> void
pcl::io::AverageBuffer<T>::push (std::vector<T>& data)
{
  assert (data.size () == size_);

  if (++data_current_idx_ >= window_size_)
    data_current_idx_ = 0;

  // New data will replace the column with index data_current_idx_. Before
  // overwriting it, we go through the old values and subtract them from the
  // data_sum_
  for (size_t i = 0; i < size_; ++i)
  {
    const float& new_value = data[i];
    const float& old_value = data_[data_current_idx_][i];
    bool new_is_nan = buffer_traits<T>::is_invalid (new_value);
    bool old_is_nan = buffer_traits<T>::is_invalid (old_value);

    if (!old_is_nan)
      data_sum_[i] -= old_value;
    if (!new_is_nan)
      data_sum_[i] += new_value;

    if (new_is_nan && !old_is_nan)
      ++data_invalid_count_[i];
    else if (!new_is_nan && old_is_nan)
      --data_invalid_count_[i];
  }

  // Finally overwrite the data
  data_[data_current_idx_].swap (data);
  data.clear ();
}



using namespace nui::events;

/** Utility function to convert RealSense-style strings (which happen to
  * consist of 2-byte chars) into standard library strings. */
std::string
toString (const pxcCHAR* pxc_string, size_t max_length)
{
  size_t i = 0;
  while (i + 1 < max_length && pxc_string[i])
    ++i;
  std::string out (i + 1, '\0');
  size_t j = 0;
  while (j < i)
    out[j] = pxc_string[j++];
  return out;
}


boost::shared_ptr<PXCSession>
createPXCSession ()
{
  PXCSession* s = PXCSession_Create();
  return makePXCSharedPtr (s);
}

boost::shared_ptr<PXCCaptureManager>
createPXCCaptureManager (PXCSession& session)
{
  PXCCaptureManager* cm = session.CreateCaptureManager ();
  return makePXCSharedPtr (cm);
}

boost::shared_ptr<PXCCapture>
createPXCCapture (PXCSession& session, pxcUID iuid)
{
  PXCCapture* c;
  if (session.CreateImpl (iuid, &c) < PXC_STATUS_NO_ERROR)
  {
    BOOST_LOG_TRIVIAL(error) << "Unable to create RealSense capture";
  }
  return makePXCSharedPtr (c);
}

boost::shared_ptr<PXCCapture::Device>
createPXCCaptureDevice (PXCCapture& capture, pxcI32 didx)
{
  PXCCapture::Device* d;
  d = capture.CreateDevice (didx);
  if (!d)
    BOOST_LOG_TRIVIAL(error) <<  "Unable to create RealSense capture device";
  return makePXCSharedPtr (d);
}

void
IntelRealSenseCameraWrapper::createDeviceList ()
{
  m_device_list.clear ();
  // Module description to match
  PXCSession::ImplDesc module_desc = {};
  module_desc.group = PXCSession::IMPL_GROUP_SENSOR;
  module_desc.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;

  for (int m = 0;; m++)
  {
    PXCSession::ImplDesc desc;
    if (m_session->QueryImpl (&module_desc, m, &desc) < PXC_STATUS_NO_ERROR)
      break;
    PXCCapture* capture;
    if (m_session->CreateImpl<PXCCapture> (&desc, &capture) < PXC_STATUS_NO_ERROR)
      continue;
    for (int j = 0;; j++)
    {
      PXCCapture::DeviceInfo device_info;
      if (capture->QueryDeviceInfo (j, &device_info) < PXC_STATUS_NO_ERROR)
        break;
      if (device_info.streams & PXCCapture::STREAM_TYPE_DEPTH)
      {
        const size_t MAX_SERIAL_LENGTH = sizeof (device_info.serial) / sizeof (device_info.serial[0]);
        std::string serial = toString (device_info.serial, MAX_SERIAL_LENGTH);
        m_device_list.push_back (DeviceInfo ());
        m_device_list.back ().serial = serial;
        m_device_list.back ().iuid = desc.iuid;
        m_device_list.back ().didx = j;
      }
    }
    capture->Release ();
  }
}
boost::shared_ptr<RealSenseDevice>
IntelRealSenseCameraWrapper::capture (DeviceInfo& device_info)
{
  // This is called from public captureDevice() functions and should already be
  // under scoped lock
  if (!device_info.device_ptr.expired ())
  {
    return device_info.device_ptr.lock ();
  }
  else
  {
    RealSenseDevice::Ptr device (new RealSenseDevice (device_info.serial));
    device->m_capture = createPXCCapture (*m_session, device_info.iuid);
    device->m_device = createPXCCaptureDevice (*device->m_capture, device_info.didx);
    device_info.device_ptr = device;
    return device;
  }
}

RealSenseDevice::Ptr
IntelRealSenseCameraWrapper::captureDevice ()
{
  for (size_t i = 0; i < m_device_list.size (); ++i)
    if (!m_device_list[i].isCaptured ())
      return (capture (m_device_list[i]));
  return (RealSenseDevice::Ptr ());  // never reached, needed just to silence -Wreturn-type warning
}

void IntelRealSenseCameraWrapper::reconfigure()
{
	std::string filter = nui::config()->getString("RealSense.filter.Name", "none");
	m_windowsize = nui::config()->get("RealSense.filter.Windowsize", 1.0f);

	if(std::string("none").compare(filter) == 0)
	{
		disableTemporalFiltering();
	}
	else if(std::string("average").compare(filter) == 0)
	{
		enableTemporalFiltering(RealSense_Average, m_windowsize);
	}
	else if(std::string("median").compare(filter) == 0)
	{
		enableTemporalFiltering(RealSense_Median, m_windowsize);
	}
}

NUISTATUS IntelRealSenseCameraWrapper::initialize()
{
	NUISTATUS status = NUI_SUCCESS;
	// Create session
	m_session = createPXCSession();
	// Create device list
	createDeviceList();
	// Capture device 
	m_device = captureDevice();
	if(m_device == NULL)
	{
		BOOST_LOG_TRIVIAL(error)  << "No Realsense camera is available.";
		return NUI_FAILED;
	}
	PXCCapture::Device::StreamProfileSet profile;
      memset (&profile, 0, sizeof (profile));
      // TODO: this should depend on Mode
	profile.depth.frameRate.max = 30;
	profile.depth.frameRate.min = 30;
	profile.depth.imageInfo.width = WIDTH;
	profile.depth.imageInfo.height = HEIGHT;
	profile.depth.imageInfo.format = PXCImage::PIXEL_FORMAT_DEPTH;
	profile.depth.options = PXCCapture::Device::STREAM_OPTION_ANY;
	profile.color.frameRate.max = 30;
	profile.color.frameRate.min = 30;
    profile.color.imageInfo.width = COLOR_WIDTH;
    profile.color.imageInfo.height = COLOR_HEIGHT;
    profile.color.imageInfo.format = PXCImage::PIXEL_FORMAT_RGB32;
    profile.ir.options = PXCCapture::Device::STREAM_OPTION_ANY;
	profile.ir.frameRate.max = 30;
	profile.ir.frameRate.min = 30;
    profile.ir.imageInfo.width = COLOR_WIDTH;
    profile.ir.imageInfo.height = COLOR_HEIGHT;
	profile.ir.imageInfo.format = PXCImage::PIXEL_FORMAT_Y8;
	profile.ir.options = PXCCapture::Device::STREAM_OPTION_ANY;
	m_device->getPXCDevice ().SetStreamProfileSet (&profile);
	bool valid = static_cast<bool>(m_device->getPXCDevice().IsStreamProfileSetValid(&profile));
    if (!valid)
        BOOST_LOG_TRIVIAL(error)  << "Invalid stream profile for PXC device";
	m_ptrProjection = m_device->getPXCDevice ().CreateProjection ();
	return status;
}

NUISTATUS IntelRealSenseCameraWrapper::unInitialize()
{
	if(m_ptrProjection)
	{
		m_ptrProjection->Release();
	}
	return NUI_SUCCESS;
}

inline void createEvent(NUIEvent* evt)
{
	evt->device = INTELREALSENSE_DEVICE;
	evt->timestamp = timestamp_now();
}

void
 IntelRealSenseCameraWrapper::enableTemporalFiltering (TemporalFilteringType type, size_t window_size)
{
  if (m_temporal_filtering_type != type ||
     (type != RealSense_None && m_depth_buffer->size () != window_size))
  {
    switch (type)
    {
      case RealSense_None:
        {
          m_depth_buffer.reset (new pcl::io::SingleBuffer<unsigned short> (SIZE));
          break;
        }
      case RealSense_Median:
        {
          // TODO: MedianFilter freezes on destructor, investigate
          m_depth_buffer.reset (new pcl::io::MedianBuffer<unsigned short> (SIZE, window_size));
          break;
        }
      case RealSense_Average:
        {
          m_depth_buffer.reset (new pcl::io::AverageBuffer<unsigned short> (SIZE, window_size));
          break;
        }
    }
    m_temporal_filtering_type = type;
  }
}

void
IntelRealSenseCameraWrapper::disableTemporalFiltering ()
{
  enableTemporalFiltering (RealSense_None, 1);
}


void IntelRealSenseCameraWrapper::process(EventQueue& evtQueue)
{
	RealSenseEvent* evt = new RealSenseEvent();
	std::vector<PXCPoint3DF32> vertices (SIZE);
	long seq = updates();
	createEvent(evt);
	// Copy meta data
	evt->cloud = PointCloudRGBA::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA> (WIDTH, HEIGHT));
	evt->rgb = cv::Mat::zeros(cv::Size(WIDTH, HEIGHT), CV_32F);
	evt->id = seq;
	evt->cloud.get()->header.frame_id = seq;
	evt->cloud.get()->header.seq = seq;
	evt->cloud.get()->header.stamp = evt->timestamp;

    pxcStatus status;
	// read current sample
    status = m_device->getPXCDevice ().ReadStreams (PXCCapture::STREAM_TYPE_ANY, &sample);
	if(status == PXC_STATUS_NO_ERROR)
	{
		/* We preform the following steps to convert received data into point clouds:
		* 
		*   1. Push depth image to the depth buffer
		*   2. Pull filtered depth image from the depth buffer
		*   3. Project (filtered) depth image into 3D
		*   4. Fill XYZ point cloud with computed points
		*   5. Fill XYZRGBA point cloud with computed points
		*   7. Project color image into 3D
		*   6. Assign colors to points in XYZRGBA point cloud
		*
		* Steps 1-2 are skipped if temporal filtering is disabled.
		* Step 4 is skipped if there are no subscribers for XYZ clouds.
		* Steps 5-7 are skipped if there are no subscribers for XYZRGBA clouds. */
		if (m_temporal_filtering_type)
		  {
			PXCImage::ImageData data;
			sample.depth->AcquireAccess (PXCImage::ACCESS_READ, &data);
			std::vector<unsigned short> data_copy (SIZE);
			memcpy (data_copy.data (), data.planes[0], SIZE * sizeof (unsigned short));
			sample.depth->ReleaseAccess (&data);

			m_depth_buffer->push (data_copy);

			sample.depth->AcquireAccess (PXCImage::ACCESS_WRITE, &data);
			unsigned short* d = reinterpret_cast<unsigned short*> (data.planes[0]);
			for (size_t i = 0; i < SIZE; i++)
			  d[i] = (*m_depth_buffer)[i];
			sample.depth->ReleaseAccess (&data);
		  }
		// -------------------- Aquire the color image ---------------------------------------------------------
		PXCImage::ImageData colordata;
		sample.color->AcquireAccess (PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &colordata);
		evt->rgb = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, CV_8UC3, colordata.planes[0], colordata.pitches[0]);
		sample.color->ReleaseAccess(&colordata);
		// -------------------- Aquire the depth image ---------------------------------------------------------
		PXCImage::ImageData depthdata;
		sample.depth->AcquireAccess (PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &depthdata);
		evt->depth = cv::Mat(HEIGHT, WIDTH, CV_16SC1, depthdata.planes[0], depthdata.pitches[0]);
		sample.depth->ReleaseAccess(&depthdata);
		// -------------------- Aquire the IR image ---------------------------------------------------------
		/*PXCImage::ImageData irdata;
		sample.ir->AcquireAccess (PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_Y8, &irdata);
		evt->ir = cv::Mat(HEIGHT, WIDTH, CV_16SC1, irdata.planes[0], irdata.pitches[0]);
		sample.depth->ReleaseAccess(&irdata);*/
		// -------------------- Aquire the point cloud ---------------------------------------------------------
		PXCImage::ImageData data;
		m_ptrProjection->QueryVertices (sample.depth, vertices.data ());
		PXCImage* mapped = m_ptrProjection->CreateColorImageMappedToDepth (sample.depth, sample.color);
		mapped->AcquireAccess (PXCImage::ACCESS_READ, &data);
		uint32_t* d = reinterpret_cast<uint32_t*> (data.planes[0]);
		evt->cloud->header.stamp = evt->timestamp;
		evt->cloud->is_dense = false;
		for (int i = 0; i < SIZE; i++)
		{
			convertPoint (vertices[i], evt->cloud->points[i]);
			memcpy (&evt->cloud->points[i].rgba, &d[i], sizeof (uint32_t));
		}
		mapped->ReleaseAccess (&data);
		mapped->Release ();
	
	}
	else if(status == PXC_STATUS_DEVICE_LOST)
	{
		BOOST_LOG_TRIVIAL(error) << "Failed to read data stream from PXC device: device lost";
	}
	else if(status == PXC_STATUS_ALLOC_FAILED)
	{
		BOOST_LOG_TRIVIAL(error) << "Failed to read data stream from PXC device: alloc failed";
	}
    sample.ReleaseImages ();
	// publish event
	evtQueue.push(evt);
}
