#include "devices/PMDNano.hpp"
#include <iostream>

using namespace nui::events;

void PMDNanoWrapper::reconfigure()
{
	m_minDistance = nui::config()->get("PMDNano.filter.MinDistance", 0.05f);
	m_maxDistance = nui::config()->get("PMDNano.filter.MaxDistance", 0.6f);
}

NUISTATUS PMDNanoWrapper::initialize()
{
	NUISTATUS status = NUI_SUCCESS;
	int res;
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (m_minDistance, m_maxDistance);
	res = pmdOpen(&_handle, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN,
	PROC_PARAM);
	if (res != PMD_OK)
	{
		handleError("Failed to load open source plugin.");
		status = NUI_FAILED;
	}

	if (pmdGetSourceDataDescription(_handle, &_description) != PMD_OK)
	{
		handleError("Could not read data description.");
		status = NUI_FAILED;
	}
	else
	{
		if (_description.subHeaderType != PMD_IMAGE_DATA)
		{
			handleError("Source is not an image.");
			status = NUI_FAILED;
		}
	}

	if (status == NUI_SUCCESS)
	{
		_width = _description.img.numColumns;
		_height = _description.img.numRows;
		_size = _width * _height;
		_source = new char[_description.size];
		_buffer = new float[_size];
		_vbuffer = new float[3 * _size];
		if (pmdGetSourceData(_handle, _source, _description.size) != PMD_OK)
		{
			handleError("Failed to get source data.");
		}
	}
	return status;
}

NUISTATUS PMDNanoWrapper::unInitialize()
{
	boost::mutex::scoped_lock lock(_mutex);
	PtrReleaseArray(_source);
	PtrReleaseArray(_buffer);
	PtrReleaseArray(_vbuffer);
	pmdClose(_handle);
	return NUI_SUCCESS;
}

inline void createEvent(NUIEvent* evt)
{
	evt->device = PMDNANO_DEVICE;
	evt->timestamp = timestamp_now();
}

cv::Size PMDNanoWrapper::depthSize() const
{
	return cv::Size(_width, _height);
}

int PMDNanoWrapper::captureDepth(cv::Mat& buffer)
{
	int res;
	res = pmdGetDistances(_handle, _buffer, _size * sizeof(float));
	if (res == PMD_OK)
	{
		std::memcpy(buffer.data, _buffer, _size * sizeof(float));
	}
	return res;
}

int PMDNanoWrapper::captureAmplitude(cv::Mat& buffer)
{
	int res;
	res = pmdGetAmplitudes(_handle, _buffer, _size * sizeof(float));
	if (res == PMD_OK)
	{
		std::memcpy(buffer.data, _buffer, _size * sizeof(float));
	}
	return res;
}

int PMDNanoWrapper::capturePointCloud(PointCloud::Ptr buffer,
		PointCloud::Ptr filtered)
{
	size_t index = 0;
	int res;
	res = pmdGet3DCoordinates(_handle, _vbuffer, 3 * _size * sizeof(float));
	if (res == PMD_OK)
	{
		for (std::vector<PointCloud>::size_type i = 0;
				i != buffer->points.size(); i++)
		{
			buffer->points[i].x = _vbuffer[3 * index];
			buffer->points[i].y = _vbuffer[3 * index + 1];
			buffer->points[i].z = _vbuffer[3 * index + 2];
			index++;
		}
		// Create the filtering object
		pass.setInputCloud(buffer);
		pass.filter(*filtered);
	}
	return res;
}

void PMDNanoWrapper::handleError(const std::string& msg)
{
	char error[128];
	int res;
	res = pmdGetLastError(_handle, error, 128);
	if(res == PMD_OK)
	{
		BOOST_LOG_TRIVIAL(error)<< msg << ": " << error << std::endl;
	}
	else
	{
		BOOST_LOG_TRIVIAL(error) << msg << std::endl;
	}
	pmdClose(_handle);
}

void PMDNanoWrapper::process(EventQueue& evtQueue)
{
	int res;
	PMDNanoEvent* evt = new PMDNanoEvent();
	createEvent(evt);
	// Copy meta data
	evt->cloud = PointCloud::Ptr(new PointCloud(_width, _height));
	evt->filteredCloud = PointCloud::Ptr(new PointCloud(_width, _height));
	evt->depth = cv::Mat::zeros(depthSize(), CV_32F);
	evt->amplitude = cv::Mat::zeros(depthSize(), CV_32F);
	evt->dWidth = _width;
	evt->dHeight = _height;
	evt->id = updates();
	res = pmdUpdate(_handle);
	if (res != PMD_OK) // check if update is successful
	{
		handleError("Update failed.");
		stop();
		return;
	}
	// capture depth image
	if (captureDepth(evt->depth) != PMD_OK)
	{
		handleError("Capturing depth failed.");
		stop();
		return;
	};
	// capture amplitude
	if (captureAmplitude(evt->amplitude) != PMD_OK)
	{
		handleError("Capturing amplitude failed.");
		stop();
		return;
	};
	// capture cloud
	if (capturePointCloud(evt->cloud, evt->filteredCloud) != PMD_OK)
	{
		handleError("Capturing point cloud failed.");
		stop();
		return;
	};
	// check if we have to sleep
	// publish event
	evtQueue.push(evt);
}
