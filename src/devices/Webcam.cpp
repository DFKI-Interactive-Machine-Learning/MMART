#include "devices/Webcam.hpp"
#include <iostream>

using namespace nui::events;

void WebCamWrapper::reconfigure() {
	nui::config();

}

NUISTATUS WebCamWrapper::initialize() {
	p_capture = new cv::VideoCapture(0);
    p_capture_2 = new cv::VideoCapture(1);
    NUISTATUS status = NUI_FAILED;
    // we need at least one camera
	if (!p_capture->isOpened())  // if not success, exit program
	{
        BOOST_LOG_TRIVIAL(error)<< "Cannot open the first video stream.";
	}
	else
	{
		status = NUI_SUCCESS;
	}
    if (!p_capture_2->isOpened())  // if not success, exit program
    {
        BOOST_LOG_TRIVIAL(error)<< "Cannot open the second video stream.";
    }
    else
    {
    	status = NUI_SUCCESS;
    }
	return status;
}

NUISTATUS WebCamWrapper::unInitialize() {

	p_capture->release();
    p_capture_2->release();
	return NUI_SUCCESS;
}
inline void createEvent(NUIEvent* evt) {
	evt->device = WEBCAM_DEVICE;
	evt->timestamp = timestamp_now();
}

void WebCamWrapper::process(EventQueue& evtQueue)
{
//    std::cout << "Process cam event queue..." << std::endl;
    if(p_capture->isOpened())
	{
//        std::cout << "cam 1 event queue..." << std::endl;
		CamEvent* evt = new CamEvent();
		createEvent(evt);
		evt->id = 0;
		evt->dWidth = p_capture->get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
		evt->dHeight = p_capture->get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the  video
		bool bSuccess = p_capture->read(evt->frame); // read a new frame from video
		if (bSuccess) //if not success, break loop
		{
			evtQueue.push(evt);
		}
		else
		{
			delete evt;
		}
	}

    if(p_capture_2->isOpened())
    {
//        std::cout << "cam 2 event queue..." << std::endl;
        CamEvent* evt_2 = new CamEvent();
        createEvent(evt_2);
        evt_2->id = 1;
        evt_2->dWidth = p_capture_2->get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
        evt_2->dHeight = p_capture_2->get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the  video
        bool bSuccess_2 = p_capture_2->read(evt_2->frame); // read a new frame from video
        if (bSuccess_2) //if not success, break loop
        {
            evtQueue.push(evt_2);
        }
        else
        {
            delete evt_2;
        }
    }
//    std::cout << "cams done..." << std::endl;
}
