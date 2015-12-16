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
#define FORMAT_QVGA_ID 0
#define FORMAT_VGA_ID 1

#define FORMAT_QVGA_WIDTH 320
#define FORMAT_QVGA_HEIGHT 240
#define FORMAT_QVGA_PIXELS 76800
#define FORMAT_VGA_WIDTH 640
#define FORMAT_VGA_HEIGHT 480
#define FORMAT_VGA_PIXELS 307200

#define FILETYPE_NONE 0
#define FILETYPE_JPG 1


#include "core/common.hpp"
#include "core/NUIDLLexport.hpp"
#include "eventmanager/Modality.hpp"
#include "eventmanager/Events.hpp"


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
#include <vector>
#include <exception>

#include <DepthSense.hxx>

using namespace DepthSense;


/** ================================= Wrapper class ================================== */
/**
 * \brief Interface for DepthSense depth camera device.
 */
class DEPTHSENSECAMERA_EXPORT DepthSenseCameraWrapper : public IModality {
public:
	/**
	 * \brief Constructs DepthSense camera. 
	 */
	DepthSenseCameraWrapper();

	virtual ~DepthSenseCameraWrapper() {
	};
	/** ----------- IModality virtual function ------------- **/
	virtual void
	process();
	virtual void reconfigure();
	void sendData();
	virtual NUISTATUS initialize();
	virtual NUISTATUS unInitialize();
	virtual NUISTATUS onStop();

private:
    static const int FRAMERATE = 30;
    static const int WIDTH = 640;
    static const int HEIGHT = 480;
    static const int SIZE = WIDTH * HEIGHT;
    static const int COLOR_WIDTH = 640;
    static const int COLOR_HEIGHT = 480;
    static const int COLOR_SIZE = COLOR_WIDTH * COLOR_HEIGHT;
	static int m_depthBuffer; 
	static int m_colorBuffer; 
};
