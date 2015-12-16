#include "devices/DepthSenseCamera.hpp"

using namespace nui::events;
using namespace DepthSense;
using namespace std;

inline void createEvent(NUIEvent* evt)
{
	evt->device = DEPTHSENSE_EVENT;
	evt->timestamp = timestamp_now();
}

/**
 * Color data.
 */
cv::Mat g_colorData;
/**
 * Color collected.
 */
bool g_colorFlag = false;
/**
 * Depth data.
 */
cv::Mat g_depthData;
/**
 * Color collected.
 */
bool g_depthFlag = false;
/**
 * Current instance.
 */
DepthSenseCameraWrapper* g_instance = NULL;

long seconds, useconds;

bool usingUSB30Flag = true; // if the camera is plugged on a USB 3.0 port

int waitSecondsBeforeGrab = 1;
const int16_t confidenceThreshold = 150;

bool interpolateDepthFlag;
bool buildColorSyncFlag, buildDepthSyncFlag, buildConfidenceFlag;

// Acquired data
uint16_t pixelsConfidenceQVGA[FORMAT_QVGA_PIXELS];
uint16_t pixelsDepthAcqQVGA[FORMAT_QVGA_PIXELS];
uint8_t pixelsColorAcqVGA[3*FORMAT_VGA_PIXELS];

// UVmap-processed frames
uint8_t pixelsColorSyncQVGA[3*FORMAT_QVGA_PIXELS];
uint16_t pixelsDepthSyncQVGA[FORMAT_QVGA_PIXELS];

// Interpolated frames
uint8_t pixelsColorSyncVGA[3*FORMAT_VGA_PIXELS];
uint16_t pixelsDepthAcqVGA[FORMAT_VGA_PIXELS];

FrameFormat frameFormatDepth = FRAME_FORMAT_QVGA; // Depth QVGA
const int nPixelsDepthAcq = FORMAT_QVGA_PIXELS;
uint16_t* pixelsDepthAcq = pixelsDepthAcqQVGA;

bool* hasData;

// Color map configuration, comment out undesired parameters

FrameFormat frameFormatColor;
int widthColor, heightColor, nPixelsColorAcq;
uint8_t* pixelsColorAcq;
uint16_t* pixelsDepthSync;

// Snapshot data
uint16_t* pixelsDepthAcqVGASnapshot;
uint8_t* pixelsColorSyncVGASnapshot;

uint16_t* pixelsDepthSyncSnapshot;
uint16_t* pixelsConfidenceQVGASnapshot;

const uint16_t noDepthDefault = 65535;
const uint16_t noDepthThreshold = 2000;
const uint16_t deltaDepthSync = 132;     // DS325

uint8_t noDepthBGR[3] = {0,0,0};

int colorPixelInd, colorPixelRow, colorPixelCol;
int debugInt;

double timeStampSeconds;
int timeStamp;
clock_t clockStartGrab;

int frameCount = -1;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;

uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;


void checkTrigger()
{
	if(g_colorFlag && g_depthFlag)
	{
		g_colorFlag = false;
		g_depthFlag = false;
		g_instance->sendData();
	}
}

/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
	for (int currentPixelInd = 0; currentPixelInd < nPixelsColorAcq; currentPixelInd++)
    {
        // Reinitialize synchronized depth
        hasData[currentPixelInd] = 0;
		pixelsColorAcq[3*currentPixelInd]   = data.colorMap[3*currentPixelInd];
        pixelsColorAcq[3*currentPixelInd+1] = data.colorMap[3*currentPixelInd+1];
        pixelsColorAcq[3*currentPixelInd+2] = data.colorMap[3*currentPixelInd+2];
    }
    g_cFrames++;
	g_colorData = cv::Mat(FORMAT_VGA_HEIGHT, FORMAT_VGA_WIDTH, CV_8UC3, pixelsColorAcq);
	g_colorFlag = true;
	// if color and depth is set trigger event
	checkTrigger();
}

/*----------------------------------------------------------------------------*/
// New depth sample event handler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
	if(!data.confidenceMap) return;
    // Initialize raw depth and UV maps
    for (int currentPixelInd = 0; currentPixelInd < nPixelsDepthAcq; currentPixelInd++)
    {
        if (buildConfidenceFlag) pixelsConfidenceQVGA[currentPixelInd] = data.confidenceMap[currentPixelInd];
        pixelsDepthSyncQVGA[currentPixelInd] = noDepthDefault;
        if (data.confidenceMap[currentPixelInd] > confidenceThreshold)
		{
            pixelsDepthAcq[currentPixelInd] = data.depthMap[currentPixelInd];
		}
        else
		{
            pixelsDepthAcq[currentPixelInd] = noDepthDefault;
		}
    }
	g_dFrames++;
	g_depthData = cv::Mat(FORMAT_QVGA_HEIGHT, FORMAT_QVGA_WIDTH, CV_16SC1, pixelsDepthAcq);
    g_depthFlag = true;
	// if color and depth is set trigger event
	checkTrigger();
}

/*----------------------------------------------------------------------------*/
void configureDepthNode()
{
    g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);
    DepthNode::Configuration config = g_dnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_QVGA;
    config.framerate = 30;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;
    g_dnode.setEnableDepthMap(true);
    g_dnode.setEnableConfidenceMap(true);
	try 
    {
        g_context.requestControl(g_dnode,0);
        g_dnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Argument Exception: " << e.what();
    }
    catch (UnauthorizedAccessException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Unauthorized Access Exception:" << e.what();
    }
    catch (IOException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "IO Exception: " << e.what();
    }
    catch (InvalidOperationException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Invalid Operation Exception: " << e.what();
    }
    catch (ConfigurationException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Configuration Exception: " << e.what();
    }
    catch (StreamingException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Streaming Exception:" << e.what();
    }
    catch (TimeoutException&)
    {
        BOOST_LOG_TRIVIAL(error) << "TimeoutException";
    }

}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
    // connect new color sample handler
    g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

    ColorNode::Configuration config = g_cnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_VGA;
    config.compression = COMPRESSION_TYPE_MJPEG;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
    config.framerate = 30;
    g_cnode.setEnableColorMap(true);
    try 
    {
        g_context.requestControl(g_cnode,0);
		g_cnode.setConfiguration(config);
		/** Configuration for camera. */
        g_cnode.setBrightness(0);
        g_cnode.setContrast(5);
        g_cnode.setSaturation(5);
        g_cnode.setHue(0);
        g_cnode.setGamma(3);
        g_cnode.setWhiteBalance(4650);
        g_cnode.setSharpness(5);
        g_cnode.setWhiteBalanceAuto(true);
        g_cnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Argument Exception: " << e.what();
    }
    catch (UnauthorizedAccessException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Unauthorized Access Exception: " << e.what();
    }
    catch (IOException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "IO Exception: " << e.what();
    }
    catch (InvalidOperationException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Invalid Operation Exception: " << e.what();
    }
    catch (ConfigurationException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Configuration Exception: " << e.what();
    }
    catch (StreamingException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "Streaming Exception: " << e.what();
    }
    catch (TimeoutException& e)
    {
        BOOST_LOG_TRIVIAL(error) << "TimeoutException " << e.what();
    }
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
    if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
    {
        g_dnode = node.as<DepthNode>();
        configureDepthNode();
        g_context.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
    {
        g_cnode = node.as<ColorNode>();
        configureColorNode();
        g_context.registerNode(node);
    }
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
    configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
        g_cnode.unset();
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
        g_dnode.unset();
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
    if (!g_bDeviceFound)
    {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
    g_bDeviceFound = false;
}


void DepthSenseCameraWrapper::reconfigure()
{

}


void DepthSenseCameraWrapper::sendData()
{
	DepthSenseEvent* evt = new DepthSenseEvent();
	evt->rgb   = g_colorData;
	evt->depth = g_depthData;
	m_updates += 1;
	createEvent(evt);
	fireEvent(NUIEVENT_PTR(evt));
}

DepthSenseCameraWrapper::DepthSenseCameraWrapper() :
			IModality(DEPTHLSENSE_DEVICE, SAMPLING_30_HZ, BLOCKING) 
{
	g_instance = this;
	reconfigure();
};

NUISTATUS DepthSenseCameraWrapper::initialize()
{
	NUISTATUS status = NUI_SUCCESS;
	//g_context = Context::create("localhost");
	g_context = Context::createStandalone();
    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

    // Get the list of currently connected devices
    std::vector<Device> da = g_context.getDevices();
    // We are only interested in the first device
    if (da.size() >= 1)
    {
        g_bDeviceFound = true;

        da[0].nodeAddedEvent().connect(&onNodeConnected);
        da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

        std::vector<Node> na = da[0].getNodes();
        for (int n = 0; n < (int)na.size();n++)
		{
            configureNode(na[n]);
		}
    }
	frameFormatColor = FRAME_FORMAT_VGA;
    widthColor = FORMAT_VGA_WIDTH;
    heightColor = FORMAT_VGA_HEIGHT;
    nPixelsColorAcq = FORMAT_VGA_PIXELS;
    pixelsColorAcq = pixelsColorAcqVGA;
	hasData                      = (bool*) malloc(nPixelsColorAcq*sizeof(bool));
    // Snapshot data
    pixelsDepthAcqVGASnapshot    = (uint16_t*) malloc(FORMAT_VGA_PIXELS*sizeof(uint16_t));
    pixelsColorSyncVGASnapshot   = (uint8_t*)  malloc(3*FORMAT_VGA_PIXELS*sizeof(uint8_t));
    pixelsDepthSyncSnapshot      = (uint16_t*) malloc(nPixelsColorAcq*sizeof(uint16_t));
    pixelsConfidenceQVGASnapshot = (uint16_t*) malloc(FORMAT_QVGA_PIXELS*sizeof(uint16_t));
	g_context.startNodes();
	return status;
}

NUISTATUS DepthSenseCameraWrapper::unInitialize()
{
	// Clean up memory
	PtrRelease(g_pProjHelper);
	free(hasData);
    free(pixelsDepthAcqVGASnapshot);
    free(pixelsColorSyncVGASnapshot);
    free(pixelsDepthSyncSnapshot);
    free(pixelsConfidenceQVGASnapshot);
	if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
    if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
	return NUI_SUCCESS;
}

void DepthSenseCameraWrapper::process()
{
    g_context.run();
}

NUISTATUS DepthSenseCameraWrapper::onStop()
{
    g_context.stopNodes();
	g_context.quit();
	return NUI_SUCCESS;
}