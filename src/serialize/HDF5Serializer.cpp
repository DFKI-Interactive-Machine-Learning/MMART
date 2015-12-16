#include "serialize/HDF5Serializer.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <vector>

#ifndef H5_NO_NAMESPACE
#ifndef H5_NO_STD
using std::cout;
using std::endl;
#endif  // H5_NO_STD
#endif
#include "H5Cpp.h"
#ifndef H5_NO_NAMESPACE
using namespace H5;
#endif
using namespace nui::events;
using namespace nui::stream;
namespace pt = boost::property_tree;

/** Storage for lables. */
std::vector<std::string> g_classLabels;

/** Storage for attention. */
std::vector<std::string> g_attentionList;

const size_t MYO_VECTOR_SIZE = 23;

const char* DATA_DESCRIPTION_MYO[MYO_VECTOR_SIZE] = {
	"state", 
	"onArm", "whichArm",
	"isUnlocked", "batteryLevel", "rssi", 
	"roll", "pitch", "yaw", 
	"acc_x", "acc_y", "acc_z", 
	"gyro_x", "gyro_y", "gyro_z",
	"emg1", "emg2", "emg3", 
	"emg4", "emg5", "emg6", 
	"emg7", "emg8"};

// TODO: Move configuration to a file
const size_t LEAP_VECTOR_SIZE = 220;
const size_t ATTENTION_SIZE = 5;
const char* DATA_DESCRIPTION_LEAP[LEAP_VECTOR_SIZE] = {
		"hand side right", "confidence", "pinchStrength", "grabStrength",
		"hand palmposition x", "hand palmposition y", "hand palmposition z",
		"hand palmvelocity x", "hand palmvelocity y", "hand palmvelocity z",
		"hand palmnormal x", "hand palmnormal y", "hand palmnormal z",
		"hand direction x", "hand direction y", "hand direction z",
		"hand spherecenter x", "hand spherecenter y", "hand spherecenter z",
		"hand sphereradius",
		// ------------------------------------------ FINGER DATA ------------------------------------------
		"thumb length", "thumb width",
		"thumb tipposition x", "thumb tipposition y", "thumb tipposition z",
		"thumb tipvelocity x", "thumb tipvelocity y", "thumb tipvelocity z",
		// --------------------------------------------------------------------------------------------------
		"thumb metacarpal length", "thumb metacarpal width",
		"thumb metacarpal center x", "thumb metacarpal center y", "thumb metacarpal center z",
		"thumb metacarpal direction x", "thumb metacarpal direction y", "thumb metacarpal direction z",
		"thumb proximal length", "thumb proximal width",
		"thumb proximal center x", "thumb proximal center y", "thumb proximal center z",
		"thumb proximal direction x", "thumb proximal direction y", "thumb proximal direction z",
		"thumb intermediate length", "thumb intermediate width",
		"thumb intermediate center x", "thumb intermediate center y", "thumb intermediate center z",
		"thumb intermediate direction x", "thumb intermediate direction y", "thumb intermediate direction z",
		"thumb distal length", "thumb distal width",
		"thumb distal center x", "thumb distal center y", "thumb distal center z",
		"thumb distal direction x", "thumb distal direction y", "thumb distal direction z",
		// --------------------------------------------------------------------------------------------------
		"index length", "index width",
		"index tipposition x", "index tipposition y", "index tipposition z",
		"index tipvelocity x", "index tipvelocity y", "index tipvelocity z",
		// --------------------------------------------------------------------------------------------------
	    "index metacarpal length", "index metacarpal width",
		"index metacarpal center x", "index metacarpal center y", "index metacarpal center z",
		"index metacarpal direction x", "index metacarpal direction y", "index metacarpal direction z",
		"index proximal length", "index proximal width",
		"index proximal center x", "index proximal center y", "index proximal center z",
		"index proximal direction x", "index proximal direction y", "index proximal direction z",
		"index intermediate length", "index intermediate width",
		"index intermediate center x", "index intermediate center y", "index intermediate center z",
		"index intermediate direction x", "index intermediate direction y", "index intermediate direction z",
		"index distal length", "index distal width",
		"index distal center x", "index distal center y", "index distal center z",
		"index distal direction x", "index distal direction y", "index distal direction z",
		// --------------------------------------------------------------------------------------------------
		"middle length", "middle width",
		"middle tipposition x", "middle tipposition y", "middle tipposition z",
		"middle tipvelocity x", "middle tipvelocity y", "middle tipvelocity z",
		// --------------------------------------------------------------------------------------------------
		"middle metacarpal length", "middle metacarpal width",
		"middle metacarpal center x", "middle metacarpal center y", "middle metacarpal center z",
		"middle metacarpal direction x", "middle metacarpal direction y", "middle metacarpal direction z",
		"middle proximal length", "middle proximal width",
		"middle proximal center x", "middle proximal center y", "middle proximal center z",
		"middle proximal direction x", "middle proximal direction y", "middle proximal direction z",
		"middle intermediate length", "middle intermediate width",
		"middle intermediate center x", "middle intermediate center y", "middle intermediate center z",
		"middle intermediate direction x", "middle intermediate direction y", "middle intermediate direction z",
		"middle distal length", "middle distal width",
		"middle distal center x", "middle distal center y", "middle distal center z",
		"middle distal direction x", "middle distal direction y", "middle distal direction z",
		// --------------------------------------------------------------------------------------------------
		"ring length", "ring width",
		"ring tipposition x", "ring tipposition y", "ring tipposition z",
		"ring tipvelocity x", "ring tipvelocity y", "ring tipvelocity z",
		// --------------------------------------------------------------------------------------------------
		"ring metacarpal length", "ring metacarpal width",
		"ring metacarpal center x", "ring metacarpal center y", "ring metacarpal center z",
		"ring metacarpal direction x", "ring metacarpal direction y", "ring metacarpal direction z",
		"ring proximal length", "ring proximal width",
		"ring proximal center x", "ring proximal center y", "ring proximal center z",
		"ring proximal direction x", "ring proximal direction y", "ring proximal direction z",
		"ring intermediate length", "ring intermediate width",
		"ring intermediate center x", "ring intermediate center y", "ring intermediate center z",
		"ring intermediate direction x", "ring intermediate direction y", "ring intermediate direction z",
		"ring distal length", "ring distal width",
		"ring distal center x", "ring distal center y", "ring distal center z",
		"ring distal direction x", "ring distal direction y", "ring distal direction z",
		// --------------------------------------------------------------------------------------------------
		"pinky length", "pinky width",
		"pinky tipposition x", "pinky tipposition y", "pinky tipposition z",
		"pinky tipvelocity x", "pinky tipvelocity y", "pinky tipvelocity z",
		// --------------------------------------------------------------------------------------------------
		"pinky metacarpal length", "pinky metacarpal width",
		"pinky metacarpal center x", "pinky metacarpal center y", "pinky metacarpal center z",
		"pinky metacarpal direction x", "pinky metacarpal direction y", "pinky metacarpal direction z",
		"pinky proximal length", "pinky proximal width",
		"pinky proximal center x", "pinky proximal center y", "pinky proximal center z",
		"pinky proximal direction x", "pinky proximal direction y", "pinky proximal direction z",
		"pinky intermediate length", "pinky intermediate width",
		"pinky intermediate center x", "pinky intermediate center y", "pinky intermediate center z",
		"pinky intermediate direction x", "pinky intermediate direction y", "pinky intermediate direction z",
		"pinky distal length", "pinky distal width",
		"pinky distal center x", "pinky distal center y", "pinky distal center z",
		"pinky distal direction x", "pinky distal direction y", "pinky distal direction z"
		// --------------------------------------------------------------------------------------------------
};




void reconfigure_serializer(NUIconfig* config)
{
	g_classLabels.clear();
	pt::ptree lTree = config->subTree("General.Datalabels");
	BOOST_FOREACH(pt::ptree::value_type v, lTree)
	{
		g_classLabels.push_back(v.second.data()); 
	}
	pt::ptree aTree = config->subTree("General.AttentionAreas");
	BOOST_FOREACH(pt::ptree::value_type v, aTree)
	{
		g_attentionList.push_back(v.second.data()); 
	}

}


// create or open a file
H5File* create_or_open(const std::string& fname)
{
    H5::Exception::dontPrint();
    H5::H5File* file = 0;

    try {
        file = new H5::H5File(fname.c_str(), H5F_ACC_RDWR);
    } catch(const H5::FileIException&) {
        file = new H5::H5File(fname.c_str(), H5F_ACC_TRUNC);
    }

    return file;
}

inline void assign(float* data, size_t* i, const Vector3f& v)
{
	data[*i] = v.x();
	*i += 1;
	data[*i] = v.y();
	*i += 1;
	data[*i] = v.z();
	*i += 1;
}

inline void assign(float* data, size_t* i, const float f)
{
	data[*i] = f;
	*i += 1;
}

float* handToVector(Hand& h) {
	float* v = new float[LEAP_VECTOR_SIZE];
	size_t i = 0;
	assign(v, &i,  (h.side() == Hand_Side_RIGHT) ? 1.f : 0.f);
	assign(v, &i, h.confidence());
	assign(v, &i, h.pinchstrength());
	assign(v, &i, h.grabstrength());
	assign(v, &i, h.palmposition());
	assign(v, &i, h.palmvelocity());
	assign(v, &i, h.palmnormal());
	assign(v, &i, h.direction());
	assign(v, &i, h.spherecenter());
	assign(v, &i, h.sphereradius());
	for (int j = 0; j < h.fingers_size(); j++) {
		Finger fi = h.fingers(j);
		assign(v, &i, fi.length());
		assign(v, &i, fi.width());
		assign(v, &i, fi.tipposition());
		assign(v, &i, fi.tipvelocity());
		for (int k = 0; k < fi.bones_size(); k++) {
			Bone b = fi.bones(k);
			assign(v, &i, b.length());
			assign(v, &i, b.width());
			assign(v, &i, b.center());
			assign(v, &i, b.direction());
		}
	}
	return v;
}

float* myotoVector(MyoFrame& h) {
	float* v = new float[MYO_VECTOR_SIZE];
	size_t i = 0;
	/**
		"state", 
	"onArm", "whichArm",
	"isUnlocked", "batteryLevel", "rssi", 
	"roll", "pitch", "yaw", 
	"acc_x", "acc_y", "acc_z", 
	"gyro_x", "gyro_y", "gyro_z",
	"emg1", "emg2", "emg3", 
	"emg4", "emg5", "emg6", 
	"emg7", "emg8"};
	*/
	assign(v, &i, static_cast<float>(h.state()));
	assign(v, &i, static_cast<float>(h.onarm()));
	assign(v, &i, static_cast<float>(h.whicharm()));
	assign(v, &i, static_cast<float>(h.isunlocked()));
	assign(v, &i, static_cast<float>(h.batterylevel()));
	assign(v, &i, static_cast<float>(h.rssi()));
	assign(v, &i, static_cast<float>(h.roll()));
	assign(v, &i, static_cast<float>(h.pitch()));
	assign(v, &i, static_cast<float>(h.yaw()));
	assign(v, &i, static_cast<float>(h.acc_x()));
	assign(v, &i, static_cast<float>(h.acc_y()));
	assign(v, &i, static_cast<float>(h.acc_z()));
	assign(v, &i, static_cast<float>(h.gyro_x()));
	assign(v, &i, static_cast<float>(h.gyro_y()));
	assign(v, &i, static_cast<float>(h.gyro_z()));
	assign(v, &i, static_cast<float>(h.emgsamples().Get(0)));
	assign(v, &i, static_cast<float>(h.emgsamples().Get(1)));
	assign(v, &i, static_cast<float>(h.emgsamples().Get(2)));
	assign(v, &i, static_cast<float>(h.emgsamples().Get(3)));
	assign(v, &i, static_cast<float>(h.emgsamples().Get(4)));
	assign(v, &i, static_cast<float>(h.emgsamples().Get(5)));
	assign(v, &i, static_cast<float>(h.emgsamples().Get(6)));
	assign(v, &i, static_cast<float>(h.emgsamples().Get(7)));
	return v;
}

float* myotoVector(MyoEvent& h) {
	float* v = new float[MYO_VECTOR_SIZE];
	size_t i = 0;
	/**
		"state", 
	"onArm", "whichArm",
	"isUnlocked", "batteryLevel", "rssi", 
	"roll", "pitch", "yaw", 
	"acc_x", "acc_y", "acc_z", 
	"gyro_x", "gyro_y", "gyro_z",
	"emg1", "emg2", "emg3", 
	"emg4", "emg5", "emg6", 
	"emg7", "emg8"};
	*/
	assign(v, &i, static_cast<float>(h.currentState));
	assign(v, &i, static_cast<float>(h.onArm));
	assign(v, &i, static_cast<float>(h.whichArm));
	assign(v, &i, static_cast<float>(h.isUnlocked));
	assign(v, &i, static_cast<float>(h.batteryLevel));
	assign(v, &i, static_cast<float>(h.rssi));
	assign(v, &i, static_cast<float>(h.roll_w));
	assign(v, &i, static_cast<float>(h.pitch_w));
	assign(v, &i, static_cast<float>(h.yaw_w));
	assign(v, &i, static_cast<float>(h.acc_x));
	assign(v, &i, static_cast<float>(h.acc_y));
	assign(v, &i, static_cast<float>(h.acc_z));
	assign(v, &i, static_cast<float>(h.gyro_x));
	assign(v, &i, static_cast<float>(h.gyro_y));
	assign(v, &i, static_cast<float>(h.gyro_z));
	assign(v, &i, static_cast<float>(h.emgSamples[0]));
	assign(v, &i, static_cast<float>(h.emgSamples[1]));
	assign(v, &i, static_cast<float>(h.emgSamples[2]));
	assign(v, &i, static_cast<float>(h.emgSamples[3]));
	assign(v, &i, static_cast<float>(h.emgSamples[4]));
	assign(v, &i, static_cast<float>(h.emgSamples[5]));
	assign(v, &i, static_cast<float>(h.emgSamples[6]));
	assign(v, &i, static_cast<float>(h.emgSamples[7]));
	return v;
}

void copy(H5File* file, std::queue<Frame>& que, std::queue<Gesture>& gesture, std::queue<UserAttention>& queAttention) {
	hsize_t i_counter = 0;
	std::vector<float*> data;
	std::vector<float*> attention_vectors;
	std::vector<int32_t> ids;
	std::vector<int64> timestamps;
	std::map <std::string, int> a2i_map;
	std::vector<UserAttention> attention;
	size_t aidx = 0;
	// Copy attention queue to vector
	while (!queAttention.empty()) {
	  attention.push_back(queAttention.front());
	  queAttention.pop();
	}
	int i;
    for(i = 0; i < g_attentionList.size(); i++)
	{
	  a2i_map[g_attentionList[i]] = i;
	}

	while (!que.empty()) {
		Frame f = que.front();
		que.pop();
		/* -------------------------------- ATTENTION ----------------------------------  */
		float* v_att = new float[ATTENTION_SIZE];
		for(int a = 0; a < ATTENTION_SIZE; a++)
		{
		  v_att[a] = 0.f;
		}

		  if(attention[aidx].timestamp() < f.timestamp())
		  {
			for (int j = 0; j < attention[aidx].areas_size(); j++) 
			{
			  const nui::stream::AttentionArea& area = attention[aidx].areas(j);
			  if(a2i_map[area.label()])
			  {
				  v_att[a2i_map[area.label()]] = area.probability();
			  }
			}
		  
		  
          if(attention.size() - 1 > aidx && attention[aidx + 1].timestamp() <= f.timestamp())
			{
              aidx++;
			}
		}
		attention_vectors.push_back(v_att); 
		/* ------------------------------------------------------------------------------  */
		for (int j = 0; j < f.hands_size(); j++) {
			Hand h = f.hands(j);
			data.push_back(handToVector(h));
			ids.push_back(h.id());
			timestamps.push_back(f.timestamp());
			i_counter += 1;
		}
	}
	size_t big_dim = i_counter * LEAP_VECTOR_SIZE;
	size_t att_dim = i_counter * ATTENTION_SIZE;
	float* farray = new float[big_dim];
	float* att_array = new float[att_dim];
	int* target   = new int[big_dim];
	
	size_t idx = 0;
	for (std::vector<float*> ::iterator it = data.begin() ; it != data.end(); ++it)
	{
		float* f_row = *it;
		for( int i = 0; i < LEAP_VECTOR_SIZE; i++)
		{
			farray[idx] = f_row[i];
			target[idx] = 0;
			idx += 1;	
		}
	}
	idx = 0;
	for (std::vector<float*> ::iterator it = attention_vectors.begin() ; it != attention_vectors.end(); ++it)
	{
		float* f_row = *it;
		for( int i = 0; i < ATTENTION_SIZE; i++)
		{
			att_array[idx] = f_row[i];
			idx += 1;
		}
	}

	hsize_t dim_2d[2]  = { i_counter, LEAP_VECTOR_SIZE };      // dataset dimensions at creation
	hsize_t dim_1d[1]  = { i_counter };
	hsize_t dim_attention[2]  = { i_counter,     ATTENTION_SIZE};
	hsize_t maxdims[2]        = { H5S_UNLIMITED, LEAP_VECTOR_SIZE };
	/* -------------------------------- DATA GROUP --------------------------------  */
	Group* data_group = new Group(file->createGroup("/data"));
	/* -------------------------------- DATA SPACE --------------------------------  */
	// Create the data space for the dataset.
	DataSpace *ds_id         = new DataSpace(1, dim_1d);
	DataSpace *ds_timestamp  = new DataSpace(1, dim_1d);
	DataSpace *ds_attentions = new DataSpace(2, dim_attention);
	DataSpace *ds_target     = new DataSpace(1, dim_1d);
	DataSpace *ds_signal     = new DataSpace(2, dim_2d);
	/* -------------------------------- DATA SET ----------------------------------  */
	DataSet *dset_ids = new DataSet(data_group->createDataSet("id",
					PredType::NATIVE_INT32,
					*ds_id));
	DataSet *dset_timestamps = new DataSet(data_group->createDataSet("timestamp",
					PredType::NATIVE_INT64,
					*ds_timestamp));
	DataSet *dset_attentions = new DataSet(data_group->createDataSet("attention",
					PredType::NATIVE_FLOAT,
					*ds_attentions));
	DataSet *dset_target = new DataSet(data_group->createDataSet("target",
					PredType::NATIVE_INT32,
					*ds_target));
	DataSet *dset_signal = new DataSet(data_group->createDataSet("signal", 
					PredType::NATIVE_FLOAT, 
					*ds_signal));
	/* -------------------------------- WRITE DATA --------------------------------  */
	dset_ids->write(ids.data(), PredType::NATIVE_INT32);
	dset_timestamps->write(timestamps.data(), PredType::NATIVE_INT64);
	dset_target->write(target, PredType::NATIVE_INT32);
	dset_signal->write(farray, PredType::NATIVE_FLOAT);
	dset_attentions->write(att_array, PredType::NATIVE_FLOAT);
	delete ds_id;
	delete ds_timestamp;
	delete ds_signal;
	delete dset_ids;
	delete dset_timestamps;
	delete dset_attentions;
	delete dset_signal;
	delete data_group;
}

void eventcopy(H5File* file, std::queue<MyoEvent>& que) {
	hsize_t i_counter = 0;
	std::vector<float*> data;
	std::vector<int32_t> ids;
	std::vector<int64> timestamps;
	size_t aidx = 0;

	while (!que.empty()) {
		MyoEvent f = que.front();
		que.pop();
		/* ------------------------------------------------------------------------------  */
		data.push_back(myotoVector(f));
		ids.push_back(f.event_id());
		timestamps.push_back(f.timestamp);
		i_counter += 1;
	}
	size_t big_dim = i_counter * MYO_VECTOR_SIZE;
	float* farray = new float[big_dim];
	int* target   = new int[big_dim];
	
	size_t idx = 0;
	for (std::vector<float*> ::iterator it = data.begin() ; it != data.end(); ++it)
	{
		float* f_row = *it;
		for( int i = 0; i < MYO_VECTOR_SIZE; i++)
		{
			farray[idx] = f_row[i];
			target[idx] = 0;
			idx += 1;
		}
	}
	idx = 0;
	hsize_t dim_2d[2]  = { i_counter, MYO_VECTOR_SIZE };      // dataset dimensions at creation
	hsize_t dim_1d[1]  = { i_counter };
	hsize_t maxdims[2]        = { H5S_UNLIMITED, MYO_VECTOR_SIZE };
	/* -------------------------------- DATA GROUP --------------------------------  */
	Group* data_group = new Group(file->createGroup("/data"));
	/* -------------------------------- DATA SPACE --------------------------------  */
	// Create the data space for the dataset.
	DataSpace *ds_id         = new DataSpace(1, dim_1d);
	DataSpace *ds_timestamp  = new DataSpace(1, dim_1d);
	DataSpace *ds_target     = new DataSpace(1, dim_1d);
	DataSpace *ds_signal     = new DataSpace(2, dim_2d);
	/* -------------------------------- DATA SET ----------------------------------  */
	DataSet *dset_ids = new DataSet(data_group->createDataSet("id",
					PredType::NATIVE_INT32,
					*ds_id));
	DataSet *dset_timestamps = new DataSet(data_group->createDataSet("timestamp",
					PredType::NATIVE_INT64,
					*ds_timestamp));
	DataSet *dset_target = new DataSet(data_group->createDataSet("target",
					PredType::NATIVE_INT32,
					*ds_target));
	DataSet *dset_signal = new DataSet(data_group->createDataSet("signal", 
					PredType::NATIVE_FLOAT, 
					*ds_signal));
	/* -------------------------------- WRITE DATA --------------------------------  */
	dset_ids->write(ids.data(), PredType::NATIVE_INT32);
	dset_timestamps->write(timestamps.data(), PredType::NATIVE_INT64);
	dset_target->write(target, PredType::NATIVE_INT32);
	dset_signal->write(farray, PredType::NATIVE_FLOAT);
	delete ds_id;
	delete ds_timestamp;
	delete ds_signal;
	delete dset_ids;
	delete dset_timestamps;
	delete dset_signal;
	delete data_group;
}

void copyMyo(std::queue<MyoFrame>& cache, MyoList* list) {
	for (int i = 0; i < list->frames_size(); i++) {
		const MyoFrame& frame = list->frames(i);
		cache.push(frame);
	}
}

void eventcopy(H5File* file, std::queue<MyoFrame>& que) {
	hsize_t i_counter = 0;
	std::vector<float*> data;
	std::vector<int32_t> ids;
	std::vector<int64> timestamps;
	size_t aidx = 0;

	while (!que.empty()) {
		MyoFrame f = que.front();
		que.pop();
		/* ------------------------------------------------------------------------------  */
		data.push_back(myotoVector(f));
		ids.push_back(f.id());
		timestamps.push_back(f.timestamp());
		i_counter += 1;
	}
	size_t big_dim = i_counter * MYO_VECTOR_SIZE;
	float* farray = new float[big_dim];
	int* target   = new int[big_dim];
	
	size_t idx = 0;
	for (std::vector<float*> ::iterator it = data.begin() ; it != data.end(); ++it)
	{
		float* f_row = *it;
		for( int i = 0; i < MYO_VECTOR_SIZE; i++)
		{
			farray[idx] = f_row[i];
			target[idx] = 0;
			idx += 1;
		}
	}
	idx = 0;
	hsize_t dim_2d[2]  = { i_counter, MYO_VECTOR_SIZE };      // dataset dimensions at creation
	hsize_t dim_1d[1]  = { i_counter };
	hsize_t maxdims[2]        = { H5S_UNLIMITED, MYO_VECTOR_SIZE };
	/* -------------------------------- DATA GROUP --------------------------------  */
	Group* data_group = new Group(file->createGroup("/data"));
	/* -------------------------------- DATA SPACE --------------------------------  */
	// Create the data space for the dataset.
	DataSpace *ds_id         = new DataSpace(1, dim_1d);
	DataSpace *ds_timestamp  = new DataSpace(1, dim_1d);
	DataSpace *ds_target     = new DataSpace(1, dim_1d);
	DataSpace *ds_signal     = new DataSpace(2, dim_2d);
	/* -------------------------------- DATA SET ----------------------------------  */
	DataSet *dset_ids = new DataSet(data_group->createDataSet("id",
					PredType::NATIVE_INT32,
					*ds_id));
	DataSet *dset_timestamps = new DataSet(data_group->createDataSet("timestamp",
					PredType::NATIVE_INT64,
					*ds_timestamp));
	DataSet *dset_target = new DataSet(data_group->createDataSet("target",
					PredType::NATIVE_INT32,
					*ds_target));
	DataSet *dset_signal = new DataSet(data_group->createDataSet("signal", 
					PredType::NATIVE_FLOAT, 
					*ds_signal));
	/* -------------------------------- WRITE DATA --------------------------------  */
	dset_ids->write(ids.data(), PredType::NATIVE_INT32);
	dset_timestamps->write(timestamps.data(), PredType::NATIVE_INT64);
	dset_target->write(target, PredType::NATIVE_INT32);
	dset_signal->write(farray, PredType::NATIVE_FLOAT);
	delete ds_id;
	delete ds_timestamp;
	delete ds_signal;
	delete dset_ids;
	delete dset_timestamps;
	delete dset_signal;
	delete data_group;
}

void configure(const H5File* file, const size_t vectorSize, const char* desc[]) {
	// Try block to detect exceptions raised by any of the calls inside it

	try {
		// Turn off the auto-printing when failure occurs so that we can
		// handle the errors appropriately
		Exception::dontPrint();
		// String data type
		StrType strType(PredType::C_S1, H5T_VARIABLE);
		// ------------------------------- Create a group in the file ---------------------------------
		Group* data_desc = new Group(file->createGroup("/data_desc"));
		// --------------------------------------------------------------------------------------------
		DSetCreatPropList* prop_des = new DSetCreatPropList;
		hsize_t dim[1];
		dim[0] = vectorSize;
		DataSpace* ds = new DataSpace(1, dim);
		DataSet* dset_description = new DataSet(
				data_desc->createDataSet("description", strType, *ds,
						*prop_des));
		dset_description->write(desc, strType);
		delete dset_description;
		delete prop_des;
		delete ds;
		// --------------------------------------------------------------------------------------------
		if(g_classLabels.size() > 0)
		{
			hsize_t dim_classes[1];
			const char** str_arr = new const char*[g_classLabels.size()];
			dim_classes[0] = g_classLabels.size();
			DataSpace* ds_classes = new DataSpace(1, dim_classes);
			DataSet* dset_classes = new DataSet(
					data_desc->createDataSet("classes", strType, *ds_classes));
			for(std::vector<std::string>::size_type i = 0; i != g_classLabels.size(); i++) 
			{
				str_arr[i] = g_classLabels[i].c_str();	
			}
			try {
				dset_classes->write(str_arr, strType);
			} 
			catch(std::exception e)
			{
				std::cerr << e.what() << std::endl;
			}
			delete str_arr;
			delete ds_classes;
			delete dset_classes;
		}
		// --------------------------------------------------------------------------------------------
		if(g_attentionList.size() > 0)
		{
			hsize_t dim_attention[1];
			dim_attention[0] = g_attentionList.size();
			DataSpace* ds_attention = new DataSpace(1, dim_attention);
			DataSet* dset_attention = new DataSet(
					data_desc->createDataSet("attention", strType, *ds_attention));
			dset_attention->write(g_attentionList[0], strType);
			delete ds_attention;
			delete dset_attention;
		}
		// --------------------------------------------------------------------------------------------
		delete data_desc;
	}  // end of try block
	   // catch failure caused by the H5File operations
	catch (FileIException& error) {
		error.printError();
	}
	// catch failure caused by the DataSet operations
	catch (DataSetIException& error) {
		error.printError();
	}
	// catch failure caused by the DataSpace operations
	catch (DataSpaceIException& error) {
		error.printError();
	}
}

void hdf5_serialize(std::queue<LeapTrackingEvent>& evt, const bfs::path path) {

}

void hdf5_serialize(std::queue<Frame>& evtQueue, const bfs::path path) {
	H5File* file = create_or_open(path.string());
	std::queue<Gesture> gesture;
	std::queue<UserAttention> attention;
	// Configures the HDF5 file
	configure(file, LEAP_VECTOR_SIZE, DATA_DESCRIPTION_LEAP);
	// Copies the data to the HDF5 file
	copy(file, evtQueue, gesture, attention);
	// closing the HDF5 file
	file->close();
	delete file;
}

void hdf5_serialize(std::queue<Frame>& leap,
				    std::queue<Gesture>& gesture,
				    std::queue<UserAttention>& attention,
				    const bfs::path path)
{
	H5File* file = create_or_open(path.string());
	// Configures the HDF5 file
	configure(file, LEAP_VECTOR_SIZE, DATA_DESCRIPTION_LEAP);
	// Copies the data to the HDF5 file
	copy(file, leap, gesture, attention);
	// closing the HDF5 file
	file->close();
	delete file;
}

void hdf5_serialize(std::queue<nui::events::MyoEvent>& myoEvents,
				    const bfs::path path)
{
	H5File* file = create_or_open(path.string());
	// Configures the HDF5 file
	
	configure(file, MYO_VECTOR_SIZE, DATA_DESCRIPTION_MYO);
	eventcopy(file, myoEvents);
	// closing the HDF5 file
	file->close();
	delete file;
}

void hdf5_serialize(std::queue<nui::stream::MyoFrame>& myoEvents,
				    const bfs::path path)
{
	H5File* file = create_or_open(path.string());
	// Configures the HDF5 file
	configure(file, MYO_VECTOR_SIZE, DATA_DESCRIPTION_MYO);
	eventcopy(file, myoEvents);
	// closing the HDF5 file
	
	file->close();
	delete file;
}


void hdf5_gbuff_cache_files(std::queue<bfs::path>& cache_queue, const bfs::path outpath)
{
	std::queue<MyoFrame>      myo_cache;
	while(!cache_queue.empty()) {
		bfs::path p = cache_queue.front();
		cache_queue.pop();
		MyoList* list = new MyoList();
		std::fstream input(p.string().c_str(), std::ios::in | std::ios::binary);
		if (!list->ParseFromIstream(&input)) {
			std::cerr << "Failed to parse cache." << endl;
		}
		copyMyo(myo_cache, list);
		delete list;
	}
	if(!myo_cache.empty())
	{
		hdf5_serialize(myo_cache, outpath);
	}
}