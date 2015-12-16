#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/tokenizer.hpp>
#include <boost/range/algorithm.hpp>
#include <math.h>  
#include "LSTM.hpp"

// --------------------------------------------- Namespaces ---------------------------------------------
using namespace NeuralNetwork;
namespace bfs = boost::filesystem;
// ---------------------------------------- Type Definitions ---------------------------------------------
typedef const boost::property_tree::ptree::value_type ValueType;
// ------------------------------------------ Helper Functions -------------------------------------------
void parseParameters(std::string parameters, Weights& weights)
{
	boost::char_separator<char> sep("[] ,");
	boost::tokenizer< boost::char_separator<char> > tok(parameters, sep);
	std::vector<std::string> numbers;
	for(boost::tokenizer< boost::char_separator<char> >::iterator beg=tok.begin(); beg!=tok.end();++beg)
	{
		numbers.push_back(*beg);
	}
	size_t i = 0;
	weights.resize(1, numbers.size()); // Resize the weigths to a one-dimesion 
	BOOST_FOREACH(std::string n, numbers)
	{
		weights(0, i++) = atof(n.c_str());
	}
}

void vec_tanh(InputVector& inp, ActivationVector& outp)
{
	for(index_t i = 0; i < inp.rows(); i++)
	{
		outp[i] = tanh(inp[i]); 
	}
}

void vec_sigmoid(InputVector& inp, ActivationVector& outp)
{
	for(index_t i = 0; i < inp.rows(); i++)
	{
		outp[i] = 1. / (1. + exp(-inp[i])); 
	}
}

// --------------------------------------------- Registration --------------------------------------------
GRT::RegisterClassifierModule< LSTM > LSTM::registerModule("LSTM");
// --------------------------------------------- Copy Methods --------------------------------------------
LSTMLayer::LSTMLayer(index_t* offset, const std::string name, bool peepholes, const size_t dim) : 
Layer(name, LSTM_LAYER, 4 * dim, dim, offset), 
	m_sequential(true), 
	m_peepholes(peepholes),
	m_dim(dim)
{

}

bool LSTM::clone(const Classifier *classifier){
    if( classifier == NULL ) return false;
    
    if( this->getClassifierType() == classifier->getClassifierType() ){
        *this = *(LSTM*) classifier;
        return true;
    }
    return false;
}

LSTM::LSTM(const LSTM &rhs){
    *this = rhs;
}

LSTM& LSTM::operator=(const LSTM &rhs)
{
	if( this != &rhs )
	{
		this->m_in_layers = rhs.m_in_layers;
		this->m_out_layers = rhs.m_out_layers;
		this->m_connectionMap = rhs.m_connectionMap;
		this->m_recurrentConnections = rhs.m_recurrentConnections;
		this->m_vertex_map = rhs.m_vertex_map;
		this->m_layerGraph = rhs.m_layerGraph;
		this->m_sortedLayers = rhs.m_sortedLayers;
		this->m_in_layers = rhs.m_in_layers;
		this->m_out_layers = rhs.m_out_layers;
		this->m_connections = rhs.m_connections;
		this->m_indim = rhs.m_indim;
		this->m_outdim = rhs.m_outdim;
		this->m_offset = rhs.m_offset;
	    //Copy the classifier variables
		copyBaseVariables( (Classifier*)&rhs );
	}
	return *this;
}
// ------------------------------------------ LSTM Building network -------------------------------------------
void LSTM::configureLayers(const NetworkConfig& config)
{
	BOOST_FOREACH(ValueType &l, config.get_child("PyBrain.Network.Modules"))
	{
		try {
			const std::string type      = l.first;
			const std::string name_str  = l.second.get<std::string>("<xmlattr>.name"); 
			Layer* layer;
			bool in = false , out = false;

			if(boost::iequals(type, BIAS_LAYER))
			{
				layer = new BiasLayer(m_offset, name_str, 1u);
			} 
			else 
			{
				in  = boost::iequals(l.second.get("<xmlattr>.inmodule", "False"), "True"); 
				out = boost::iequals(l.second.get("<xmlattr>.outmodule", "False"), "True");  
				size_t val_size = l.second.get("dim.<xmlattr>.val", 0u);

				if (boost::iequals(type, TANH_LAYER))
				{
					 layer = new TanhLayer(m_offset, name_str, val_size);
				}
				else if (boost::iequals(type, LINEAR_LAYER))
				{
					layer = new LinearLayer(m_offset, name_str, val_size);
				}
				else if (boost::iequals(type, LSTM_LAYER))
				{
					const bool peepholes = boost::iequals(l.second.get("peepholes.<xmlattr>.val", "False"), "True"); 
					layer = new LSTMLayer(m_offset, name_str, 
										peepholes,
										val_size);
				}
			}
			if(in)
			{
				m_indim += layer->getInputSize();
				m_in_layers.push_back(layer);
			}
			else if(out)
			{
				m_outdim += layer->getOutputSize();
				m_out_layers.push_back(layer);
			}
			m_layers[name_str] = layer;
			// add vertex to the graph for topological sort
			index_t vid = boost::add_vertex(m_layerGraph);
			m_vertex_map.insert(index2name(vid, name_str));
		} 
		catch(const std::runtime_error be)
		{
			cerr << be.what() << endl;
		}
	}
}

void LSTM::connectLayers(const NetworkConfig& config)
{
	BOOST_FOREACH(ValueType &l, config.get_child("PyBrain.Network.Connections"))
	{
		const std::string type      = l.first;
		const std::string name_str  = l.second.get<std::string>("<xmlattr>.name"); 
		const bool recurrent        = boost::iequals(l.second.get("<xmlattr>.recurrent", "False"), "True"); 
		if(boost::iequals(type, FULL_CONNECTION))
		{
			const std::string in_layer  = l.second.get<std::string>("inmod.<xmlattr>.val"); 
			const std::string out_layer = l.second.get<std::string>("outmod.<xmlattr>.val"); 
			const std::string parameters = l.second.get<std::string>("Parameters");
			index_t v1 = m_vertex_map.right.at(in_layer);
			index_t v2 = m_vertex_map.right.at(out_layer);
			Layer* in  = m_layers[in_layer];
			Layer* out = m_layers[out_layer];
			Weights params;
			parseParameters(parameters, params);
			Connection* c = new FullConnection(name_str, in, out, params, recurrent);
			if(recurrent)
			{
				m_recurrentConnections.push_back(c);
			} 
			else
			{
				boost::add_edge(v1, v2, m_layerGraph); // avoid a cycle
				m_connections.push_back(c);
				m_connectionMap[in_layer].push_back(c);
			}
		}
	}
}

void LSTM::sortLayers()
{
	// Perform a topological sort.
    std::deque<int> topo_order;
	boost::topological_sort(m_layerGraph, std::front_inserter(topo_order)); // We need to do a graph based topological sort
    for(std::deque<int>::const_iterator i = topo_order.begin();
        i != topo_order.end();
        ++i)
    {
		m_sortedLayers.push_back(m_layers[m_vertex_map.left.at(*i)]);
    }
	m_sorted = true;
}

void LSTM::buildNetwork(const NetworkConfig& config)
{
	configureLayers(config); // First read the layers
	connectLayers(config);   // Connect the layers
	sortLayers();            // Sort layers
}

// ------------------------------------------ LSTM Loading functions -------------------------------------------
bool LSTM::loadModelFromFile(fstream &file)
{
	NetworkConfig config;  
	read_xml(file, config);  
	buildNetwork(config);    
	// Build
	for(index_t i = 0; i < m_outdim; i++)
	{
		classDistances.push_back(0.);
		classLikelihoods.push_back(0.);
	}
	trained = true;
	return true;
}

bool LSTM::loadModelFromFile(string filename)
{
    std::fstream file;
    file.open(filename.c_str(), std::ios::in);
    if( !loadModelFromFile( file ) ){
        return false;
    }
    file.close();
    return true;
}

bool LSTM::saveModelToFile(string filename)
{
	// not need yet... Training is implemented in Python
	return true;
}

bool LSTM::saveModelToFile(fstream &file)
{
	// not need yet... Training is implemented in Python
	return true;
}
void LSTM::updateHistory()
{
	/*
	 * TODO: Improve memory consumption
	 */
	//if(	(*m_offset) < MAX_HISTORY) 
	{
		(*m_offset)++;
	}
	//else
	{
		/*BOOST_FOREACH(Layer* l, m_sortedLayers) 
		{
			l->clearHistory(MAX_HISTORY);
		}*/
	}

}
// ------------------------------------------ LSTM Evaluation functions -------------------------------------------
void LSTM::forward()
{
	index_t offset = *m_offset;
	InputVector& inp = m_input_buffer[offset];
	/**
	 * Forgetting
	 */
	if(this->m_forget) this->m_offset++;
	index_t idx = 0;
	// First update input layers
	BOOST_FOREACH(Layer* l, m_in_layers) 
	{ 
		l->m_inbuffer.push_back(inp);
	}
	// Update recurrent connections
	if(offset > 0) 
	{
		BOOST_FOREACH(Connection* c, m_recurrentConnections) 
		{ 
			c->forward(offset-1, offset);
		}
	}
	// Update layer chain
	BOOST_FOREACH(Layer* l, m_sortedLayers) 
	{
		l->forward();
		BOOST_FOREACH(Connection* c, m_connectionMap[l->getName()]) 
		{
			c->forward(offset, offset);
		}
	}
	// Forget input
	if(m_forget)
	{
		BOOST_FOREACH(Layer* l, m_sortedLayers)
		{
			l->shift(-1);
		}
	}
	// Updating output layers
	index_t o_i = 0;
	index_t b_i = 0;
	BOOST_FOREACH(Layer* l, m_out_layers) // if there are several output layers
	{ 
		for(b_i = 0; b_i < l->getOutputSize(); b_i++)
		{
			m_output_buffer[offset][o_i++] =  l->m_outbuffer[offset][b_i];
		}
	}
	updateHistory();
}


bool LSTM::predict(GRT::VectorDouble inputVector)
{
	InputVector      in(m_indim);
	ActivationVector out(m_outdim);
	// Copy input
	for(size_t i = 0; i < inputVector.size();i++) in[i] = inputVector[i];
	out.setZero();
	m_input_buffer.push_back(in);   // add vector to buffer
	m_output_buffer.push_back(out);
	forward();

	index_t offset = *m_offset;
	if(m_forget)
	{
		out = this->m_output_buffer[offset];
	} 
	else
	{
		out = this->m_output_buffer[offset-1];
	}
	double max = 0.;
	index_t max_i = 0;
	for(index_t i = 0; i < out.rows(); i++)
	{
		if(out[i] > max)
		{
			max = out[i];
			max_i = i;
		}
		classLikelihoods[i] = out[i];
	}
	predictedClassLabel = max_i;
	return true;
}

bool LSTM::reset()
{
	m_input_buffer.clear();
	m_output_buffer.clear();
	return true;
}
// ------------------------------------------ Connections Functions -------------------------------------------
Connection& Connection::operator=(const Connection &c)
{
	if( this != &c )
	{
		Connection* nc = new Connection(c);
		return *nc;
	}
	return *this;
}

void FullConnection::forward(const index_t inpIndex, const index_t outIndex)
{
	InputVector input = m_parameters * m_in_layer->m_outbuffer[inpIndex];
	m_out_layer->addInput(input, outIndex);
}
// ------------------------------------------ Layer Functions -------------------------------------------
Layer& Layer::operator=(const Layer &c)
{
	if( this != &c )
	{
		Layer* nc = new Layer(c);
		return *nc;
	}
	return *this;
}

void Layer::addInput(InputVector v, index_t i)  
{
	while(i >= m_inbuffer.size())
	{
		InputVector inp(this->m_inputSize);
		inp.setZero();
		appendInput(inp);
	}
	m_inbuffer[i] += v;
};

void Layer::shift(int shift)
{

}

void Layer::clearHistory(size_t max)
{
	if(m_inbuffer.size() >= max)
	{
		m_inbuffer.erase(m_inbuffer.begin());
		m_outbuffer.erase(m_outbuffer.begin());
	}
}

void Layer::forward()
{
	index_t offset = *m_offset;
	if(this->m_inbuffer.size() <= offset)
	{
		InputVector inp;
		inp.resize(this->getInputSize());
		inp.setZero();
		m_inbuffer.push_back(inp);
	}
	if(this->m_outbuffer.size() <= offset)
	{
		ActivationVector out;
		out.resize(this->getOutputSize());
		m_outbuffer.push_back(out);
	}		
	forward(this->m_inbuffer[offset],this->m_outbuffer[offset]);
}

InputVector LSTMLayer::f(InputVector& in)
{
	InputVector out(m_outputSize);
	vec_sigmoid(in, out);
	return out;
}

InputVector LSTMLayer::h(InputVector& in)
{
	InputVector out(m_outputSize);
	vec_tanh(in, out);
	return out;
}

InputVector LSTMLayer::g(InputVector& in)
{
	InputVector out(m_outputSize);
	vec_tanh(in, out);
	return out;
}

void LSTMLayer::forward(InputVector& inp, ActivationVector& outp)
{
	// ------------------------ Get offset ----------------------
	index_t offset = *m_offset;
	// ----------------------------------------------------------
	InputVector v_in(this->m_dim);
	v_in = inp.head(this->m_dim);
	InputVector v_f(this->m_dim);
	v_f = inp.segment(this->m_dim, this->m_dim);
	InputVector v_cell(this->m_dim);
	v_cell = inp.segment(this->m_dim*2, this->m_dim);
	InputVector v_out(this->m_dim);
	v_out = inp.segment(this->m_dim*3, this->m_dim);
	// Grow buffers
	if(offset >= m_ingatex.size())
	{
		m_ingatex.push_back(v_in);
	}
	if(offset >= m_forgetgatex.size())
	{
		m_forgetgatex.push_back(v_f);
	}
	if(offset >= m_outgatex.size())
	{
		m_outgatex.push_back(v_out);
	}
	if(this->m_peepholes && offset > 0)
	{
		 this->m_ingatex[offset] += this->m_ingatePeepWeights * this->m_state[offset-1];
	}
	this->m_ingate.push_back(f(m_ingatex[offset]));
    this->m_forgetgate.push_back(f(this->m_forgetgatex[offset]));
    this->m_state.push_back(this->m_ingate[offset].cwiseProduct(g(v_cell)));
    if (offset > 0)
	{
        this->m_state[offset] += this->m_forgetgate[offset].cwiseProduct(this->m_state[offset-1]);
	}
    if (this->m_peepholes)
	{
        this->m_outgatex[offset] += this->m_outgatePeepWeights * this->m_state[offset];
	}
    this->m_outgate.push_back(f(this->m_outgatex[offset]));
    outp = this->m_outgate[offset].cwiseProduct(h(this->m_state[offset]));
}

void BiasLayer::forward(InputVector& inp, ActivationVector& outp)
{
	outp.setOnes(); // All neurons set to 1.
}

void LinearLayer::forward(InputVector& inp, ActivationVector& outp)
{
	for(index_t i = 0; i < this->m_outputSize; i++)
	{
		outp[i] = inp[i]; // Simplest kind of layer, not doing any transformation.
	}
}

void TanhLayer::forward(InputVector& inp, ActivationVector& outp)
{
	vec_tanh(inp, outp);
}

void SoftmaxLayer::forward(InputVector& inp, ActivationVector& outp)
{
	for(index_t i = 0; this->m_outputSize; i++)
	{
		outp[i] = 0.; // TODO implement Softmax Layer
	}
}
