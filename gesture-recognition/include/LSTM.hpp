/*
 * This file is part of the IEE SA Gesture Control project.
 * Copyright (C) 2013 DFKI GmbH. All rights reserved.
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
#include <boost/bimap/bimap.hpp>
#include <boost/filesystem.hpp>
#include "boost/graph/adjacency_list.hpp"
#include <boost/graph/topological_sort.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <Eigen/Core>
#include "GRT/GRT.h"
#include <vector>
#include <map>
using namespace Eigen;
namespace bpt = boost::property_tree;
// ------------------------------------------     Typedefs     -------------------------------------------
typedef bpt::ptree NetworkConfig; 

namespace NeuralNetwork 
{
	// ------------------ Constants --------------------------------------------------
	const std::string LINEAR_LAYER    = "LinearLayer";
	const std::string TANH_LAYER      = "TanhLayer";
	const std::string BIAS_LAYER      = "BiasUnit";
	const std::string LSTM_LAYER      = "LSTMLayer" ;
	const std::string SOFTMAX_LAYER   = "SoftMaxLayer";
	const std::string FULL_CONNECTION = "FullConnection";
	const size_t      MAX_HISTORY     = 10u;
	// ------------------ Typedefs --------------------------------------------------
	typedef unsigned int                  index_t;
	typedef VectorXd                      InputVector;
	typedef VectorXd                      ActivationVector;
	typedef MatrixXd                      Weights;
	typedef std::vector<InputVector>      InputBuffer;
	typedef std::vector<ActivationVector> ActivationBuffer;

	/**
	 * \brief Each Layer has an input and output. 
	 * There is a specific transformation performed.
	 */
	class Layer 
	{
	public:
		/**
		 * Input buffer
		 */
		InputBuffer      m_inbuffer;
				/**
		 * Input buffer
		 */
		ActivationBuffer m_outbuffer;
		/**
		 * Offset for the buffer
		 */
		index_t*          m_offset;
		/**
		 * \brief Constructs a layer of the network.
		 * \param[in] name -- Name o the layer
		 * \param[in] type -- type of the layer
		 * \param[in] in   -- size of the input 
		 * \param[in] out  -- size of the output
		 */
		Layer(const std::string name, 
			  const std::string type, 
			  const size_t in, 
			  const size_t out,
			  index_t* offset) :
			  m_inputSize(in), 
			  m_outputSize(out), 
			  m_name(name), 
			  m_type(type)

		  {
			m_offset = offset;
		  };
		Layer(const Layer &c) :
		      m_inputSize(c.m_inputSize), 
			  m_outputSize(c.m_outputSize), 
			  m_name(c.m_name), 
			  m_type(c.m_type),
			  m_offset(c.m_offset)
		{
		};

		Layer& operator=(const Layer &c);
		/**
		 * Returns the name of the layer.
		 * \returns name of this layer
		 */
		std::string getName() { return m_name; };
		/**
		 * Returns the type of the layer.
		 * \returns type of this layer
		 */
		std::string getType() { return m_type; };
		/**
		 * \brief Setting the output size.
		 */
		void setOutPutSize(size_t s) 
		{ 
			m_outputSize = s;
		}
		/**
		 * \brief Appends input vector to the input buffer.
		 * \param[in] v -- input vector
		 */
		void appendInput(InputVector v)  
		{
			m_inbuffer.push_back(v);
		};
		/**
		 * \brief Adding input vector.
		 * \param[in] v - vector
		 * \param[in] i - index position where the vector should be added to. buffer[i] += v
		 */
		void addInput(InputVector v, index_t i=0);

		/**
		 * \brief Appends activation vector to the output buffer.
		 * \param[in] v -- input vector
		 */
		void appendOutput(ActivationVector v) 
		{
			m_outbuffer.push_back(v);
		};
	
		/**
		 * \brief Returns the input size of the layer.
		 * \return size 
		 */
		const size_t getInputSize() 
		{ 
			return m_inputSize; 
		};
		/**
		 * \brief Returns the output size of the layer.
		 * \return size 
		 */
		const size_t getOutputSize() 
		{ 
			return m_outputSize; 
		};
		/**
		 * \brief Forward evaluation of the network.
		 */
		void forward();
		/**
		 * \brief Clearing history.
		 * \param size -- maximum values in the history
		 */
		void clearHistory(size_t size);
		/**
		 * \brief Shifting
		 */
		void shift(int s);
	protected:
		/**
		 * \brief Input dimension.
		 */
		const size_t       m_inputSize;
		/**
		 * \brief Output dimension.
		 */
		size_t             m_outputSize;
		/**
		 * \brief Name of the layer.
		 */
		const std::string  m_name;
		/**
		 * \brief Type of the layer.
		 */
		const std::string  m_type;
		/**
		 * \brief Forward evaluation of the layer.
		 * \param[in]  inp  -- Input vector
		 * \param[out] outp -- Output vector
		 */
		virtual void forward(InputVector& inp, ActivationVector& outp){};
	};
	/**
	 * \brief Linear Layer.
	 * Linear layer is quite simple.
	 */
	class LinearLayer : public Layer {
	public:
		LinearLayer(index_t* offset, const std::string name, const size_t dim) : 
		  // Linear Layer | input and output size is the same
		  Layer(name, LINEAR_LAYER, dim, dim, offset) {
		  };
	protected:
		  virtual void forward(InputVector& inp, ActivationVector& outp);
	};
	 /**
	 * \brief Softmax Layer.
	 * Softmax.
	 */
	class SoftmaxLayer : public Layer {
	public:

		SoftmaxLayer(index_t* offset, const std::string name, const size_t in, const size_t out=0) : 
		  Layer(name, SOFTMAX_LAYER, in, out, offset) {

		  };
	protected:
		  virtual void forward(InputVector& inp, ActivationVector& outp);
	};
	/**
	 * \brief Tanh Layer.
	 * 
	 */
	class TanhLayer : public Layer {
	public:

		TanhLayer(index_t* offset, const std::string name, const size_t dim) : 
		  Layer(name, TANH_LAYER, dim, dim, offset) {

		  };
	protected:
		  virtual void forward(InputVector& inp, ActivationVector& outp);
	};
	/**
	 * \brief Bias Layer.
	 *
	 * Bias vector.
	 */
	class BiasLayer : public Layer {
	public:

		BiasLayer(index_t* offset, const std::string name, const size_t out=0) : 
		  Layer(name, BIAS_LAYER, 0, out, offset) {
		  };
	protected:
		  virtual void forward(InputVector& inp, ActivationVector& outp);
	};

	/**
	 * \brief LSTM Layer.
	 *
	 * Long short-term memory cell layer.
	 *
     * The input consists of 4 parts, in the following order:
     * - input gate
     * - forget gate
     * - cell input
     * - output gate
	 *
	 */
	class LSTMLayer : public Layer {
	public:

		LSTMLayer(index_t* offset, const std::string name, bool peepholes, const size_t dim);
		virtual void forward(InputVector& inp, ActivationVector& outp);
	protected:
		 bool m_sequential;
		 /** 
		  * Peep holes feature.
		  */
		 bool m_peepholes;
		 /**
		  * Dimension of the input.
		  */
		 const size_t m_dim;
		 Weights       m_ingatePeepWeights;
		 Weights       m_forgetgatePeepWeights;
         Weights       m_outgatePeepWeights;
		 InputBuffer   m_ingatex;
		 InputBuffer   m_forgetgatex;
		 InputBuffer   m_outgatex;
		 InputBuffer   m_state;
		 InputBuffer   m_outgate;
		 InputBuffer   m_ingate;
		 InputBuffer   m_forgetgate;

		 InputVector f(InputVector& in);
		 InputVector g(InputVector& in);
		 InputVector h(InputVector& in);

	};
	/** Collection of Network Layers */
	typedef std::vector<Layer*>                     Layers;
	/** List of the Layer names */
	//typedef std::vector<std::string>                LayerNames;
	typedef std::map<std::string, Layer*>           LayersMap;
	typedef std::map<std::string, Layer*>::iterator LayerIter;
	typedef boost::adjacency_list<>                 LayerGraph;
	typedef boost::bimaps::bimap<int, std::string>  LayerVerticesMap;
	typedef LayerVerticesMap::value_type            index2name; 
	/**
	 * \brief Connects two layers.
	 * 
	 */
	class Connection
	{
	public:
		Connection(const std::string name, 
			       Layer* in,
		           Layer* out,
				   Weights w, 
				   const bool recurrent) :
		  m_name(name), 
	      m_recurrent(recurrent), 
		  m_in_layer(in),
		  m_out_layer(out)
		  {
			 m_parameters.resize(m_out_layer->getInputSize(), m_in_layer->getOutputSize());
			 index_t p = 0;
			 for(index_t i = 0; i < m_out_layer->getInputSize(); i++)
			{
				for(index_t j = 0; j < m_in_layer->getOutputSize(); j++)
				{
					 m_parameters(i, j) = w(p++);
				 }
			 }
		  };

		Connection(const Connection &c) :
			m_in_layer(c.m_in_layer), 
			m_out_layer(c.m_out_layer),
			m_name(c.m_name),
			m_recurrent (c.m_recurrent),
			m_parameters(c.m_parameters)
		{

		};

		Connection& operator=(const Connection &c);
		/**
		 * \brief Forward evaluation of the network connection.
		 * 
		 * Propagate the information from the incoming module's output buffer, 
		 * adding it to the outgoing node's input buffer, and possibly transforming
		 * it on the way.
		 *
		 * For this transformation use inmodOffset as an offset for the inmod and
		 * outmodOffset as an offset for the outmodules offset.
		 */
		virtual void forward(const index_t inpIndex=0u, const index_t outIndex=0u) {};
		Weights          m_parameters;
	protected:
		const std::string  m_name;
		Layer* m_in_layer;
		Layer* m_out_layer;
		const bool m_recurrent;
	};
	/**
	 * \brief Full connections of layers.
	 * 
	 */
	class FullConnection : public Connection
	{
	public:
		FullConnection(const std::string name, 
			           Layer* in,
		               Layer* out, 
					   Weights vec, 
					   const bool recurrent) :
			Connection(name, in, out, vec, recurrent) 
		{
		};
		virtual void forward(const index_t inpIndex=0u, const index_t outIndex=0u);
	};
	/**
	 * List of connections.
	 */
	typedef std::vector<Connection*> Connections;
	/**
	 * Maps the name of a Layer to conncected connections.
	 */
	typedef std::map<std::string, std::vector<Connection*> > ConnectionMap;
	/**
	 * \brief Implementation of a Long Short Term Memory Network.
	 *
	 */
	class LSTM : public GRT::Classifier
	{
	public :
		LSTM() : m_numBlocks(0), m_indim(0), m_outdim(0),
				m_cellsPerBlock(0), m_numCells(0),
				m_gatesPerBlock(0), m_unitsPerBlock(0),
				m_peepsPerBlock(0), m_offset(NULL), m_forget(false) 
		{
			classifierType = "LSTM";
			classifierMode = TIMESERIES_CLASSIFIER_MODE;
			m_offset = new index_t;
			*m_offset = 0u;
		};
		virtual~LSTM() 
		{
		};

		LSTM(const LSTM &rhs);

		LSTM& operator=(const LSTM &rhs);
		/**
		 * \brief Returns the last activation vector.
		 * \return Activation vector
		 */
		ActivationVector lastActivation() { return m_output_buffer.size() > 0 ? m_output_buffer.back() : ActivationVector(m_outdim).setZero() ; };

		virtual bool clone(const Classifier *classifier);
		virtual bool reset();
   	    virtual bool predict(GRT::VectorDouble inputVector);
        virtual bool saveModelToFile(string filename);
        virtual bool saveModelToFile(fstream &file);
    	virtual bool loadModelFromFile(string filename);
        virtual bool loadModelFromFile(fstream &file);
	protected :	
		size_t           m_indim;
		size_t           m_outdim;
		size_t           m_numBlocks;
		size_t           m_cellsPerBlock;
		size_t           m_numCells;
		size_t           m_gatesPerBlock;
		size_t           m_unitsPerBlock;
		size_t           m_peepsPerBlock;
		Layers           m_in_layers;
		LayersMap        m_layers;
		Layers           m_out_layers;
		LayerGraph       m_layerGraph;
		Layers           m_sortedLayers;
		LayerVerticesMap m_vertex_map;
		Connections      m_recurrentConnections;
		Connections      m_connections;
		ConnectionMap    m_connectionMap;
		ActivationVector m_last_output;
		index_t*         m_offset;
		bool             m_forget;
		bool             m_sequential;
		bool             m_sorted;
		/**
		 * \brief Forward activation.
		 */
		void forward();
		/** 
		 * Updates the offset and histroy management.
		 */
		void updateHistory();
		/**
		 * \brief Build network from pybrain XML.
		 * \param[in] -- 
		 */
		void buildNetwork(const NetworkConfig& config);
		/**
		 * \brief Configure each layer. 
		 * \param config -- XML configuration of network
		 */
		void configureLayers(const NetworkConfig& config); 
		/**
		 * \brief Configures the connections. 
		 * \param config -- XML configuration of network
		 */
		void connectLayers(const NetworkConfig& config); 	
		/**
		 * \brief Sorts the layers of the network.
		 */
		void sortLayers();
	private:
		/**
		 * Input buffer.
		 */
		InputBuffer      m_input_buffer;
		/**
		 * Output buffer.
		 */
		ActivationBuffer m_output_buffer;
		/** 
		 * \brief Registering the module.
		 */
		static GRT::RegisterClassifierModule< LSTM > registerModule;
	};
};
