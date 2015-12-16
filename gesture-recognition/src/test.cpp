#include <string>
#include "GestureRecognition.hpp"
#include <utility>
#include <vector>
#include "H5Cpp.h"
#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/tokenizer.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/property_tree/ptree.hpp>
using namespace GesCo;


int main(int argc, char* argv[])
{
	/**
	 * Path to the training data.
	 */
	bfs::path train = "G:\\data\\IEE_data\\training";
	/**
	 * Path to the test data.
	 */
	bfs::path test = "G:\\data\\IEE_data\\test";
	/**
	 * Path to the model
	 */
	bfs::path model = "..\\..\\..\\lstm_toolkit\\data\\gesture-control\\models";
	/**
	 * Setup up classifier system. 
	 */
	GesCoSystem system;
	/** First setup the system with configuration file. */
	system.setupSystem(model);
	/** Train the classifier system. */
	if(system.trainSystem(train))
	{
		/** Test the classifier system. */
		system.testSystem(test);
	}
	return 0;
}
