#include "NormalizationFilter.hpp"

namespace GRT{
    
//Register the NormalizationFilter module with the PreProcessing base class
RegisterPreProcessingModule< NormalizationFilter > NormalizationFilter::registerModule("NormalizationFilter");

NormalizationFilter::NormalizationFilter(VectorDouble mean, VectorDouble normalize_factors)
{
    preProcessingType = "NormalizationFilter";
    debugLog.setProceedingText("[DEBUG DOUBLE MOVING AVERAGE FILTER]");
    errorLog.setProceedingText("[ERROR DOUBLE MOVING AVERAGE FILTER]");
    warningLog.setProceedingText("[WARNING DOUBLE MOVING AVERAGE FILTER]");
	this->m_mean              = mean;
	this->m_normalize_factors = normalize_factors;
	this->numOutputDimensions = this->numInputDimensions  = normalize_factors.size();
}
    
NormalizationFilter::NormalizationFilter(const NormalizationFilter &rhs)
{
	this->m_mean              = rhs.m_mean;
	this->m_normalize_factors = rhs.m_normalize_factors;
	this->numOutputDimensions = rhs.numOutputDimensions;
	this->numInputDimensions  = rhs.numInputDimensions;
    //Copy the base variables
    copyBaseVariables( (PreProcessing*)&rhs );
}
    
NormalizationFilter::~NormalizationFilter()
{

}
    
NormalizationFilter& NormalizationFilter::operator=(const NormalizationFilter &rhs){
    if(this!=&rhs)
	{
		this->m_mean              = rhs.m_mean;
		this->m_normalize_factors = rhs.m_normalize_factors;
		this->numOutputDimensions = rhs.numOutputDimensions;
		this->numInputDimensions  = rhs.numInputDimensions;
        //Copy the base variables
        copyBaseVariables( (PreProcessing*)&rhs );
    }
    return *this;
}
    
bool NormalizationFilter::clone(const PreProcessing *preProcessing)
{
    if( preProcessing == NULL ) return false;
    if( this->getPreProcessingType() == preProcessing->getPreProcessingType() )
	{    
        NormalizationFilter *ptr = (NormalizationFilter*)preProcessing;
        //Clone the classLabelTimeoutFilter values 
		this->m_mean              = ptr->m_mean;
		this->m_normalize_factors = ptr->m_normalize_factors; 
		this->numOutputDimensions = ptr->numOutputDimensions;
		this->numInputDimensions  = ptr->numInputDimensions;
        //Clone the preprocessing base variables
        return copyBaseVariables( preProcessing );
    }
    errorLog << "clone(const PreProcessing *preProcessing) -  PreProcessing Types Do Not Match!" << endl;
    return false;
}
    
bool NormalizationFilter::process(const vector< double > &inputVector)
{
	if( inputVector.size() != numInputDimensions ) return false;
    processedData = filter( inputVector );
    if( processedData.size() == numOutputDimensions ) return true;
    return false;
}

bool NormalizationFilter::reset(){
    return false;
}
    
bool NormalizationFilter::saveSettingsToFile(string filename)
{
    return true;
}
    
bool NormalizationFilter::saveSettingsToFile(fstream &file)
{
    return true;
}

bool NormalizationFilter::loadSettingsFromFile(string filename)
{   
    return true;
}

bool NormalizationFilter::loadSettingsFromFile(fstream &file)
{
	return true;
}
   
vector< double > NormalizationFilter::filter(const vector< double > &x)
{
	vector< double > normalizedVec;
	UINT f_i;
	for(f_i = 0; f_i < x.size(); f_i++)
		{
			if(f_i <m_normalize_factors.size()) 
			{
				double norm = (m_normalize_factors[f_i] == 0.) ? 0.0000000001 : m_normalize_factors[f_i];
				normalizedVec.push_back( (x[f_i] - m_mean[f_i]) / norm);
			}
		}
    return normalizedVec;
}

}//End of namespace GRT