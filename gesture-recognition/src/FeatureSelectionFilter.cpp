#include "FeatureSelectionFilter.hpp"

namespace GRT{
    
//Register the FeatureSelectionFilter module with the PreProcessing base class
RegisterPreProcessingModule< FeatureSelectionFilter > FeatureSelectionFilter::registerModule("FeatureSelectionFilter");

FeatureSelectionFilter::FeatureSelectionFilter(const boost::dynamic_bitset<> feature_selection, UINT outsize)
{
    preProcessingType = "FeatureSelectionFilter";
    debugLog.setProceedingText("[DEBUG DOUBLE MOVING AVERAGE FILTER]");
    errorLog.setProceedingText("[ERROR DOUBLE MOVING AVERAGE FILTER]");
    warningLog.setProceedingText("[WARNING DOUBLE MOVING AVERAGE FILTER]");
    m_feature_selection = feature_selection;
	numOutputDimensions = outsize;
}

FeatureSelectionFilter::FeatureSelectionFilter()
{
    preProcessingType = "FeatureSelectionFilter";
    debugLog.setProceedingText("[DEBUG DOUBLE MOVING AVERAGE FILTER]");
    errorLog.setProceedingText("[ERROR DOUBLE MOVING AVERAGE FILTER]");
    warningLog.setProceedingText("[WARNING DOUBLE MOVING AVERAGE FILTER]");
}

FeatureSelectionFilter::FeatureSelectionFilter(const FeatureSelectionFilter &rhs)
{
	this->m_feature_selection = rhs.m_feature_selection;
    //Copy the base variables
    copyBaseVariables( (PreProcessing*)&rhs );
}
    
FeatureSelectionFilter::~FeatureSelectionFilter()
{

}
    
FeatureSelectionFilter& FeatureSelectionFilter::operator=(const FeatureSelectionFilter &rhs){
    if(this!=&rhs){
        this->m_feature_selection = rhs.m_feature_selection;

        
        //Copy the base variables
        copyBaseVariables( (PreProcessing*)&rhs );
    }
    return *this;
}
    
bool FeatureSelectionFilter::clone(const PreProcessing *preProcessing){
    
    if( preProcessing == NULL ) return false;
    
    if( this->getPreProcessingType() == preProcessing->getPreProcessingType() ){
        
        FeatureSelectionFilter *ptr = (FeatureSelectionFilter*)preProcessing;
        //Clone the classLabelTimeoutFilter values 
        this->m_feature_selection = ptr->m_feature_selection;

        //Clone the preprocessing base variables
        return copyBaseVariables( preProcessing );
    }
    
    errorLog << "clone(const PreProcessing *preProcessing) -  PreProcessing Types Do Not Match!" << endl;
    
    return false;
}

    
bool FeatureSelectionFilter::process(const vector< double > &inputVector)
{
    processedData = filter( inputVector );
    if( processedData.size() == numOutputDimensions ) return true;
    return false;
}

bool FeatureSelectionFilter::reset(){

    return true;
}
    
bool FeatureSelectionFilter::saveSettingsToFile(string filename){
    
    if( !initialized ){
        errorLog << "saveSettingsToFile(string filename) - The FeatureSelectionFilter has not been initialized" << endl;
        return false;
    }
    
    std::fstream file; 
    file.open(filename.c_str(), std::ios::out);
    
    if( !saveSettingsToFile( file ) ){
        file.close();
        return false;
    }
    
    file.close();
    
    return true;
}
    
bool FeatureSelectionFilter::saveSettingsToFile(fstream &file){
    
    if( !file.is_open() ){
        errorLog << "saveSettingsToFile(fstream &file) - The file is not open!" << endl;
        return false;
    }
    
    file << "GRT_DOUBLE_MOVING_AVERAGE_FILTER_FILE_V1.0" << endl;
    
    file << "NumInputDimensions: " << numInputDimensions << endl;
    file << "NumOutputDimensions: " << numOutputDimensions << endl;

    
    return true;
}

bool FeatureSelectionFilter::loadSettingsFromFile(string filename){
    
    std::fstream file; 
    file.open(filename.c_str(), std::ios::in);
    
    if( !loadSettingsFromFile( file ) ){
        file.close();
        initialized = false;
        return false;
    }
    
    file.close();
    
    return true;
}

bool FeatureSelectionFilter::loadSettingsFromFile(fstream &file)
{
  
    //Init the filter module to ensure everything is initialized correctly
    return true; //init(filterSize,numInputDimensions);  
}


    
vector< double > FeatureSelectionFilter::filter(const vector< double > &x)
{
	vector< double > selectionVec;
	UINT f_i;
	for(f_i = 0; f_i < m_feature_selection.size(); f_i++)
		{
			if(m_feature_selection[f_i])
			{
				selectionVec.push_back( x[f_i]);
			}
		}
    return selectionVec;
}

}//End of namespace GRT