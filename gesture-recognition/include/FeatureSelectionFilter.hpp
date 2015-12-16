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

#include "GRT/CoreModules/PreProcessing.h"
#include <boost/dynamic_bitset.hpp>

namespace GRT
{

class FeatureSelectionFilter : public PreProcessing {
public:
    /**
     Constructor, sets the size of the double moving average filter and the dimensionality of the data it will filter.
	 
     @param UINT filterSize: the size of the moving average filter, should be a value greater than zero. Default filterSize = 5
     @param UINT numDimensions: the dimensionality of the data to filter.  Default numDimensions = 1
     */
    FeatureSelectionFilter(const boost::dynamic_bitset<>, UINT outsize);
	FeatureSelectionFilter();
    
    /**
     Copy Constructor, copies the DoubleMovingAverageFilter from the rhs instance to this instance
     
	 @param const DoubleMovingAverageFilter &rhs: another instance of the DoubleMovingAverageFilter class from which the data will be copied to this instance
     */
    FeatureSelectionFilter(const FeatureSelectionFilter &rhs);

    /**
     Default Destructor
     */
    virtual ~FeatureSelectionFilter();
    
    /**
     Sets the equals operator, copies the data from the rhs instance to this instance
     
	 @param const DoubleMovingAverageFilter &rhs: another instance of the DoubleMovingAverageFilter class from which the data will be copied to this instance
	 @return a reference to this instance of DoubleMovingAverageFilter
     */
    FeatureSelectionFilter& operator=(const FeatureSelectionFilter &rhs);
    
    /**
     Sets the PreProcessing clone function, overwriting the base PreProcessing function.
     This function is used to clone the values from the input pointer to this instance of the PreProcessing module.
     This function is called by the GestureRecognitionPipeline when the user adds a new PreProcessing module to the pipeline.
     
	 @param const PreProcessing *preProcessing: a pointer to another instance of a DoubleMovingAverageFilter, the values of that instance will be cloned to this instance
	 @return true if the clone was successful, false otherwise
     */
    virtual bool clone(const PreProcessing *preProcessing);
        
    /**
     Sets the PreProcessing process function, overwriting the base PreProcessing function.
     This function is called by the GestureRecognitionPipeline when any new input data needs to be processed (during the prediction phase for example).
     This function calls the DoubleMovingAverageFilter's filter function.
     
	 @param const vector< double > &inputVector: the inputVector that should be processed.  Must have the same dimensionality as the PreProcessing module
	 @return true if the data was processed, false otherwise
     */
    virtual bool process(const vector< double > &inputVector);
    
    /**
     Sets the PreProcessing reset function, overwriting the base PreProcessing function.
     This function is called by the GestureRecognitionPipeline when the pipelines main reset() function is called.
     This function resets the filter values by re-initiliazing the filter.
     
	 @return true if the filter was reset, false otherwise
     */
    virtual bool reset();
    
    /**
     This saves the current settings of the DoubleMovingAverageFilter to a file.
     This overrides the saveSettingsToFile function in the PreProcessing base class.
     
     @param string filename: the name of the file to save the settings to
     @return returns true if the model was saved successfully, false otherwise
     */
    virtual bool saveSettingsToFile(string filename);
    
    /**
     This saves the current settings of the DoubleMovingAverageFilter to a file.
     This overrides the saveSettingsToFile function in the PreProcessing base class.
     
     @param fstream &file: a reference to the file the settings will be saved to
     @return returns true if the settings were saved successfully, false otherwise
     */
    virtual bool saveSettingsToFile(fstream &file);
    
    /**
     This loads the DoubleMovingAverageFilter settings from a file.
     This overrides the loadSettingsFromFile function in the PreProcessing base class.
     
     @param string filename: the name of the file to load the settings from
     @return returns true if the settings were loaded successfully, false otherwise
     */
    virtual bool loadSettingsFromFile(string filename);
    
    /**
     This loads the DoubleMovingAverageFilter settings from a file.
     This overrides the loadSettingsFromFile function in the PreProcessing base class.
     
     @param fstream &file: a reference to the file to load the settings from
     @return returns true if the model was loaded successfully, false otherwise
     */
    virtual bool loadSettingsFromFile(fstream &file);
    
    /**
     Filters the input, the dimensionality of the input vector should match that of the filter.
     
     @param const vector< double > &x: the values to filter, the dimensionality of the input vector should match that of the filter
	 @return the filtered values.  An empty vector will be returned if the values were not filtered
     */
    vector< double > filter(const vector< double > &x);
    
    /**
     Returns the last value(s) that were filtered.
     
	 @return the filtered values.  An empty vector will be returned if the values were not filtered
     */
    vector< double > getFilteredData(){ return processedData; }    

	static RegisterPreProcessingModule< FeatureSelectionFilter > registerModule;
protected:
	/**
	 * Feature selection
	 */
	boost::dynamic_bitset<> m_feature_selection;
};

}//End of namespace GRT
