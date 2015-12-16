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
#include "core/common.hpp"
#include "core/NUIDLLexport.hpp"

#include "eventmanager/Events.hpp"
#include "eventmanager/IProcessingModule.hpp"

#include "GRT/GRT.h"

/**
* \brief Eye Tracking.
*
*/
class GESTURERECOGNITION_EXPORT ModelClassifier : public IProcessingModule {
public:
	ModelClassifier() :
		IProcessingModule(MODELCLASSIFIER_MODULE) {
		reconfigure();
		nui::config()->registerForReconfigure(MODELCLASSIFIER_MODULE,
			boost::bind(&ModelClassifier::reconfigure, this));
	};
	void reconfigure();
protected:
	virtual void process(nui::events::LeapTrackingEvent&);
	virtual void process(nui::events::MyoEvent&);

	virtual NUISTATUS initialize();

	virtual NUISTATUS unInitialize();
private:

};

