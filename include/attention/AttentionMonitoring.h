/*
 * This file is part of the AV-NUI project.
 * Copyright (C) 2012 DFKI GmbH. All rights reserved.
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
#include "eventmanager/SimpleEventListener.hpp"

#include <opencv2/core/core.hpp>
#include <iostream>
#include <stdint.h>
#include <boost/filesystem.hpp>
#include <queue>
#include "attention/SpatialAttentionModel.h"

/**
 * \brief Eye Tracking
 */
class EVENTMANAGER_EXPORT AttentionMonitoring : public ISimpleEventListener {
public:
    AttentionMonitoring() :
		ISimpleEventListener(EYE_TRACKING_MODULE) {
		reconfigure();
        nui::config()->registerForReconfigure("AttentionMonitoring", boost::bind(&AttentionMonitoring::reconfigure, this));
	}
	;
	void reconfigure();
protected:

	virtual void process(nui::events::UserAttentionEvent&);
    virtual void process(nui::events::HeadTrackingEvent&);

	/** \brief Pointer to thread. */
    boost::thread* m_process_thread;

	/** \brief Mutex for locks  */
    boost::mutex m_event_queue_mutex;

    std::queue<nui::events::HeadTrackingEvent> m_tracking_event_queue;

    DGroegerCV::SpatialAttentionModel *m_attention_model;

    /**
     * \brief processes the video frames.
     */
    void processEvent();

	virtual NUISTATUS initialize();

	virtual NUISTATUS unInitialize();


private:

};
