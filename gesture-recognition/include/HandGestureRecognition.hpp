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
#include "eventmanager/Events.hpp"
#include "eventmanager/IProcessingModule.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <queue>
#include <stdint.h>


/**
 * \brief Hand Gestures
 *
 */
class GESTURERECOGNITION_EXPORT HandGestureRecognition : public IProcessingModule {
public:
	HandGestureRecognition() :
		IProcessingModule(HANDGESTURE_RECOGNITON_MODULE) {
		reconfigure();
		nui::config()->registerForReconfigure(HANDGESTURE_RECOGNITON_MODULE,
				boost::bind(&HandGestureRecognition::reconfigure, this));
	};
	void reconfigure();
protected:
    virtual void process(nui::events::PMDNanoEvent&);

	virtual NUISTATUS initialize();

	virtual NUISTATUS unInitialize();
private:

	pcl::VoxelGrid<pcl::PointXYZ> m_vg;
	/**
	 * Flag if streams should be streamed.
	 */
	bool m_stream_gestures;
};

