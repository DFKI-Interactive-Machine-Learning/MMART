/*
 * InputImage.h
 *
 *  Created on: Apr 10, 2014
 *      Author: Daniel Gröger
 */

#ifndef SPATIALATTENTIONMODEL_H_
#define SPATIALATTENTIONMODEL_H_

#include <opencv/cv.h>
#include "TargetArea.h"
#include "AreaHit.h"

namespace DGroegerCV {

/**
 * Simple class to store an image along with GT data for eye location and head pose
 */
class SpatialAttentionModel {

private:

    /** all possible attention targets */
    std::vector<TargetArea*> *areas_;

public:
    SpatialAttentionModel();
    virtual ~SpatialAttentionModel();

    std::vector<AreaHit*> GetTargetsForRay(const cv::Vec3d& ray_origin, const cv::Vec3d& ray_direction);

    void AddTargetArea(TargetArea* area);
    std::vector<TargetArea*> areas();

};
}




#endif /* SPATIALATTENTIONMODEL_H_ */
