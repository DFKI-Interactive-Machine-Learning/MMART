/*
 * AreaHit.h
 *
 *  Created on: Jun 24, 2014
 *      Author: Daniel Gröger
 */

#ifndef AREAHIT_H_
#define AREAHIT_H_

#include <opencv/cv.h>
#include "TargetArea.h"

namespace DGroegerCV {

/**
 * Simple class representing a planar area in 3D space.
 */
class AreaHit {

private:

    double distance_;
    double percent_;
    TargetArea *area_;
    cv::Vec3d intersection_;

public:
    AreaHit(double distance, TargetArea *area, const cv::Vec3d& intersection);
	virtual ~AreaHit();

    double percent() const;
    void set_percent(double percent);
    double distance() const;
    TargetArea *area() const;
    cv::Vec3d intersection() const;
};
}




#endif /* AREAHIT_H_ */
