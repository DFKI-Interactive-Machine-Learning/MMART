/*
 * AreaHit.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: Daniel Gröger
 */

#include "attention/AreaHit.h"

namespace DGroegerCV {


AreaHit::AreaHit(double distance, TargetArea *area, const cv::Vec3d& intersection) {
    this->distance_ = distance;
    this->area_ = area;
    this->intersection_ = intersection;
}

AreaHit::~AreaHit() {
}


double AreaHit::percent() const
{
    return percent_;
}

double AreaHit::distance() const
{
    return distance_;
}

TargetArea *AreaHit::area() const
{
    return area_;
}

void AreaHit::set_percent(double percent)
{
    percent_ = percent;
}

cv::Vec3d AreaHit::intersection() const
{
    return intersection_;
}

}

