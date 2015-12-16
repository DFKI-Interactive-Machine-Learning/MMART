/*
 * InputImage.h
 *
 *  Created on: Apr 10, 2014
 *      Author: Daniel Gröger
 */

#ifndef TARGETAREA_H_
#define TARGETAREA_H_

#include <opencv/cv.h>

namespace DGroegerCV {

/**
 * Simple class representing a planar area in 3D space.
 */
class TargetArea {

private:

    /** label for this area */
    std::string label_;

    /** The point defining the area (bottom left corner when looking at the plane)*/
    cv::Vec3d point_;

    /** vector pointing from mPoint to top left corner */
    cv::Vec3d vector_top_;

    /** vector pointing from mPoint to bottom right corner */
    cv::Vec3d vector_right_;

    /** the normal of the plane (is calculated in constructor */
    cv::Vec3d plane_normal_;

    /** whether this target area is active */
    bool is_active_;

public:
    TargetArea(const std::string& label, const cv::Vec3d& point, const cv::Vec3d& vector_top, const cv::Vec3d& vector_right, bool is_active = true);
	virtual ~TargetArea();

    double GetScore(const cv::Vec3d& ray_origin, const cv::Vec3d& ray_direction, cv::Vec3d& return_intersection_point);
    bool Intersect(const cv::Vec3d& ray_origin, const cv::Vec3d& ray_direction, cv::Vec3d& return_intersection_point);
    std::vector<cv::Vec3d> GetCorners();
    std::string label() const;
    bool is_active() const;
    void set_is_active(bool is_active);
};
}




#endif /* TARGETAREA_H_ */
