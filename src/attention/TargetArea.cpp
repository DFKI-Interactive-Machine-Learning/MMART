/*
 * InputImage.cpp
 *
 *  Created on: Apr 10, 2014
 *      Author: Daniel Gröger
 */

#include "attention/TargetArea.h"

namespace DGroegerCV {



TargetArea::TargetArea(const std::string& label, const cv::Vec3d& point, const cv::Vec3d& vector_top, const cv::Vec3d& vector_right, bool is_active) {
    label_ = label;
    point_ = point;
    vector_top_ = vector_top;
    vector_right_ = vector_right;
    is_active_ = is_active;

    plane_normal_ = vector_top_.cross(vector_right_);
}

TargetArea::~TargetArea() {
}

bool TargetArea::Intersect(const cv::Vec3d& ray_origin, const cv::Vec3d& ray_direction, cv::Vec3d& return_intersection_point) {
    double ZERO_TOLERANCE = 1e-08;


    //distance plane point
    double signed_distance = (point_ - ray_origin).dot(plane_normal_);
    double DdN = ray_direction.dot(plane_normal_);

    if(std::fabs(DdN) < ZERO_TOLERANCE || std::fabs(signed_distance) < ZERO_TOLERANCE) {
        return false;
    }

    double d = signed_distance/DdN;
    return_intersection_point = ray_origin + d * ray_direction;

    return true;
}

double TargetArea::GetScore(const cv::Vec3d& ray_origin, const cv::Vec3d& ray_direction, cv::Vec3d& return_intersection_point) {

    if(!this->Intersect(ray_origin, ray_direction, return_intersection_point)) {
        return -1.0;
    }

    cv::Vec3d intersection_in_local_coords = return_intersection_point-point_;

    double length_top = cv::norm(vector_top_);
    double length_right = cv::norm(vector_right_);


    cv::Vec3d unit_top = vector_top_ * (1/length_top);
    cv::Vec3d unit_right = vector_right_ * (1/length_right);

    double top = unit_top.dot(intersection_in_local_coords);
    if(top <0 || top > cv::norm(vector_top_)) {
        return -1.0;
    }
    double right = unit_right.dot(intersection_in_local_coords);
    if(right <0 || right > cv::norm(vector_right_)) {
        return -1.0;
    }

    //find distance to closest edge
    double min_top = top;
    if(top>length_top*0.5) {
        min_top = length_top - top;
    }

    double min_right = right;
    if(right>length_right*0.5) {
        min_right = length_right - right;
    }

    if(min_right<min_top) {
        return min_right/(length_right*0.5);
    }
    return min_top/(length_top*0.5);


    //return distance from center
    cv::Vec3d center = point_ + (vector_top_*0.5) + (vector_right_*0.5);

    double distance = cv::norm(return_intersection_point - center);
    if(distance < 1e-08) {
        return 1e-08;
    }
    return distance;
}

std::vector<cv::Vec3d> TargetArea::GetCorners() {
    std::vector<cv::Vec3d> corners;
    corners.push_back(point_);
    corners.push_back(point_+vector_top_);
    corners.push_back(point_+vector_top_+vector_right_);
    corners.push_back(point_+vector_right_);
    return corners;
}


std::string TargetArea::label() const
{
    return label_;
}

bool TargetArea::is_active() const
{
    return is_active_;
}

void TargetArea::set_is_active(bool is_active)
{
    is_active_ = is_active;
}

}
