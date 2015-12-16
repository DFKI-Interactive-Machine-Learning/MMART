/*
 * InputImage.cpp
 *
 *  Created on: Apr 10, 2014
 *      Author: Daniel Gröger
 */

#include "attention/SpatialAttentionModel.h"

namespace DGroegerCV {


SpatialAttentionModel::SpatialAttentionModel() {
    this->areas_ = new std::vector<TargetArea*>();
}

SpatialAttentionModel::~SpatialAttentionModel() {
    delete this->areas_;
}

bool compareAreaHit (const AreaHit* i, const AreaHit* j) { return (i->distance()>j->distance()); }

std::vector<AreaHit*> SpatialAttentionModel::GetTargetsForRay(const cv::Vec3d& ray_origin, const cv::Vec3d& ray_direction) {
    std::vector<AreaHit*> targets;
    double total_distance = 0.0;
    for(std::vector<TargetArea*>::iterator iter = this->areas_->begin(); iter != this->areas_->end(); ++iter) {
        TargetArea* area = *iter;
        if(!area->is_active()) {
            continue;
        }
        cv::Vec3d intersection_point;
        double score = area->GetScore(ray_origin, ray_direction, intersection_point);
        if(score > -1) {
            targets.push_back(new AreaHit(score, area, intersection_point));
            total_distance += score;
        }
    }

    if(targets.size()!=0) {

        std::sort (targets.begin(), targets.end(), compareAreaHit);

        int i=1;
        int s = targets.size();
        //TODO calculate percentage on multi hit
        for(std::vector<AreaHit*>::iterator iter = targets.begin(); iter != targets.end(); ++iter) {
            AreaHit* hit = *iter;
            hit->set_percent(targets.at(s-i)->distance()*100.0/total_distance);//this wont work ;)
            i++;
        }


    }

    return targets;
}

void SpatialAttentionModel::AddTargetArea(TargetArea* area) {
    this->areas_->push_back(area);
}

std::vector<TargetArea*> SpatialAttentionModel::areas() {
    std::vector<TargetArea*> areas;
    for(std::vector<TargetArea*>::iterator iter = this->areas_->begin(); iter != this->areas_->end(); ++iter) {
        TargetArea* area = *iter;
        areas.push_back(area);
    }
    return areas;
}

}
