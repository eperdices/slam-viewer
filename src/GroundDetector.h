/**
 *
 *  Copyright (C) 2018 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef SLAM_VIEWER_GROUNDDETECTOR_H
#define SLAM_VIEWER_GROUNDDETECTOR_H

#include <vector>
#include <mutex>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "Map.h"
#include "MapPoint.h"

namespace SLAM_VIEWER {

class GroundDetector {
 public:
    GroundDetector(Map * map);

    // Get calculated plane
    cv::Mat GetPlane();

 private:
    // Detect main plane from list of 3D Point
    void DetectPlane(const std::vector<MapPoint*>& points, int iterations);

    // Compute plane with selected points
    void ComputePlane(const std::vector<MapPoint *>& points);

    Map * map_;
    cv::Mat pwt_;   // Transformation from world to plane

    std::mutex mutexPlane_;


 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace SLAM_VIEWER

#endif  // SLAM_VIEWER_GROUNDDETECTOR_H
