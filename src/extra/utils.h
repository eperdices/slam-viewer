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

#ifndef SLAM_VIEWER_UTILS_H_
#define SLAM_VIEWER_UTILS_H_

#include <cstdlib>
#include <opencv2/core/core.hpp>

namespace SLAM_VIEWER {

// Get a random int in range [min..max]
int Random(int min, int max);

// Exponential SO3
cv::Mat ExpSO3(const float &x, const float &y, const float &z);
cv::Mat ExpSO3(const cv::Mat &v);

}  // namespace SLAM_VIEWER

#endif  // SLAM_VIEWER_UTILS_H_
