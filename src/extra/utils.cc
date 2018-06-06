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

#include "utils.h"

namespace SLAM_VIEWER {

int Random(int min, int max) {
  int d = max - min + 1;
  return static_cast<int>(((static_cast<double>(rand())/(static_cast<double>(RAND_MAX) + 1.0)) * d) + min);
}

cv::Mat ExpSO3(const float &x, const float &y, const float &z) {
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                                         z, 0, -x,
                                         -y,    x, 0);
    if(d<1e-4)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

cv::Mat ExpSO3(const cv::Mat &v) {
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

void DO() {
    
}

}  // namespace SLAM_VIEWER
