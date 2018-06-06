/**
 *
 *    Copyright (C) 2018 Eduardo Perdices <eperdices at gsyc dot es>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *    GNU Library General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <iostream>
#include "GroundDetector.h"
#include "extra/utils.h"

using std::vector;
using std::mutex;
using std::unique_lock;

namespace SLAM_VIEWER {

GroundDetector::GroundDetector(Map * map) {
    map_ = map;

    {
        unique_lock<mutex> lock(mutexPlane_);
        pwt_ = cv::Mat::eye(4,4,CV_32F);
    }

    // Detect plane
    vector<MapPoint*> points = map_->GetAllMapPoints();
    DetectPlane(points ,50);
}

cv::Mat GroundDetector::GetPlane() {
    unique_lock<mutex> lock(mutexPlane_);
    return pwt_;
}

void GroundDetector::DetectPlane(const vector<MapPoint*> &points, int iterations) {
    // Retrieve 3D points
    vector<Eigen::Vector3d> vPoints;
    vPoints.reserve(points.size());

    for(size_t i=0; i<points.size(); i++)
        vPoints.push_back(points[i]->GetWorldPos());

    const int N = vPoints.size();

    if (N < 50) {
        std::cout << "Couldn't calculate ground, not enough 3D points" << std::endl;
        return;
    }

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
        vAllIndices.push_back(i);

    float bestDist = 1e10;
    vector<float> bestvDist;

    // RANSAC
    for(int n=0; n<iterations; n++) {
        vAvailableIndices = vAllIndices;

        cv::Mat A(3,4,CV_32F);
        A.col(3) = cv::Mat::ones(3,1,CV_32F);

        // Get min set of points
        for(short i = 0; i < 3; ++i) {
            int randi = Random(0, vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            A.at<float>(i, 0) = vPoints[idx](0);
            A.at<float>(i, 1) = vPoints[idx](1);
            A.at<float>(i, 2) = vPoints[idx](2);

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3,0);
        const float b = vt.at<float>(3,1);
        const float c = vt.at<float>(3,2);
        const float d = vt.at<float>(3,3);

        vector<float> vDistances(N,0);

        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

        for(int i=0; i<N; i++)
            vDistances[i] = fabs(vPoints[i](0)*a+vPoints[i](1)*b+vPoints[i](2)*c+d)*f;

        vector<float> vSorted = vDistances;
        sort(vSorted.begin(),vSorted.end());

        int nth = std::max((int)(0.2*N),20);
        const float medianDist = vSorted[nth];

        if(medianDist < bestDist) {
            bestDist = medianDist;
            bestvDist = vDistances;
        }
    }

    // Compute threshold inlier/outlier
    const float th = 1.4*bestDist;
    vector<bool> vbInliers(N,false);
    int nInliers = 0;
    for(int i=0; i<N; i++) {
        if(bestvDist[i] < th) {
            nInliers++;
            vbInliers[i] = true;
        }
    }

    vector<MapPoint*> vInlierMPs(nInliers, NULL);
    int nin = 0;
    for(int i=0; i<N; i++) {
        if(vbInliers[i]) {
            vInlierMPs[nin] = points[i];
            nin++;
        }
    }

    // Compute plane with selected inliers
    ComputePlane(vInlierMPs);
}

void GroundDetector::ComputePlane(const std::vector<MapPoint *>& points) {
    const int N = points.size();
    const int maxN = 500;
    int step;

    std::cout << "[DEBUG] Calculating Main Plane..." << std::endl;

    // Recompute plane with all points
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);

    Eigen::Vector3d origin;
    origin.setZero();

    if (N > maxN)
        step = N/maxN;
    else
        step = 1;

    int nPoints = 0;
    for(int i=0; i<N; i+=step) {
        Eigen::Vector3d Xw = points[i]->GetWorldPos();
        origin += Xw;

        A.at<float>(nPoints, 0) = Xw(0);
        A.at<float>(nPoints, 1) = Xw(1);
        A.at<float>(nPoints, 2) = Xw(2);

        nPoints++;
    }
    A.resize(nPoints);

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a = vt.at<float>(3,0);
    float b = vt.at<float>(3,1);
    float c = vt.at<float>(3,2);

    origin = origin*(1.0f/nPoints);
    const float f = 1.0f/sqrt(a*a+b*b+c*c);

    const float nx = a*f;
    const float ny = b*f;
    const float nz = c*f;

    cv::Mat normal = (cv::Mat_<float>(3,1)<<nx,ny,nz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(normal);
    const float sa = cv::norm(normal);
    const float ca = up.dot(normal);
    const float ang = atan2(sa,ca);
    float rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f; // Arbitrary orientation

    {
        unique_lock<mutex> lock(mutexPlane_);
        pwt_.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
        pwt_.at<float>(0,3) = origin(0);
        pwt_.at<float>(1,3) = origin(1);
        pwt_.at<float>(2,3) = origin(2);
    }

    std::cout << "[DEBUG] DONE" << std::endl;
}



}    // namespace SLAM_VIEWER
