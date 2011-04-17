/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef MATCHING_RESULT_H
#define MATCHING_RESULT_H
#include <opencv2/features2d/features2d.hpp>
#include <hogman_minimal/graph/loadEdges3d.h>
#include <Eigen/Core>

class MatchingResult {
    public:
        MatchingResult() : rmse(0.0) {
            edge.id1 = edge.id2 = -1;
        }
        std::vector<cv::DMatch> inlier_matches;
        std::vector<cv::DMatch> all_matches;
        AISNavigation::LoadedEdge3D edge;
        float rmse;
        Eigen::Matrix4f ransac_trafo;
        Eigen::Matrix4f icp_trafo;
        Eigen::Matrix4f final_trafo;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
        
#endif
