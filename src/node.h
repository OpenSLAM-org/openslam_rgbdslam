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


#ifndef RGBD_SLAM_NODE_H_
#define RGBD_SLAM_NODE_H_


#include "ros/ros.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include "globaldefinitions.h"

// ICP_1 for external binary
//#define USE_ICP_BIN

// ICP_2 for included function
//#define USE_ICP_CODE

//#define USE_SIFT_GPU


#ifdef USE_ICP_BIN
#include "gicp-fallback.h"
#endif

#ifdef USE_ICP_CODE
#include "../gicp/gicp.h"
#include "../gicp/transform.h"
#endif

#include "matching_result.h" 
#include <Eigen/StdVector>

// Search structure for descriptormatching
typedef cv::flann::Index cv_flannIndex;

//!Holds the data for one graph node and provides functionality to compute relative transformations to other Nodes.
class Node {
public:
	///Visual must be CV_8UC1, depth CV_32FC1, 
	///id must correspond to the hogman vertex id
	///detection_mask must be CV_8UC1 with non-zero 
	///at potential keypoint locations
	Node(ros::NodeHandle& nh, const cv::Mat& visual,
			cv::Ptr<cv::FeatureDetector> detector,
			cv::Ptr<cv::DescriptorExtractor> extractor,
			cv::Ptr<cv::DescriptorMatcher> matcher, // deprecated!
			const sensor_msgs::PointCloud2ConstPtr point_cloud,
			const cv::Mat& detection_mask = cv::Mat());
	//default constructor. TODO: still needed?
	Node(){}
	///Delete the flannIndex if built
	~Node();


	///Compare the features of two nodes and compute the transformation
	MatchingResult matchNodePair(const Node* older_node);

	///Compute the relative transformation between the nodes
	///Do either max_ransac_iterations or half of it, 
	///Iterations with more than half of the initial_matches 
	///inlying, count twice. Iterations with more than 80% of 
	///the initial_matches inlying, count threefold
	bool getRelativeTransformationTo(const Node* target_node, 
			std::vector<cv::DMatch>* initial_matches,
			Eigen::Matrix4f& resulting_transformation, 
			float& rmse,
			std::vector<cv::DMatch>& matches,//for visualization?
					unsigned int max_ransac_iterations = 10000) const;

#ifdef USE_ICP_BIN
	// initial_transformation: optional transformation applied to this->pc before
	// using icp
	bool getRelativeTransformationTo_ICP_bin(const Node* target_node,Eigen::Matrix4f& transformation,
			const Eigen::Matrix4f* initial_transformation = NULL);
#endif
	
#ifdef USE_ICP_CODE
	bool getRelativeTransformationTo_ICP_code(const Node* target_node,Eigen::Matrix4f& transformation,
			const Eigen::Matrix4f* initial_transformation = NULL);
#endif


	///Send own pointcloud on given topic with given timestamp
	void publish(const char* frame, ros::Time timestamp);

	// void publish();
	// void moveAndPublish(const Eigen::Matrix4f& trafo);
	// void moveAndPublishRansac(const Eigen::Matrix4f& trafo);

	void buildFlannIndex();
	int findPairsFlann(const Node* other, vector<cv::DMatch>* matches) const;

#ifdef USE_ICP_CODE

	dgc::gicp::GICPPointSet* gicp_point_set;
	
	static const double gicp_epsilon = 1e-4;
	static const double gicp_d_max = 0.10; // 10cm
	static const unsigned int gicp_max_iterations = 200;
	static const unsigned int gicp_point_cnt = 20000;
		
	bool gicp_initialized;
	
	void Eigen2GICP(const Eigen::Matrix4f& m, dgc_transform_t g_m);
	void GICP2Eigen(const dgc_transform_t g_m, Eigen::Matrix4f& m);
	void gicpSetIdentity(dgc_transform_t m);
	void createGICPStructures(unsigned int max_count = 1000);

#endif



	//PointCloud pc;
	///pointcloud_type centrally defines what the pc is templated on
	pointcloud_type pc_col;

	cv::Mat feature_descriptors_;         ///<descriptor definitions
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > feature_locations_3d_;  ///<backprojected 3d descriptor locations relative to cam position in homogeneous coordinates (last dimension is 1.0)
	std::vector<cv::KeyPoint> feature_locations_2d_; ///<Where in the image are the descriptors
	unsigned int id_; ///must correspond to the hogman vertex id

protected:


	ros::Publisher cloud_pub_;
	// ros::Publisher cloud_pub_ransac;

	//void removeNANsFromPointCloud(PointCloud& pcloud);
	// void removeNANsFromPointCloud(PointCloud& pcloud, pointcloud_type& pcloud_rgb);

	cv_flannIndex* flannIndex;
	image_geometry::PinholeCameraModel cam_model_;  
	cv::Ptr<cv::DescriptorMatcher> matcher_;

	/** remove invalid keypoints (NaN or outside the image) and return the backprojection of valid ones*/
#ifdef USE_SIFT_GPU
	//remove also unused descriptors
	void projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
					std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
					const pointcloud_type& point_cloud, float* descriptors_in, cv::Mat& descriptors_out);
#else
	void projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
			std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
			const pointcloud_type& point_cloud);
#endif

	/*
    ///Compare the features of two nodes and compute the transformation
    void processNodePair(const Node* older_node, 
                         std::vector<cv::DMatch>& inliers,std::vector<cv::DMatch>& outliers,
                         AISNavigation::LoadedEdge3D& edge_out,
                         Eigen::Matrix4f& ransac_transform, Eigen::Matrix4f& final_trafo) const;
	 */
	// helper for ransac
	// check for distances only if max_dist_cm > 0
	template<class InputIterator>
	Eigen::Matrix4f getTransformFromMatches(const Node* other_node, 
			InputIterator iter_begin,
			InputIterator iter_end,
			bool* valid = NULL, 
			float max_dist_m = -1
	) const;
	//std::vector<cv::DMatch> const* matches,
	//pcl::TransformationFromCorrespondences& tfc);


	///Get the norm of the translational part of an affine matrix (Helper for isBigTrafo)
	void mat2dist(const Eigen::Matrix4f& t, double &dist){
		dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
	}
	///Get euler angles from affine matrix (helper for isBigTrafo)
	void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw) ;

	void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist);

	// helper for ransac
	void computeInliersAndError(const std::vector<cv::DMatch>& initial_matches,
			const Eigen::Matrix4f& transformation,
			const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
			const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& targets,
			std::vector<cv::DMatch>& new_inliers, //output var
			double& mean_error, vector<double>& errors,
			double squaredMaxInlierDistInM = 0.0009) const; //output var;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
