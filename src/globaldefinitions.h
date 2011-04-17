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


#ifndef GLOBALDEFINITIONS_H
#define GLOBALDEFINITIONS_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

//Determines whether or not to process node pairs concurrently
#define CONCURRENT_EDGE_COMPUTATION 1


///This file contains the parameters that determine the
///behaviour of the program
typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;
//typedef boost::shared_ptr< ::sensor_msgs::PointCloud2_<ContainerAllocator>  const> ConstPtr;
//typedef boost::shared_ptr<pointcloud_type const> pointcloud_const_ptr;

///No output on "timings" logger for less than this time in seconds
extern const float global_min_time_reported;

///In glviewer.cpp
extern const float global_squared_meshing_threshold; ///< Do not connect points with a depth jump over this (in meters squared, i.e. 0.0004 means 2cm)
///In main.cpp
///Kinect topics to listen to
extern const char* global_topic_image_mono;
extern const char* global_topic_image_depth;
extern const char* global_topic_points;
///Use these keypoints/features
extern const char* global_feature_detector_type;//Fast is really fast but the Keypoints are not robust
extern const char* global_feature_extractor_type;
///Identify like this in the ros communication network
extern const char* global_rosnode_name;
extern const char* global_ros_namespace;

///In openNIListener.cpp
extern const bool global_start_paused;
extern const int global_subscriber_queue_size;
extern const int global_publisher_queue_size;
extern const char* global_topic_reframed_cloud;
extern const char* global_topic_transformed_cloud;
extern const char* global_topic_first_cloud;
extern const char* global_topic_sampled_cloud;

///This influences speed dramatically
extern const int global_adjuster_max_keypoints;
extern const int global_adjuster_min_keypoints;
extern const int global_fast_adjuster_max_iterations;
extern const int global_surf_adjuster_max_iterations;

///Ignorance w.r.t small motion
extern const float global_min_translation_meter;
extern const float global_min_rotation_degree; 

extern const bool global_use_glwidget;
extern const unsigned int global_connectivity;
#endif
