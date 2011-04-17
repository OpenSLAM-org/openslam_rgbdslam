#include "globaldefinitions.h"
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

//######################## GLOBAL CONFIGURATION #############################
//This file serves as a central place for configuration. This should be 
//replaced by a configuration facility using the parameter server or config files

///In main.cpp
///Kinect topics to listen to
const char* global_topic_image_mono =  "/camera/rgb/image_mono";
const char* global_topic_image_depth = "/camera/depth/image";
const char* global_topic_points =      "/camera/rgb/points";

///Use these keypoints/features
const char* global_feature_detector_type =  "FAST";
const char* global_feature_extractor_type = "SURF";
///Identify like this in the ros communication network
const char* global_rosnode_name = "rgbdslam"; const char* global_ros_namespace = "rgbdslam";

///In openNIListener.cpp
const bool global_start_paused = true;
const int global_subscriber_queue_size = 20;
const int global_publisher_queue_size = 1;
const char* global_topic_reframed_cloud =    "reframed_cloud";
const char* global_topic_transformed_cloud = "transformed_cloud";
const char* global_topic_first_cloud =       "reference_cloud";
const char* global_topic_sampled_cloud =     "sampled_cloud";
///This influences speed and quality dramatically
const int global_adjuster_max_keypoints = 1800;
const int global_adjuster_min_keypoints = 1000;
const int global_fast_adjuster_max_iterations = 10;
const int global_surf_adjuster_max_iterations = 5; //may slow down initially
///Ignorance w.r.t small motion (reduces redundancy)
const float global_min_translation_meter = 0.1;
const float global_min_rotation_degree = 5; 
///No output on "timings" logger for less than this time in seconds
const float global_min_time_reported = 0.01;

///In glviewer.cpp
///Do not connect points with a distance over this, relative to distance to the camera
///(in meters squared, i.e. 0.0004 means 2cm)
const float global_squared_meshing_threshold = 0.0009; 

///Disabling the 3d display will speed-up the capturing. 
///Memory requirements seem unaffected
const bool global_use_glwidget = true;
///Maximally this many comparisons per node
///(lower=faster, higher=better loop closing)
const unsigned int global_connectivity = 10;
