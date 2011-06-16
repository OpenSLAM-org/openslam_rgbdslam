#include "parameter_server.h"
#include <ros/ros.h>
#include <string>

using namespace std;

ParameterServer* ParameterServer::_instance = NULL;

ParameterServer::ParameterServer() {
	pre = ros::this_node::getName();
	pre += "/config/";

	parseConfig();
	getValues();
}

ParameterServer* ParameterServer::instance() {
	if (_instance == NULL) {
		_instance = new ParameterServer();
	}
	return _instance;
}
void ParameterServer::parseConfig() {
  config["topic_image_mono"]             =  std::string("/camera/rgb/image_mono");
  config["topic_image_depth"]            =  std::string("/camera/depth/image");
  config["topic_points"]                 =  std::string("/camera/rgb/points");
  config["topic_reframed_cloud"]         =  std::string("reframed_cloud");
  config["topic_transformed_cloud"]      =  std::string("transformed_cloud");
  config["topic_first_cloud"]            =  std::string("reference_cloud");
  config["topic_sampled_cloud"]          =  std::string("sampled_cloud");
  config["feature_detector_type"]        =  std::string("SURF");
  config["feature_extractor_type"]       =  std::string("SURF");
  config["start_paused"]                 =  static_cast<bool>  (1);
  config["subscriber_queue_size"]        =  static_cast<int>   (20);
  config["publisher_queue_size"]         =  static_cast<int>   (1);
  config["adjuster_max_keypoints"]       =  static_cast<int>   (1800);
  config["adjuster_min_keypoints"]       =  static_cast<int>   (1000);
  config["min_matches"]                  =  static_cast<int>   (100);
  config["fast_adjuster_max_iterations"] =  static_cast<int>   (10);
  config["surf_adjuster_max_iterations"] =  static_cast<int>   (5);
  config["min_translation_meter"]        =  static_cast<double>(0.1);
  config["min_rotation_degree"]          =  static_cast<int>   (5);
  config["min_time_reported"]            =  static_cast<double>(0.01);
  config["squared_meshing_threshold"]    =  static_cast<double>(0.0009);
  config["use_glwidget"]                 =  static_cast<bool>  (1);
  config["preserve_raster_on_save"]      =  static_cast<bool>  (0);
  config["connectivity"]                 =  static_cast<int>   (10);
  config["max_dist_for_inliers"]         =  static_cast<double>(0.03);
  config["drop_async_frames"]            =  static_cast<bool>  (1);
}

void ParameterServer::getValues() {
	map<string, boost::any>::const_iterator itr;
	for (itr = config.begin(); itr != config.end(); ++itr) {
		string name = itr->first;
		if (itr->second.type() == typeid(string)) {
			config[name] = getFromParameterServer<string> (pre + name,
					boost::any_cast<string>(itr->second));
			ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<string>(itr->second));
		} else if (itr->second.type() == typeid(int)) {
			config[name] = getFromParameterServer<int> (pre + name,
					boost::any_cast<int>(itr->second));
			ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<int>(itr->second));
		}
		else if (itr->second.type() == typeid(double)) {
			config[name] = getFromParameterServer<double> (pre + name,
					boost::any_cast<double>(itr->second));
			ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<double>(itr->second));
		} else if (itr->second.type() == typeid(bool)) {
			config[name] = getFromParameterServer<bool> (pre + name,
					boost::any_cast<bool>(itr->second));
			ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<bool>(itr->second));
		}
	}
}
