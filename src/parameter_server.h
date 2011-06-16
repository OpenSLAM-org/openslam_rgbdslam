#ifndef PARAMETER_SERVER_H_
#define PARAMETER_SERVER_H_
#include <string>
#include <ros/ros.h>
#include <boost/any.hpp>

//this is a global definition of the points to be used
//changes to omit color would need adaptations in 
//the visualization too
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;
#define CONCURRENT_EDGE_COMPUTATION

class ParameterServer {
public:
	static ParameterServer* instance();

	template<typename T>
	T get(const std::string param) {
		boost::any value = config[param];
		return boost::any_cast<T>(value);
	}

private:
	std::map<std::string, boost::any> config;

	static ParameterServer* _instance;
	std::string pre;
	ros::NodeHandle handle;

	ParameterServer();

	void getValues();
	void parseConfig();

	/*
	 * Returns a value from the parameter server
	 */
	template<typename T>
	T getFromParameterServer(const std::string param, T def) {
		std::string fullParam = param;
		T result;
		handle.param(fullParam, result, def);
		return result;
	}
};

#endif /* PARAMETER_SERVER_H_ */
