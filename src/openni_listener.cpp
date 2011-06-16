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


//Documentation see header file
#include "pcl/ros/conversions.h"
#include <pcl/io/io.h>
#include "pcl/common/transform.h"
#include "pcl_ros/transforms.h"
#include "openni_listener.h"
#include <cv_bridge/CvBridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <cv.h>
#include <ctime>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
//#include <QMessageBox>
#include "node.h"

#include "parameter_server.h"


OpenNIListener::OpenNIListener(ros::NodeHandle nh, GraphManager* graph_mgr,
                               const char* visual_topic, 
                               const char* depth_topic, const char* cloud_topic, 
                               const char* detector_type, const char* extractor_type) 
: graph_mgr_(graph_mgr),
  visual_sub_ (nh, visual_topic, ParameterServer::instance()->get<int>("subscriber_queue_size")),
  depth_sub_(nh, depth_topic, ParameterServer::instance()->get<int>("subscriber_queue_size")),
  cloud_sub_(nh, cloud_topic, ParameterServer::instance()->get<int>("subscriber_queue_size")),
  sync_(MySyncPolicy(ParameterServer::instance()->get<int>("subscriber_queue_size")),  visual_sub_, depth_sub_, cloud_sub_),
  depth_mono8_img_(cv::Mat()),
  nh_(nh),
  /*callback_counter_(0),*/
  save_bag_file(false),
  pause_(ParameterServer::instance()->get<bool>("start_paused")),
  getOneFrame_(false),
  first_frame_(true)
  //pc_pub(nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 2))
{
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  sync_.registerCallback(boost::bind(&OpenNIListener::cameraCallback, this, _1, _2, _3));
  ROS_INFO_STREAM("Listening to " << visual_topic << ", " << depth_topic \
                   << " and " << cloud_topic << "\n"); 
  detector_ = this->createDetector(detector_type);
  ROS_FATAL_COND(detector_.empty(), "No valid opencv keypoint detector!");
  extractor_ = this->createDescriptorExtractor(extractor_type);
  matcher_ = new cv::BruteForceMatcher<cv::L2<float> >() ;

  ParameterServer* params = ParameterServer::instance();
  pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (params->get<std::string>("topic_reframed_cloud"),
		  params->get<int>("publisher_queue_size"));
  pub_transf_cloud_ = nh.advertise<sensor_msgs::PointCloud2> (
		  params->get<std::string>("topic_transformed_cloud"), params->get<int>("publisher_queue_size"));
  pub_ref_cloud_ = nh.advertise<sensor_msgs::PointCloud2>(params->get<std::string>("topic_first_cloud"),
		  params->get<int>("publisher_queue_size"));
  
}



void OpenNIListener::cameraCallback (const sensor_msgs::ImageConstPtr& visual_img_msg, 
                                     const sensor_msgs::ImageConstPtr& depth_img_msg,   
                                     const sensor_msgs::PointCloud2ConstPtr& point_cloud) {
  std::clock_t starttime=std::clock();
  ROS_DEBUG("Received data from kinect");

  //Get images into OpenCV format
	sensor_msgs::CvBridge bridge;
	cv::Mat depth_float_img = bridge.imgMsgToCv(depth_img_msg); 
	cv::Mat visual_img =  bridge.imgMsgToCv(visual_img_msg, "mono8");
  if(visual_img.rows != depth_float_img.rows || 
     visual_img.cols != depth_float_img.cols ||
     point_cloud->width != (uint32_t) visual_img.cols ||
     point_cloud->height != (uint32_t) visual_img.rows){
    ROS_ERROR("PointCloud, depth and visual image differ in size! Ignoring Data");
    return;
  }
  depthToCV8UC1(depth_float_img, depth_mono8_img_); //float can't be visualized or used as mask in float format TODO: reprogram keypoint detector to use float values with nan to mask
   

  Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
  Q_EMIT newDepthImage (cvMat2QImage(depth_mono8_img_,1));//overwrites last cvMat2QImage
  if(getOneFrame_) { //if getOneFrame_ is set, unset it and skip check for  pause
      getOneFrame_ = false;
  } else if(pause_ && !save_bag_file) { //Visualization and nothing else
    return; 
  }

  ros::Time d_time = depth_img_msg->header.stamp;
  ros::Time rgb_time = visual_img_msg->header.stamp;
  ros::Time pc_time = point_cloud->header.stamp;
  long rgb_timediff = abs(static_cast<long>(rgb_time.nsec) - static_cast<long>(pc_time.nsec));
  if(d_time.nsec != pc_time.nsec ||rgb_timediff > 33333333){
      ROS_WARN_COND(d_time.nsec != pc_time.nsec, "PointCloud doesn't correspond to depth image");
      ROS_WARN_COND(rgb_timediff > 33333333, "Point cloud and RGB image off more than 1/30sec: %li (nsec)", rgb_timediff);
      ROS_WARN_COND(ParameterServer::instance()->get<bool>("drop_async_frames"), "Asynchronous frames ignored. See parameters if you want to keep async frames.");
      ROS_INFO("Depth image time: %d - %d", d_time.sec,   d_time.nsec);
      ROS_INFO("RGB   image time: %d - %d", rgb_time.sec, rgb_time.nsec);
      ROS_INFO("Point cloud time: %d - %d", pc_time.sec,  pc_time.nsec);
      getOneFrame_ = true; //more luck next time?
      if(ParameterServer::instance()->get<bool>("drop_async_frames"))return;
  } else {
      ROS_DEBUG("Depth image time: %d - %d", d_time.sec,   d_time.nsec);
      ROS_DEBUG("RGB   image time: %d - %d", rgb_time.sec, rgb_time.nsec);
      ROS_DEBUG("Point cloud time: %d - %d", pc_time.sec,  pc_time.nsec);
  }

	if (bagfile_mutex.tryLock() && save_bag_file){
		// todo: make the names dynamic
		bag.write("/camera/rgb/points", ros::Time::now(), point_cloud);
		bag.write("/camera/rgb/image_mono", ros::Time::now(), visual_img_msg);
		bag.write("/camera/depth/image", ros::Time::now(), depth_img_msg);
    ROS_INFO_STREAM("Wrote to bagfile " << bag.getFileName());
    bagfile_mutex.unlock();
    if(pause_) return; 
	}


  //ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > min_time_reported, "timings", "Callback runtime before addNode: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");

  //######### Main Work: create new node an add it to the graph ###############################
  Q_EMIT setGUIStatus("Computing Keypoints and Features");
  //TODO: make it an reference counting pointer object?
  std::clock_t node_creation_time=std::clock();
  Node* node_ptr = new Node(nh_,visual_img, detector_, extractor_, matcher_,
                            point_cloud, depth_mono8_img_);
  if((std::clock()-node_creation_time)/(double)CLOCKS_PER_SEC > 15.0){
      pause_ = true;
      setGUIInfo("<span style='color:red'>Node creation took more than 15 seconds. Paused processing to prevent CPU overload</span>");
      ROS_WARN("Node creation was really slow (>15s). Processing has been paused to prevent the computer from overload!");
      //delete node_ptr;
      //return; 
  }
  std::clock_t parallel_wait_time=std::clock();
  future_.waitForFinished(); //Wait if GraphManager ist still computing. Does this skip on empty qfuture?
  ROS_INFO_STREAM_NAMED("timings", "waiting time: "<< ( std::clock() - parallel_wait_time ) / (double)CLOCKS_PER_SEC  <<"sec"); 
  //Q_EMIT newVisualImage(cvMat2QImage(visual_img, 0)); //visual_idx=0
  //Q_EMIT newDepthImage (cvMat2QImage(visual_img, depth_mono8_img_,depth_mono8_img_,1));//overwrites last cvMat2QImage
  //processNode runs in the background and after finishing visualizes the results
//#define CONCURRENT_NODE_CONSTRUCTION 1
#ifdef CONCURRENT_NODE_CONSTRUCTION
  ROS_WARN("Processing Node in parallel");
  future_ =  QtConcurrent::run(this, &OpenNIListener::processNode, visual_img, point_cloud, node_ptr);
#else
  processNode(visual_img, point_cloud, node_ptr);
#endif
  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << "runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}

void OpenNIListener::processNode(cv::Mat& visual_img,   // will be drawn to
                                 const sensor_msgs::PointCloud2ConstPtr point_cloud,
                                 Node* new_node){
  std::clock_t starttime=std::clock();
  Q_EMIT setGUIStatus("GraphSLAM");
  bool has_been_added = graph_mgr_->addNode(new_node);

  //######### Visualization code  #############################################
  cv::Mat feature_img = cv::Mat::zeros(visual_img.rows, visual_img.cols, CV_8UC1);
  if(has_been_added){
      graph_mgr_->drawFeatureFlow(feature_img);
      //Q_EMIT newFeatureFlowImage(cvMat2QImage(visual_img, 2)); //include the feature flow now
  }
  Q_EMIT newFeatureFlowImage(cvMat2QImage(visual_img,depth_mono8_img_, feature_img, 2)); //show registration
  ROS_DEBUG("Sending PointClouds"); 
  //if node position was optimized: publish received pointcloud in new frame
  if (has_been_added && graph_mgr_->freshlyOptimized_ && (pub_cloud_.getNumSubscribers() > 0)){
      ROS_INFO("Sending original pointcloud with appropriatly positioned frame"); 
      sensor_msgs::PointCloud2 msg = *point_cloud;
      msg.header.stamp = graph_mgr_->time_of_last_transform_;
      msg.header.frame_id = "/slam_transform";
      pub_cloud_.publish(msg);
  }
  //slow, mainly for debugging (and only done if subscribed to): Transform pointcloud to fixed coordinate system and resend 
  if (has_been_added && graph_mgr_->freshlyOptimized_ && (pub_transf_cloud_.getNumSubscribers() > 0)){
      ROS_WARN("Sending transformed pointcloud in fixed frame because /rgbdslam/transformed_slowdown_cloud was subscribed to. It's faster to subscribe to /rgbdslam/cloud instead"); 
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl::fromROSMsg(*point_cloud, pcl_cloud);
      pcl_ros::transformPointCloud(pcl_cloud, pcl_cloud, graph_mgr_->world2cam_ );
      sensor_msgs::PointCloud2 msg; 
      pcl::toROSMsg(pcl_cloud, msg);
      msg.header.frame_id = "/openni_camera";
      msg.header.stamp = ros::Time::now();
      pub_transf_cloud_.publish(msg);
  }
  //when first node has been added,  send it out once unchanged as reference
  if (first_frame_ && has_been_added && (pub_ref_cloud_.getNumSubscribers() > 0)){
      ROS_INFO("Sending Reference PointCloud once"); 
      pub_ref_cloud_.publish(*point_cloud);
      first_frame_ = false;
  }

  if(!has_been_added) delete new_node;
  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << "runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}


using namespace cv;
///Analog to opencv example file and modified to use adjusters
FeatureDetector* OpenNIListener::createDetector( const string& detectorType ) {
	ParameterServer* params = ParameterServer::instance();
	FeatureDetector* fd = 0;
    if( !detectorType.compare( "FAST" ) ) {
        //fd = new FastFeatureDetector( 20/*threshold*/, true/*nonmax_suppression*/ );
        fd = new DynamicAdaptedFeatureDetector (new FastAdjuster(20,true), 
												params->get<int>("adjuster_min_keypoints"),
												params->get<int>("adjuster_max_keypoints"),
												params->get<int>("fast_adjuster_max_iterations"));
    }
    else if( !detectorType.compare( "STAR" ) ) {
        fd = new StarFeatureDetector( 16/*max_size*/, 5/*response_threshold*/, 10/*line_threshold_projected*/,
                                      8/*line_threshold_binarized*/, 5/*suppress_nonmax_size*/ );
    }
    else if( !detectorType.compare( "SIFT" ) ) {
        fd = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                     SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
        ROS_INFO("Default SIFT threshold: %f, Default SIFT Edge Threshold: %f", 
                 SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                 SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
    }
    else if( !detectorType.compare( "SURF" ) ) {
        fd = new DynamicAdaptedFeatureDetector(new SurfAdjuster(),
        										params->get<int>("adjuster_min_keypoints"),
												params->get<int>("adjuster_max_keypoints"),
												params->get<int>("surf_adjuster_max_iterations"));
    }
    else if( !detectorType.compare( "MSER" ) ) {
        fd = new MserFeatureDetector( 1/*delta*/, 60/*min_area*/, 14400/*_max_area*/, 0.35f/*max_variation*/,
                0.2/*min_diversity*/, 200/*max_evolution*/, 1.01/*area_threshold*/, 0.003/*min_margin*/,
                5/*edge_blur_size*/ );
    }
    else if( !detectorType.compare( "GFTT" ) ) {
        fd = new GoodFeaturesToTrackDetector( 200/*maxCorners*/, 0.001/*qualityLevel*/, 1./*minDistance*/,
                                              5/*int _blockSize*/, true/*useHarrisDetector*/, 0.04/*k*/ );
    }
    else {
      ROS_ERROR("No valid detector-type given: %s. Using SURF.", detectorType.c_str());
      fd = createDetector("SURF"); //recursive call with correct parameter
    }
    ROS_ERROR_COND(fd == 0, "No detector could be created");
    return fd;
}

DescriptorExtractor* OpenNIListener::createDescriptorExtractor( const string& descriptorType ) {
    DescriptorExtractor* extractor = 0;
    if( !descriptorType.compare( "SIFT" ) ) {
        extractor = new SiftDescriptorExtractor();/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(), bool isNormalize=true, bool recalculateAngles=true, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/
    }
    else if( !descriptorType.compare( "SURF" ) ) {
        extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
    }
    else {
      ROS_ERROR("No valid descriptor-matcher-type given: %s. Using SURF", descriptorType.c_str());
      extractor = createDescriptorExtractor("SURF");
    }
    return extractor;
}

void OpenNIListener::toggleBagRecording(){
  bagfile_mutex.lock();
  save_bag_file = !save_bag_file;
	// save bag
	if (save_bag_file)
	{
		time_t rawtime; 
		struct tm * timeinfo;
		char buffer [80];

		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
    // create a nice name
		strftime (buffer,80,"kinect_%Y-%m-%d-%H-%M-%S.bag",timeinfo);

		bag.open(buffer, rosbag::bagmode::Write);
    ROS_INFO_STREAM("Opened bagfile " << bag.getFileName());
	} else {
    ROS_INFO_STREAM("Closing bagfile " << bag.getFileName());
    bag.close();
  }
  bagfile_mutex.unlock();
}

void OpenNIListener::togglePause(){
  pause_ = !pause_;
  if(pause_) Q_EMIT setGUIStatus("Processing Thread Stopped");
  else Q_EMIT setGUIStatus("Processing Thread Running");
}
void OpenNIListener::getOneFrame(){
  getOneFrame_=true;
}
/// Create a QImage from image. The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
QImage OpenNIListener::cvMat2QImage(const cv::Mat& channel1, const cv::Mat& channel2, const cv::Mat& channel3, unsigned int idx){
  if(rgba_buffers_.size() <= idx){
      rgba_buffers_.resize(idx+1);
  }
  if(channel2.rows != channel1.rows || channel2.cols != channel1.cols ||
     channel3.rows != channel1.rows || channel3.cols != channel1.cols){
     ROS_ERROR("Image channels to be combined differ in size");
  }
  if(channel1.rows != rgba_buffers_[idx].rows || channel1.cols != rgba_buffers_[idx].cols){
    ROS_DEBUG("Created new rgba_buffer with index %i", idx);
    rgba_buffers_[idx] = cv::Mat( channel1.rows, channel1.cols, CV_8UC4); 
    printMatrixInfo(rgba_buffers_[idx]);
  }
  std::vector<cv::Mat> input;
  cv::Mat alpha( channel1.rows, channel1.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  input.push_back(channel1);
  input.push_back(channel2);
  input.push_back(channel3);
  input.push_back(alpha);
  cv::merge(input, rgba_buffers_[idx]);
  printMatrixInfo(rgba_buffers_[idx]);
  return QImage((unsigned char *)(rgba_buffers_[idx].data), 
                rgba_buffers_[idx].cols, rgba_buffers_[idx].rows, 
                rgba_buffers_[idx].step, QImage::Format_RGB32 );
}

/// Create a QImage from image. The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
QImage OpenNIListener::cvMat2QImage(const cv::Mat& image, unsigned int idx){
  ROS_DEBUG_STREAM("Converting Matrix of type " << openCVCode2String(image.type()) << " to RGBA");
  if(rgba_buffers_.size() <= idx){
      rgba_buffers_.resize(idx+1);
  }
  ROS_DEBUG_STREAM("Target Matrix is of type " << openCVCode2String(rgba_buffers_[idx].type()));
  if(image.rows != rgba_buffers_[idx].rows || image.cols != rgba_buffers_[idx].cols){
    ROS_DEBUG("Created new rgba_buffer with index %i", idx);
    rgba_buffers_[idx] = cv::Mat( image.rows, image.cols, CV_8UC4); 
    printMatrixInfo(rgba_buffers_[idx]);
  }
  cv::Mat alpha( image.rows, image.cols, CV_8UC1, cv::Scalar(255)); //TODO this could be buffered for performance
  cv::Mat in[] = { image, alpha };
  // rgba[0] -> bgr[2], rgba[1] -> bgr[1],
  // rgba[2] -> bgr[0], rgba[3] -> alpha[0]
  int from_to[] = { 0,0,  0,1,  0,2,  1,3 };
  mixChannels( in , 2, &rgba_buffers_[idx], 1, from_to, 4 );
  printMatrixInfo(rgba_buffers_[idx]);
  //cv::cvtColor(image, rgba_buffers_, CV_GRAY2RGBA);}
  //}
  return QImage((unsigned char *)(rgba_buffers_[idx].data), 
                rgba_buffers_[idx].cols, rgba_buffers_[idx].rows, 
                rgba_buffers_[idx].step, QImage::Format_RGB32 );
}

/*/ Old helper functions from earlier times. Could be useful lateron
void transformPointCloud (const Eigen::Matrix4f &transform, const sensor_msgs::PointCloud2 &in,
                          sensor_msgs::PointCloud2 &out)
{
  // Get X-Y-Z indices
  int x_idx = pcl::getFieldIndex (in, "x");
  int y_idx = pcl::getFieldIndex (in, "y");
  int z_idx = pcl::getFieldIndex (in, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    ROS_ERROR ("Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
    return;
  }

  if (in.fields[x_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[y_idx].datatype != sensor_msgs::PointField::FLOAT32 || 
      in.fields[z_idx].datatype != sensor_msgs::PointField::FLOAT32) {
    ROS_ERROR ("X-Y-Z coordinates not floats. Currently only floats are supported.");
    return;
  }
  // Check if distance is available
  int dist_idx = pcl::getFieldIndex (in, "distance");
  // Copy the other data
  if (&in != &out)
  {
    out.header = in.header;
    out.height = in.height;
    out.width  = in.width;
    out.fields = in.fields;
    out.is_bigendian = in.is_bigendian;
    out.point_step   = in.point_step;
    out.row_step     = in.row_step;
    out.is_dense     = in.is_dense;
    out.data.resize (in.data.size ());
    // Copy everything as it's faster than copying individual elements
    memcpy (&out.data[0], &in.data[0], in.data.size ());
  }

  Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

  for (size_t i = 0; i < in.width * in.height; ++i) {
    Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
    Eigen::Vector4f pt_out;
    
    bool max_range_point = false;
    int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
    float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
    if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2])) {
      if (distance_ptr==NULL || !std::isfinite(*distance_ptr)) { // Invalid point 
        pt_out = pt;
      } else { // max range point
        pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
        pt_out = transform * pt;
        max_range_point = true;
        //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
      }
    } else {
      pt_out = transform * pt;
    }

    if (max_range_point) {
      // Save x value in distance again
      *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
      pt_out[0] = std::numeric_limits<float>::quiet_NaN();
      //std::cout << __PRETTY_FUNCTION__<<": "<<i << "is a max range point.\n";
    }

    memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
    memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
    memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));
    
    xyz_offset += in.point_step;
  }

  // Check if the viewpoint information is present
  int vp_idx = pcl::getFieldIndex (in, "vp_x");
  if (vp_idx != -1) {
    // Transform the viewpoint info too
    for (size_t i = 0; i < out.width * out.height; ++i) {
      float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
      // Assume vp_x, vp_y, vp_z are consecutive
      Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
      Eigen::Vector4f vp_out = transform * vp_in;

      pstep[0] = vp_out[0];
      pstep[1] = vp_out[1];
      pstep[2] = vp_out[2];
    }
  }
}
*/

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
  //The following doesn't work due to NaNs
  //double minVal, maxVal; 
  //minMaxLoc(float_img, &minVal, &maxVal);
  //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
  //mono8_img = cv::Scalar(0);
  //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
}

//Little debugging helper functions
std::string openCVCode2String(unsigned int code){
  switch(code){
    case 0 : return std::string("CV_8UC1" );
    case 8 : return std::string("CV_8UC2" );
    case 16: return std::string("CV_8UC3" );
    case 24: return std::string("CV_8UC4" );
    case 2 : return std::string("CV_16UC1");
    case 10: return std::string("CV_16UC2");
    case 18: return std::string("CV_16UC3");
    case 26: return std::string("CV_16UC4");
    case 5 : return std::string("CV_32FC1");
    case 13: return std::string("CV_32FC2");
    case 21: return std::string("CV_32FC3");
    case 29: return std::string("CV_32FC4");
  }
  return std::string("Unknown");
}

void printMatrixInfo(cv::Mat& image){
  ROS_DEBUG_STREAM("Matrix Type:" << openCVCode2String(image.type()) <<  " rows: " <<  image.rows  <<  " cols: " <<  image.cols);
}
