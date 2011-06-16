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


#ifndef OPENNI_LISTENER_H
#define OPENNI_LISTENER_H
#include "ros/ros.h"
//#include <pcl_tf/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "graph_manager.h"
#include <qtconcurrentrun.h>
#include <QImage> //for cvMat2QImage not listet here but defined in cpp file
#include <rosbag/bag.h>


//The policy merges kinect messages with approximately equal timestamp into one callback 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                        sensor_msgs::Image, 
                                                        sensor_msgs::PointCloud2> MySyncPolicy;

//!Handles most of the ROS-based communication

/** The purpose of this class is to listen to 
 * synchronized image pairs from the kinect
 * convert them to opencv, process them, convert
 * them to a qt image and send them to the mainwindow
 * defined in qtcv.h/.cpp
 */
class OpenNIListener : public QObject {
  ///QT Stuff, to communicate with the gui
  Q_OBJECT
  Q_SIGNALS:
    ///Connect to this signal to get up-to-date optical images from the listener
    void newVisualImage(QImage);
    ///Connect to this signal to get up-to-date featureFlow visualizations from the listener
    void newFeatureFlowImage(QImage);
    ///Connect to this signal to get up-to-date depth images from the listener
    void newDepthImage(QImage);
    ///Connect to this signal to get the transformation matrix from the last frame as QString
    void newTransformationMatrix(QString);
    //void pauseStatus(bool is_paused);
    ///Set the info label on the right side in the statusbar of the GUI
    void setGUIInfo(QString message);
    ///Set the temporary status-message in the GUI
    void setGUIStatus(QString message);
  public Q_SLOTS:
    ///Switch between processing or ignoring new incoming data
    void togglePause();
    void toggleBagRecording();
    ///Process a single incomming frame. Useful in pause-mode for getting one snapshot at a time
    void getOneFrame();

  public:
    //!Ctor: setup synced listening to kinect data and prepare the feature handling
    /*!Constructor: The listener needs to know the topic for the pointcloud, 
     * the GraphManager instance, gray and depth image
       Valid detector types: FAST, STAR, SIFT, SURF, MSER, GFTT
       Valid extractor types: SIFT, SURF
       As a detector SURF is better, FAST is faster, GFTT is fast but really bad in combination the SURF Extractor
    */
    OpenNIListener(ros::NodeHandle nh, GraphManager* g_mgr, const char* visual_topic,  
                   const char* depth_topic, const char* cloud_topic,
                   const char* detector_type, const char* extractor_type); 

    //! Listen to kinect data, construct nodes and feed the graph manager with it.
    /*! For each dataset from the kinect, do some data conversion,
     *  construct a node, hand it to the graph manager and
     *  do some visualization of the result in the GUI and RVIZ.
     */
    void cameraCallback (const sensor_msgs::ImageConstPtr& visual_img,  
                         const sensor_msgs::ImageConstPtr& depth_img, 
                         const sensor_msgs::PointCloud2ConstPtr& point_cloud);
  
    ///The GraphManager uses the Node objects to do the actual SLAM
    ///Public, s.t. the qt signals can be connected to by the holder of the OpenNIListener
    GraphManager* graph_mgr_;
    
  protected:
    /// Create a QImage from image. The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
    QImage cvMat2QImage(const cv::Mat& image, unsigned int idx); 
    QImage cvMat2QImage(const cv::Mat& channel1, const cv::Mat& channel2, const cv::Mat& channel3, unsigned int idx);

    //processNode is called by cameraCallback in a separate thread and after finishing visualizes the results
    void processNode(cv::Mat& visual_img,  
                     const sensor_msgs::PointCloud2ConstPtr point_cloud,
                     Node* new_node);
    /// Creates Feature Detector Objects accordingt to the type.
    /// Possible detectorTypes: FAST, STAR, SIFT, SURF, GFTT
    /// FAST and SURF are the self-adjusting versions (see http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_feature_detectors.html#DynamicAdaptedFeatureDetector)
    cv::FeatureDetector* createDetector( const string& detectorType );
    /// Create an object to extract features at keypoints. The Exctractor is passed to the Node constructor and must be the same for each node.
    cv::DescriptorExtractor* createDescriptorExtractor( const string& descriptorType );

    //Variables
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher > matcher_;
    message_filters::Subscriber<sensor_msgs::Image> visual_sub_ ;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    
    message_filters::Synchronizer<MySyncPolicy> sync_;
    cv::Mat depth_mono8_img_;
    std::vector<cv::Mat> rgba_buffers_;
    ros::NodeHandle nh_; // store to give it to the nodes
    ros::Publisher pub_cloud_;
    ros::Publisher pub_transf_cloud_;
    ros::Publisher pub_ref_cloud_;
    
    rosbag::Bag bag;
    bool save_bag_file;
    bool read_from_bag_file;
    
    //ros::Publisher pc_pub; 
    /*unsigned int callback_counter_;*/
    bool pause_;
    bool getOneFrame_;
    bool first_frame_;
    QFuture<void> future_;
    QMutex bagfile_mutex;
};

/*/Copied from pcl_tf/transform.cpp
void transformPointCloud (const Eigen::Matrix4f &transform, 
                          const sensor_msgs::PointCloud2 &in,
                          sensor_msgs::PointCloud2 &out);*/

///Convert the CV_32FC1 image to CV_8UC1 with a fixed scale factor
void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img);

///Return the macro string for the cv::Mat type integer
std::string openCVCode2String(unsigned int code);

///Print Type and size of image
void printMatrixInfo(cv::Mat& image);
/*
struct images_and_cloud {
	sensor_msgs::ImageConstPtr rgb;
	sensor_msgs::ImageConstPtr depth;
	sensor_msgs::PointCloud2ConstPtr points;
	
	bool valid () {
		return (rgb != NULL && depth != NULL && points != NULL);
	}
	
	void reset () {
		rgb.reset(); depth.reset(); points.reset();
	}
};
*/
#endif
