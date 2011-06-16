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


#include "node.h"
#include <cmath>
#include <ctime>
#include <Eigen/Geometry>
#include "pcl/ros/conversions.h"
#include <pcl/common/transformation_from_correspondences.h>
#include <opencv2/highgui/highgui.hpp>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 

#ifdef USE_SIFT_GPU
#include "sift_gpu_feature_detector.h"
#endif

//#include <math.h>
#include <fstream>
#ifdef USE_ICP_BIN
#include "gicp-fallback.h"
#endif

#ifdef USE_ICP_CODE
#include "../gicp/transform.h"
#endif


//#include <iostream>
#include <Eigen/StdVector>

Transformation3 eigen2Hogman(const Eigen::Matrix4f eigen_mat) {
  std::clock_t starttime=std::clock();

  Eigen::Affine3f eigen_transform(eigen_mat);
  Eigen::Quaternionf eigen_quat(eigen_transform.rotation());
  Vector3 translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
  Quaternion rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(),
      eigen_quat.w());
  Transformation3 result(translation, rotation);

  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << "runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
  return result;
}

Node::Node(ros::NodeHandle& nh, const cv::Mat& visual,
    cv::Ptr<cv::FeatureDetector> detector,
    cv::Ptr<cv::DescriptorExtractor> extractor,
    cv::Ptr<cv::DescriptorMatcher> matcher,
    const sensor_msgs::PointCloud2ConstPtr point_cloud,
    const cv::Mat& detection_mask)
: id_(0), 
flannIndex(NULL),
matcher_(matcher)
{
#ifdef USE_ICP_CODE
  gicp_initialized = false;
#endif
  std::clock_t starttime=std::clock();

#ifdef USE_SIFT_GPU
  SiftGPUFeatureDetector* siftgpu = SiftGPUFeatureDetector::GetInstance();
  float* descriptors = siftgpu->detect(visual, feature_locations_2d_);
  if (descriptors == NULL) {
    ROS_FATAL("Can't run SiftGPU");
  }
#else
  ROS_FATAL_COND(detector.empty(), "No valid detector!");
  detector->detect( visual, feature_locations_2d_, detection_mask);// fill 2d locations
#endif

  ROS_INFO("Feature detection and descriptor extraction runtime: %f", ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC);
  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature detection runtime: " << ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC );

  /*
    if (id_  == 0)
        cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("clouds_from_node_base",10);
    else{
   */
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/rgbdslam/batch_clouds",20);
  //   cloud_pub_ransac = nh_->advertise<sensor_msgs::PointCloud2>("clouds_from_node_current_ransac",10);
  //} */

  // get pcl::Pointcloud to extract depthValues a pixel positions
  std::clock_t starttime5=std::clock();
  // TODO: If batch sending/saving of clouds would be removed, the pointcloud wouldn't have to be saved
  // which would slim down the Memory requirements
  pcl::fromROSMsg(*point_cloud,pc_col);
  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime5) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "pc2->pcl conversion runtime: " << ( std::clock() - starttime5 ) / (double)CLOCKS_PER_SEC );

  // project pixels to 3dPositions and create search structures for the gicp
#ifdef USE_SIFT_GPU
  // removes also unused descriptors from the descriptors matrix
  // build descriptor matrix
  projectTo3DSiftGPU(feature_locations_2d_, feature_locations_3d_, pc_col, descriptors, feature_descriptors_); //takes less than 0.01 sec

  if (descriptors != NULL) delete descriptors;

#else
  projectTo3D(feature_locations_2d_, feature_locations_3d_, pc_col); //takes less than 0.01 sec
#endif

  // projectTo3d need a dense cloud to use the points.at(px.x,px.y)-Call
#ifdef USE_ICP_CODE
  std::clock_t starttime4=std::clock();
  createGICPStructures(); 
  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime4) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "gicp runtime: " << ( std::clock() - starttime4 ) / (double)CLOCKS_PER_SEC );
#endif

  std::clock_t starttime2=std::clock();
#ifndef USE_SIFT_GPU
//  ROS_INFO("Use extractor");
  //cv::Mat topleft, topright;
  //topleft = visual.colRange(0,visual.cols/2+50);
  //topright= visual.colRange(visual.cols/2+50, visual.cols-1);
	//std::vector<cv::KeyPoint> kp1, kp2; 
  //extractor->compute(topleft, kp1, feature_descriptors_); //fill feature_descriptors_ with information 
  extractor->compute(visual, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
#endif
  assert(feature_locations_2d_.size() == feature_locations_3d_.size());
  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime2) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "Feature extraction runtime: " << ( std::clock() - starttime2 ) / (double)CLOCKS_PER_SEC );

  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "constructor runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}

Node::~Node(){
  if(flannIndex)
    delete flannIndex;
}

void Node::publish(const char* frame, ros::Time timestamp){
  if (cloud_pub_.getNumSubscribers() > 0){
    sensor_msgs::PointCloud2 cloudMessage;
    pcl::toROSMsg(pc_col,cloudMessage);
    cloudMessage.header.frame_id = frame;
    cloudMessage.header.stamp = timestamp;
    cloud_pub_.publish(cloudMessage);
    ROS_INFO("Pointcloud with id %i sent with frame %s", id_, frame);
  } else 
    ROS_INFO("Sending of point cloud requested, but no subscriber. Ignored");
}


#ifdef USE_ICP_CODE
bool Node::getRelativeTransformationTo_ICP_code(const Node* target_node,Eigen::Matrix4f& transformation,
    const Eigen::Matrix4f* initial_transformation){
  //std::clock_t starttime_icp = std::clock();
  dgc_transform_t initial;

  // use optional matrix as first guess in icp
  if (initial_transformation == NULL){
    gicpSetIdentity(initial); 
  }else {
    Eigen2GICP(*initial_transformation,initial);
  }


  dgc_transform_t final_trafo;
  dgc_transform_identity(final_trafo);


  assert(gicp_initialized && target_node->gicp_initialized);

  unsigned int iterations = target_node->gicp_point_set->AlignScan(this->gicp_point_set, initial, final_trafo, gicp_d_max);


  GICP2Eigen(final_trafo,transformation);

  //ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime_icp) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "ICP 2 runtime: " << ( std::clock() - starttime_icp ) / (double)CLOCKS_PER_SEC );

  return iterations < gicp_max_iterations;

}

# endif

#ifdef USE_ICP_BIN
bool Node::getRelativeTransformationTo_ICP_bin(const Node* target_node,
    Eigen::Matrix4f& transformation,
    const Eigen::Matrix4f* initial_transformation){
  std::clock_t starttime_icp = std::clock();

  bool converged;

  if (initial_transformation != NULL)
  {
    pointcloud_type pc2;
    pcl::transformPointCloud(pc_col,pc2,*initial_transformation);
    converged = gicpfallback(pc2,target_node->pc_col, transformation);
  }
  else {
    converged = gicpfallback(pc_col,target_node->pc_col, transformation); }

  // Paper
  // ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime_icp) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "ICP runtime: " << ( std::clock() - starttime_icp ) / (double)CLOCKS_PER_SEC );

  return converged;
}
#endif
//#ifndef USE_ICP_CODE
//#ifndef USE_ICP_BIN
//bool Node::getRelativeTransformationTo_ICP(const Node* target_node,
//    Eigen::Matrix4f& transformation,
//    const Eigen::Matrix4f* initial_transformation){
//  ROS_ERROR("ICP is called but not available, as compilation did not include it");
//  return false;
//}
//#endif
//#endif

// build search structure for descriptor matching
void Node::buildFlannIndex() {
  //std::clock_t starttime=std::clock();
  // use same type as in http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
  flannIndex = new cv_flannIndex(feature_descriptors_, cv::flann::KDTreeIndexParams(4));
  ROS_DEBUG("Built flannIndex (address %p) for Node %i", flannIndex, this->id_);
  // ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.001, "timings", "buildFlannIndex runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}


#ifdef USE_ICP_CODE
void Node::createGICPStructures(unsigned int max_count){

  gicp_point_set = new dgc::gicp::GICPPointSet();

  dgc::gicp::GICPPoint g_p;
  g_p.range = -1;
  for(int k = 0; k < 3; k++) {
    for(int l = 0; l < 3; l++) {
      g_p.C[k][l] = (k == l)?1:0;
    }
  }

  int step = 1;
  if (pc_col.points.size()>max_count)
    step = ceil(pc_col.points.size()*1.0/max_count);

  int cnt = 0;
  for (unsigned int i=0; i<pc_col.points.size(); i++ ){
    point_type  p = pc_col.points.at(i);
    if (!(isnan(p.x) || isnan(p.y) || isnan(p.z))) {
      // add points to pointset for icp
      if (cnt++%step == 0){
        g_p.x=p.x;
        g_p.y=p.y;
        g_p.z=p.z;
        gicp_point_set->AppendPoint(g_p);
      }
    }
  }
  ROS_WARN("gicp_point_set.Size() %i", gicp_point_set->Size() );


  std::clock_t starttime_gicp = std::clock();
  // build search structure for gicp:
  gicp_point_set->SetDebug(false);
  gicp_point_set->SetGICPEpsilon(gicp_epsilon);
  gicp_point_set->BuildKDTree();
  gicp_point_set->ComputeMatrices();
  gicp_point_set->SetMaxIterationInner(8); // as in test_gicp->cpp
  gicp_point_set->SetMaxIteration(gicp_max_iterations);
  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime_gicp) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "function runtime to create gicp-Structures: "<< ( std::clock() - starttime_gicp ) / (double)CLOCKS_PER_SEC  <<"sec");

  ROS_INFO_STREAM("time for creating the structure: " << ((std::clock()-starttime_gicp*1.0) / (double)CLOCKS_PER_SEC));
  ROS_INFO_STREAM("current: " << std::clock() << " " << "start_time: " << starttime_gicp);

  gicp_initialized = true;

}
#endif

//TODO: This function seems to be resistant to paralellization probably due to knnSearch
int Node::findPairsFlann(const Node* other, vector<cv::DMatch>* matches) const {
  std::clock_t starttime=std::clock();
  assert(matches->size()==0);

  if (other->flannIndex == NULL) {
    ROS_FATAL("Node %i in findPairsFlann: flann Index of Node %i was not initialized", this->id_, other->id_);
    return -1;
  }

  // number of neighbours found (has to be two, see l. 57)
  const int k = 2;

  // compare
  // http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
  cv::Mat indices(feature_descriptors_.rows, k, CV_32S);
  cv::Mat dists(feature_descriptors_.rows, k, CV_32F);

  //ROS_INFO("find flann pairs: feature_descriptor (rows): %i", feature_descriptors_.rows);

  // get the best two neighbours
  other->flannIndex->knnSearch(feature_descriptors_, indices, dists, k,
      cv::flann::SearchParams(64));

  int* indices_ptr = indices.ptr<int> (0);
  float* dists_ptr = dists.ptr<float> (0);

  cv::DMatch match;
  for (int i = 0; i < indices.rows; ++i) {
    if (dists_ptr[2 * i] < 0.6 * dists_ptr[2 * i + 1]) {
      match.queryIdx = i;
      match.trainIdx = indices_ptr[2 * i];
      match.distance = dists_ptr[2 * i];

      assert(match.trainIdx < other->feature_descriptors_.rows);
      assert(match.queryIdx < feature_descriptors_.rows);

      matches->push_back(match);
    }
  }

  //ROS_INFO("matches size: %i, rows: %i", (int) matches->size(), feature_descriptors_.rows);

  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "findPairsFlann runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
  return matches->size();
}



#ifdef USE_SIFT_GPU
void Node::projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
    const pointcloud_type& point_cloud, float* descriptors_in, cv::Mat& descriptors_out){

  std::clock_t starttime=std::clock();

  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  std::list<int> featuresUsed;

  int index = -1;
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    ++index;

    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= point_cloud.width || p2d.x < 0 ||
        p2d.y >= point_cloud.height || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    point_type p3d = point_cloud.at((int) p2d.x,(int) p2d.y);

    if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z)){
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    i++; //Only increment if no element is removed from vector
  }

  //create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  for (int y = 0; y < size && featuresUsed.size() > 0; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      descriptors_out.at<float>(y, x) = descriptors_in[id * 128 + x];
    }
  }

  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}



#else
void Node::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
    const pointcloud_type& point_cloud){

  std::clock_t starttime=std::clock();

  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= point_cloud.width || p2d.x < 0 ||
        p2d.y >= point_cloud.height || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    point_type p3d = point_cloud.at((int) p2d.x,(int) p2d.y);

    //ROS_INFO("3d: %f, %f, %f, 2d: %f, %f", p3d.x, p3d.y, p3d.z, p2d.x, p2d.y);

    if ( isnan(p3d.x) || isnan(p3d.y) || isnan(p3d.z)){
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    i++; //Only increment if no element is removed from vector
  }



  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}
#endif



void Node::computeInliersAndError(const std::vector<cv::DMatch>& matches,
                                  const Eigen::Matrix4f& transformation,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  std::vector<cv::DMatch>& inliers, //output var
                                  double& mean_error,
                                  vector<double>& errors,
                                  double squaredMaxInlierDistInM) const{ //output var

  std::clock_t starttime=std::clock();

  inliers.clear();
  errors.clear();

  vector<pair<float,int> > dists;
  std::vector<cv::DMatch> inliers_temp;

  assert(matches.size() > 0);
  mean_error = 0.0;
  for (unsigned int j = 0; j < matches.size(); j++){ //compute new error and inliers

    unsigned int this_id = matches[j].queryIdx;
    unsigned int earlier_id = matches[j].trainIdx;

    Eigen::Vector4f vec = (transformation * origins[this_id]) - earlier[earlier_id];

    double error = vec.dot(vec);

    if(error > squaredMaxInlierDistInM)
      continue; //ignore outliers
    if(!(error >= 0.0)){
      ROS_ERROR_STREAM("Transformation for error !> 0: " << transformation);
      ROS_ERROR_STREAM(error << " " << matches.size());
    }
    error = sqrt(error);
    dists.push_back(pair<float,int>(error,j));
    inliers_temp.push_back(matches[j]); //include inlier

    mean_error += error;
    errors.push_back(error);
  }

  if (inliers_temp.size()<3){ //at least the samples should be inliers
    ROS_WARN_COND(inliers_temp.size() > 3, "No inliers at all in %d matches!", (int)matches.size()); // only warn if this checks for all initial matches
    mean_error = 1e9;
  } else {
    mean_error /= inliers_temp.size();

    // sort inlier ascending according to their error
    sort(dists.begin(),dists.end());

    inliers.resize(inliers_temp.size());
    for (unsigned int i=0; i<inliers_temp.size(); i++){
      inliers[i] = matches[dists[i].second];
    }
  }
  if(!(mean_error>0)) ROS_ERROR_STREAM("Transformation for mean error !> 0: " << transformation);
  if(!(mean_error>0)) ROS_ERROR_STREAM(mean_error << " " << inliers_temp.size());
  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");

}

template<class InputIterator>
Eigen::Matrix4f Node::getTransformFromMatches(const Node* earlier_node,
                                              InputIterator iter_begin,
                                              InputIterator iter_end,
                                              bool& valid, 
                                              float max_dist_m) const 
{
  pcl::TransformationFromCorrespondences tfc;
  valid = true;
  vector<Eigen::Vector3f> t, f;

  for ( ;iter_begin!=iter_end; iter_begin++) {
    int this_id    = iter_begin->queryIdx;
    int earlier_id = iter_begin->trainIdx;

    Eigen::Vector3f from(this->feature_locations_3d_[this_id][0],
                         this->feature_locations_3d_[this_id][1],
                         this->feature_locations_3d_[this_id][2]);
    Eigen::Vector3f  to (earlier_node->feature_locations_3d_[earlier_id][0],
                         earlier_node->feature_locations_3d_[earlier_id][1],
                         earlier_node->feature_locations_3d_[earlier_id][2]);
    if (max_dist_m > 0) {  //storing is only necessary, if max_dist is given
      f.push_back(from);
      t.push_back(to);    
    }
    tfc.add(from, to, 1.0/to(0));//the further, the less weight b/c of accuracy decay
  }


  // find smalles distance between a point and its neighbour in the same cloud
  // je groesser das dreieck aufgespannt ist, desto weniger fallen kleine positionsfehler der einzelnen
  // Punkte ist Gewicht!

  if (max_dist_m > 0)
  {  
    //float min_neighbour_dist = 1e6;
    Eigen::Matrix4f foo;

    valid = true;
    for (uint i=0; i<f.size(); i++)
    {
      float d_f = (f.at((i+1)%f.size())-f.at(i)).norm();
      float d_t = (t.at((i+1)%t.size())-t.at(i)).norm();

      if ( abs(d_f-d_t) > max_dist_m ) {
        valid = false;
        return Eigen::Matrix4f();
      }
    }
    //here one could signal that some samples are very close, but as the transformation is validated elsewhere we don't
    //if (min_neighbour_dist < 0.5) { ROS_INFO...}
  }
  // get relative movement from samples
  return tfc.getTransformation().matrix();
}


///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool Node::getRelativeTransformationTo(const Node* earlier_node,
                                       std::vector<cv::DMatch>* initial_matches,
                                       Eigen::Matrix4f& resulting_transformation,
                                       float& rmse, 
                                       std::vector<cv::DMatch>& matches,
                                       unsigned int ransac_iterations) const
{
  std::clock_t starttime=std::clock();

  assert(initial_matches != NULL);
  matches.clear();
  
  if(initial_matches->size() <= (unsigned int) ParameterServer::instance()->get<int>("min_matches")){
    ROS_INFO("Only %d feature matches between %d and %d (minimal: %i)",(int)initial_matches->size() , this->id_, earlier_node->id_, ParameterServer::instance()->get<int>("min_matches"));
    return false;
  }

  unsigned int min_inlier_threshold = int(initial_matches->size()*0.2);
  std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
  double inlier_error; //all squared errors
  srand((long)std::clock());
  
  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");
  vector<double> dummy;

  // best values of all iterations (including invalids)
  double best_error = 1e6, best_error_invalid = 1e6;
  unsigned int best_inlier_invalid = 0, best_inlier_cnt = 0, valid_iterations = 0;

  Eigen::Matrix4f transformation;
  
  const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
  for (uint n_iter = 0; n_iter < ransac_iterations; n_iter++) {
    //generate a map of samples. Using a map solves the problem of drawing a sample more than once
    std::set<cv::DMatch> sample_matches;
    std::vector<cv::DMatch> sample_matches_vector;
    while(sample_matches.size() < sample_size){
      int id = rand() % initial_matches->size();
      sample_matches.insert(initial_matches->at(id));
      sample_matches_vector.push_back(initial_matches->at(id));
    }

    bool valid; // valid is false iff the sampled points clearly aren't inliers themself 
    transformation = getTransformFromMatches(earlier_node, sample_matches.begin(), sample_matches.end(),valid,max_dist_m);
    if (!valid) continue; // valid is false iff the sampled points aren't inliers themself 
    if(transformation!=transformation) continue; //Contains NaN
    
    //test whether samples are inliers (more strict than before)
    computeInliersAndError(sample_matches_vector, transformation, this->feature_locations_3d_, 
                           earlier_node->feature_locations_3d_, inlier, inlier_error,  /*output*/
                           dummy, max_dist_m*max_dist_m); 
    if(inlier_error > 1000) continue; //most possibly a false match in the samples
    computeInliersAndError(*initial_matches, transformation, this->feature_locations_3d_, 
                           earlier_node->feature_locations_3d_, inlier, inlier_error,  /*output*/
                           dummy, max_dist_m*max_dist_m);

    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid) {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }
    // ROS_INFO("iteration %d  cnt: %d, best: %d,  error: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100);

    if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
      //inlier.size() < ((float)initial_matches->size())*min_inlier_ratio || 
      // ROS_INFO("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
      continue;
    }
    // ROS_INFO("Refining iteration from %i samples: all matches: %i, inliers: %i, inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), inlier_error);
    valid_iterations++;
    //if (inlier_error > 0) ROS_ERROR("size: %i", (int)dummy.size());
    assert(inlier_error>0);

    //Performance hacks:
    ///Iterations with more than half of the initial_matches inlying, count twice
    if (inlier.size() > initial_matches->size()*0.5) n_iter++;
    ///Iterations with more than 80% of the initial_matches inlying, count threefold
    if (inlier.size() > initial_matches->size()*0.8) n_iter++;



    if (inlier_error < best_error) { //copy this to the result
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      best_inlier_cnt = inlier.size();
      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
      rmse = inlier_error;
      best_error = inlier_error;
      // ROS_INFO("  new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);

    }else
    {
      // ROS_INFO("NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);
    }

    //int max_ndx = min((int) min_inlier_threshold,30); //? What is this 30?
    double new_inlier_error;

    transformation = getTransformFromMatches(earlier_node, matches.begin(), matches.end(), valid); // compute new trafo from all inliers:
    if(transformation!=transformation) continue; //Contains NaN
    computeInliersAndError(*initial_matches, transformation,
                           this->feature_locations_3d_, earlier_node->feature_locations_3d_,
                           inlier, new_inlier_error, dummy, max_dist_m*max_dist_m);

    // ROS_INFO("asd recomputed: inliersize: %i, inlier error: %f", (int) inlier.size(),100*new_inlier_error);


    // check also invalid iterations
    if (inlier.size() > best_inlier_invalid)
    {
      best_inlier_invalid = inlier.size();
      best_error_invalid = inlier_error;
    }

    if(inlier.size() < min_inlier_threshold || new_inlier_error > max_dist_m){
      //inlier.size() < ((float)initial_matches->size())*min_inlier_ratio || 
      // ROS_INFO("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
      continue;
    }
    // ROS_INFO("Refined iteration from %i samples: all matches %i, inliers: %i, new_inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), new_inlier_error);

    assert(new_inlier_error>0);

    if (new_inlier_error < best_error) 
    {
      resulting_transformation = transformation;
      matches = inlier;
      assert(matches.size()>= min_inlier_threshold);
      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
      rmse = new_inlier_error;
      best_error = new_inlier_error;
      // ROS_INFO("  improved: new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
    }else
    {
      // ROS_INFO("improved: NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
    }
  } //iterations
  ROS_INFO("%i good iterations (from %i), inlier pct %i, inlier cnt: %i, error: %.2f cm",valid_iterations, (int) ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse*100);
  // ROS_INFO("best overall: inlier: %i, error: %.2f",best_inlier_invalid, best_error_invalid*100);

  ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > ParameterServer::instance()->get<double>("min_time_reported"), "timings", __FUNCTION__ << "runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
  return matches.size() >= min_inlier_threshold;
}


#ifdef USE_ICP_CODE
void Node::Eigen2GICP(const Eigen::Matrix4f& m, dgc_transform_t g_m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      g_m[i][j] = m(i,j);

}
void Node::GICP2Eigen(const dgc_transform_t g_m, Eigen::Matrix4f& m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      m(i,j) = g_m[i][j];
}

void Node::gicpSetIdentity(dgc_transform_t m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      if (i==j)
        m[i][j] = 1;
      else
        m[i][j] = 0;
}
#endif


//TODO: Merge this with processNodePair
MatchingResult Node::matchNodePair(const Node* older_node){
  MatchingResult mr;
  const unsigned int min_matches = 16; // minimal number of feature correspondences to be a valid candidate for a link
  // std::clock_t starttime=std::clock();


  this->findPairsFlann(older_node, &mr.all_matches); 
  ROS_DEBUG("found %i inital matches",(int) mr.all_matches.size());
  if (mr.all_matches.size() < min_matches){
    ROS_INFO("Too few inliers: Adding no Edge between %i and %i. Only %i correspondences to begin with.",
        older_node->id_,this->id_,(int)mr.all_matches.size());
  } 
  else if (!getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches) ){ // mr.all_matches.size()/3
      ROS_INFO("Found no valid trafo, but had initially %d feature matches",(int) mr.all_matches.size());
  } else  {

      mr.final_trafo = mr.ransac_trafo;
      
#ifdef USE_ICP_CODE
      getRelativeTransformationTo_ICP_code(older_node,mr.icp_trafo, &mr.ransac_trafo);
#endif  
      
#ifdef USE_ICP_BIN
      // improve transformation by using the generalized ICP
      // std::clock_t starttime_gicp1 = std::clock();
      bool converged = getRelativeTransformationTo_ICP_bin(older_node,mr.icp_trafo, &mr.ransac_trafo);
      //ROS_INFO_STREAM("Paper: ICP1: " << ((std::clock()-starttime_gicp1*1.0) / (double)CLOCKS_PER_SEC));


      ROS_INFO("icp: inliers: %i", mr.inlier_matches.size());
      
      if (!converged) { 
        ROS_INFO("ICP did not converge. No Edge added");
        return mr;
      }

      mr.final_trafo = mr.ransac_trafo * mr.icp_trafo;

      MatchingResult mr_icp;
      vector<double> errors;
      double error;
      std::vector<cv::DMatch> inliers;
      // check if icp improves alignment:
      computeInliersAndError(mr.inlier_matches, mr.final_trafo,
          this->feature_locations_3d_, older_node->feature_locations_3d_,
          inliers, error, errors, 0.04*0.04); 

      for (uint i=0; i<errors.size(); i++)
      {
        cout << "error: " << round(errors[i]*10000)/100 << endl;
      }

      cout << "error was: " << mr.rmse << " and is now: " << error << endl;

      double roll, pitch, yaw, dist;
      mat2components(mr.ransac_trafo, roll, pitch, yaw, dist);
      cout << "ransac: " << roll << " "<< pitch << " "<< yaw << "   "<< dist << endl;


      mat2components(mr.icp_trafo, roll, pitch, yaw, dist);
      cout << "icp: " << roll << " "<< pitch << " "<< yaw << "   "<< dist << endl;

      mat2components(mr.final_trafo, roll, pitch, yaw, dist);
      cout << "final: " << roll << " "<< pitch << " "<< yaw << "   "<< dist << endl;


      cout << "ransac: " << endl << mr.ransac_trafo << endl;
      cout << "icp: " << endl << mr.icp_trafo << endl;
      cout << "total: " << endl << mr.final_trafo << endl;


      if (error > (mr.rmse+0.02))
      {
        cout << "#### icp-error is too large, ignoring the connection" << endl;
        return mr;
      }
#endif
       

#ifndef USE_ICP_BIN
#ifndef USE_ICP_CODE      
      mr.final_trafo = mr.ransac_trafo;    
#endif
#endif

      mr.edge.id1 = older_node->id_;//and we have a valid transformation
      mr.edge.id2 = this->id_; //since there are enough matching features,
      mr.edge.mean = eigen2Hogman(mr.final_trafo);//we insert an edge between the frames
      mr.edge.informationMatrix =   Matrix6::eye(mr.inlier_matches.size()*mr.inlier_matches.size()); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)
  }
  // Paper
  return mr;
}

///Get euler angles from affine matrix (helper for isBigTrafo)
void Node::mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw) {
  roll = atan2(t(2,1),t(2,2));
  pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
  yaw = atan2(t(1,0),t(0,0));
}

void Node::mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist){

  mat2RPY(t, roll,pitch,yaw);
  mat2dist(t, dist);

  roll = roll/M_PI*180;
  pitch = pitch/M_PI*180;
  yaw = yaw/M_PI*180;

}

