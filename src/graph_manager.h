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


/*
 * graph_manager.h
 *
 *  Created on: 19.01.2011
 *      Author: hess
 */

#ifndef GRAPH_MANAGER_H_
#define GRAPH_MANAGER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "node.h"
#include <hogman_minimal/graph_optimizer_hogman/graph_optimizer3d_hchol.h>
#include <hogman_minimal/graph/loadEdges3d.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <map>
#include <QObject>
#include <QString>
#include <QMatrix4x4>
#include <QList>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <memory> //for auto_ptr
#include "glviewer.h"
#include "globaldefinitions.h"

//#define ROSCONSOLE_SEVERITY_INFO

namespace AIS = AISNavigation;

//!Computes a globally optimal trajectory from transformations between Node-pairs
class GraphManager : public QObject {
    Q_OBJECT
    Q_SIGNALS:
    ///Connect to this signal to get the transformation matrix from the last frame as QString
    void newTransformationMatrix(QString);
    void sendFinished();
    void setGUIInfo(QString message);
    void setGUIStatus(QString message);
    void setPointCloud(pointcloud_type const * pc, QMatrix4x4 transformation);
    void updateTransforms(QList<QMatrix4x4>* transformations);
    void setGUIInfo2(QString message);
    void setGraphEdges(QList<QPair<int, int> >* edge_list);
    void deleteLastNode();

    
    
    
    public Q_SLOTS:
    /// Start over with new graph
    void reset();
    ///iterate over all Nodes, sending their transform and pointcloud
    void sendAllClouds();
    ///Call saveAllCloudsToFile, if possible as background thread
    void saveAllClouds(QString filename, bool compact);
    ///Throw the last node out, reoptimize
    void deleteLastFrame(); 
    void setMaxDepth(float max_depth);

    public:
    GraphManager(ros::NodeHandle, GLViewer* glviewer);
    ~GraphManager();

    /// Add new node to the graph.
    /// Node will be included, if a valid transformation to one of the former nodes
    /// can be found. If appropriate, the graph is optimized
    /// graphmanager owns newNode after this call. Do no delete the object
    bool addNode(Node* newNode); 

    ///Draw the features's motions onto the canvas
    ///for the edge computed in the last call of hogmanEdge.
    ///This should always be edge between the first and the last inserted nodes
    void drawFeatureFlow(cv::Mat& canvas, 
                         cv::Scalar line_color = cv::Scalar(255,0,0,0), 
                         cv::Scalar circle_color = cv::Scalar(0,0,255,0)); 

    ///Flag to indicate that the graph is globally corrected after the addNode call.
    ///However, currently optimization is done in every call anyhow
    bool freshlyOptimized_;
    GLViewer glviewer_;
    ros::Time time_of_last_transform_;
    tf::Transform  world2cam_;
    std::map<int, Node* > graph_;
    
    void flannNeighbours();

    float Max_Depth;
    //void setMaxDepth(float max_depth);
protected:

    ///iterate over all Nodes, transform them to the fixed frame, aggregate and send 
    void saveAllCloudsToFile(QString filename, bool compact);
    void pointCloud2MeshFile(QString filename, pointcloud_type full_cloud);
    std::vector < cv::DMatch > last_inlier_matches_;
    std::vector < cv::DMatch > last_matches_;
    /// The parameter max_targets determines how many potential edges are wanted
    /// max_targets < 0: No limit
    /// max_targets = 0: Compare to first frame only
    /// max_targets = 1: Compare to previous frame only
    /// max_targets > 1: Select intelligently
    std::vector<int> getPotentialEdgeTargets(const Node* new_node, int max_targets);
    
    std::vector<int> getPotentialEdgeTargetsFeatures(const Node* new_node, int max_targets);
    
    void optimizeGraph();
    void initializeHogman();
    bool addEdgeToHogman(AIS::LoadedEdge3D edge, bool good_edge);


    ///Send markers to visualize the graph edges (cam transforms) in rviz (if somebody subscribed)
    void visualizeGraphEdges() const;
    ///Send markers to visualize the graph nodes (cam positions) in rviz (if somebody subscribed)
    void visualizeGraphNodes() const;
    ///Send markers to visualize the graph ids in rviz (if somebody subscribed)
    void visualizeGraphIds() const;
        
    
    
    ///Send markers to visualize the last matched features in rviz (if somebody subscribed)
    void visualizeFeatureFlow3D(unsigned int marker_id = 0,
                                bool draw_outlier = true) const;
    ///Send the transform between openni_camera (the initial position of the cam)
    ///and the cumulative motion. 
    ///This is called periodically by a ros timer and after each optimizer run
    void broadcastTransform(const ros::TimerEvent& event);
    ///Constructs a list of transformation matrices from all nodes (used for visualization in glviewer)
    ///The object lives on the heap and needs to be destroyed somewhere else (i.e. in glviewer)
    QList<QMatrix4x4>* getAllPosesAsMatrixList();
    QList<QPair<int, int> >* getGraphEdges() const;
    void resetGraph();


    void mergeAllClouds(pointcloud_type & merge);
    
    
    AIS::GraphOptimizer3D* optimizer_;

    ros::Publisher marker_pub_; 
    //ros::Publisher transform_pub_;
    ros::Publisher ransac_marker_pub_;
    ros::Publisher aggregate_cloud_pub_;
    ros::Publisher sampled_cloud_pub_;
    
    
    ros::Timer timer_;
    tf::TransformBroadcaster br_;
    tf::Transform kinect_transform_; ///<transformation of the last frame to the first frame (assuming the first one is fixed)
    //Eigen::Matrix4f latest_transform_;///<same as kinect_transform_ as Eigen
    QMatrix4x4 latest_transform_;///<same as kinect_transform_ as Eigen


    // true if translation > 10cm or largest euler-angle>5 deg
    // used to decide if the camera has moved far enough to generate a new nodes
    bool isBigTrafo(const Eigen::Matrix4f& t);
    bool isBigTrafo(const Transformation3& t);

    /// get euler angles from 4x4 homogenous
    void static mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw);
    /// get translation-distance from 4x4 homogenous
    void static mat2dist(const Eigen::Matrix4f& t, double &dist);
    void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist);

    bool reset_request_;
    std::clock_t last_batch_update_;
    unsigned int marker_id;
    int last_matching_node_;
    bool batch_processing_runs_;

};

geometry_msgs::Point pointInWorldFrame(const Eigen::Vector4f& point3d, Transformation3 transf);
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, pointcloud_type &cloud_to_append_to,
                                   const tf::Transform transformation, float Max_Depth, bool compact);

#endif /* GRAPH_MANAGER_H_ */
