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


#include <hogman_minimal/stuff/macros.h>
#include <hogman_minimal/math/transformation.h>
#include <sys/time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
//#include <rgbdslam/CloudTransforms.h>
#include "graph_manager.h"
#include "pcl_ros/transforms.h"
#include "pcl/io/pcd_io.h"
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/features2d/features2d.hpp>
#include <QThread>
#include <qtconcurrentrun.h>
#include <QtConcurrentMap> 
#include <QFile>
#include <utility>



QMatrix4x4 hogman2QMatrix(const Transformation3 hogman_trans) {
    std::clock_t starttime=std::clock();
    _Matrix< 4, 4, double > m = hogman_trans.toMatrix(); //_Matrix< 4, 4, double >
    QMatrix4x4 qmat( static_cast<qreal*>( m[0] )  );
    return qmat;
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

tf::Transform hogman2TF(const Transformation3 hogman_trans) {
    std::clock_t starttime=std::clock();

    tf::Transform result;
    tf::Vector3 translation;
    translation.setX(hogman_trans.translation().x());
    translation.setY(hogman_trans.translation().y());
    translation.setZ(hogman_trans.translation().z());

    tf::Quaternion rotation;
    rotation.setX(hogman_trans.rotation().x());
    rotation.setY(hogman_trans.rotation().y());
    rotation.setZ(hogman_trans.rotation().z());
    rotation.setW(hogman_trans.rotation().w());

    result.setOrigin(translation);
    result.setRotation(rotation);
    return result;

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 

}

void printTransform(const char* name, const tf::Transform t) {
    ROS_DEBUG_STREAM(name << ": Translation " << t.getOrigin().x() << " " << t.getOrigin().y() << " " << t.getOrigin().z());
    ROS_DEBUG_STREAM(name << ": Rotation " << t.getRotation().getX() << " " << t.getRotation().getY() << " " << t.getRotation().getZ() << " " << t.getRotation().getW());
}


GraphManager::GraphManager(ros::NodeHandle nh, GLViewer* glviewer) :
    freshlyOptimized_(true), //the empty graph is "optimized" i.e., sendable
    glviewer_(glviewer),
    time_of_last_transform_(ros::Time()),
    optimizer_(0), 
    latest_transform_(), //constructs identity
    reset_request_(false),
    last_batch_update_(std::clock()),
    marker_id(0),
    last_matching_node_(-1),
    batch_processing_runs_(false)
{
    std::clock_t starttime=std::clock();

    int numLevels = 3;
    int nodeDistance = 2;
    optimizer_ = new AIS::HCholOptimizer3D(numLevels, nodeDistance);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/pose_graph_markers", 1);
    kinect_transform_.setRotation(tf::Quaternion::getIdentity());//TODO: initialize transloation too
    timer_ = nh.createTimer(ros::Duration(0.1), &GraphManager::broadcastTransform, this);

    ransac_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rgbdslam/correspondence_marker", 10);
    aggregate_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/rgbdslam/aggregate_clouds",20);

    Max_Depth = -1;

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

GraphManager::~GraphManager() {
  //TODO: delete all Nodes
    //for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
    delete (optimizer_);
}

void GraphManager::drawFeatureFlow(cv::Mat& canvas, cv::Scalar line_color,
                                   cv::Scalar circle_color){
    std::clock_t starttime=std::clock();
    ROS_DEBUG("Number of features to draw: %d", (int)last_inlier_matches_.size());

    const double pi_fourth = 3.14159265358979323846 / 4.0;
    const int line_thickness = 1;
    const int circle_radius = 6;
    if(graph_.size() < 2)return;// || last_matching_node_ < 0) return; //feature flow is only available between at least two nodes

    Node* earliernode = graph_[graph_.size()-2];//graph_.size()-2; //compare current to previous
    Node* newernode = graph_[graph_.size()-1];

    //encircle all keypoints in this image
    //for(unsigned int feat = 0; feat < newernode->feature_locations_2d_.size(); feat++) {
    //    cv::Point2f p; 
    //    p = newernode->feature_locations_2d_[feat].pt;
    //    cv::circle(canvas, p, circle_radius, circle_color, line_thickness, 8);
    //}
    cv::Mat tmpimage = cv::Mat::zeros(canvas.rows, canvas.cols, CV_8UC1);
    cv::drawKeypoints(tmpimage, newernode->feature_locations_2d_, tmpimage, cv::Scalar(90), 5);
    canvas+=tmpimage;
    for(unsigned int mtch = 0; mtch < last_inlier_matches_.size(); mtch++) {
        cv::Point2f p,q; //TODO: Use sub-pixel-accuracy
        unsigned int newer_idx = last_inlier_matches_[mtch].queryIdx;
        unsigned int earlier_idx = last_inlier_matches_[mtch].trainIdx;
        q = newernode->feature_locations_2d_[newer_idx].pt;
        p = earliernode->feature_locations_2d_[earlier_idx].pt;

        double angle;    angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
        double hypotenuse = cv::norm(p-q);
            cv::line(canvas, p, q, line_color, line_thickness, 8);
        if(hypotenuse > 1.5){  //only larger motions larger than one pix get an arrow tip
            cv::line( canvas, p, q, line_color, line_thickness, 8 );
            /* Now draw the tips of the arrow.  */
            p.x =  (q.x + 4 * cos(angle + pi_fourth));
            p.y =  (q.y + 4 * sin(angle + pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, 8 );
            p.x =  (q.x + 4 * cos(angle - pi_fourth));
            p.y =  (q.y + 4 * sin(angle - pi_fourth));
            cv::line( canvas, p, q, line_color, line_thickness, 8 );
        } else { //draw a smaller circle into the bigger one 
            cv::circle(canvas, p, circle_radius-2, circle_color, line_thickness, 8);
        }
    }
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}


/// max_targets determines how many potential edges are wanted
/// max_targets < 0: No limit
/// max_targets = 0: Compare to first frame only
/// max_targets = 1: Compare to previous frame only
/// max_targets > 1: Select intelligently (TODO: rather stupid at the moment)
std::vector<int> GraphManager::getPotentialEdgeTargets(const Node* new_node, int max_targets){
    const int last_targets = 3; //always compare to the last n, spread evenly for the rest
    double max_id_plus1 = (double)graph_.size()- last_targets;
    max_targets -= last_targets;
    std::vector<int> ids_to_link_to;
    std::stringstream ss;
    ss << "Node ID's for comparison with the new node " << graph_.size() << ":";
    //Special Cases
    if(graph_.size() == 0){
        ROS_WARN("Do not call this function as long as the graph is empty");
        return ids_to_link_to;
    }
    for(int i = 2; i <= (int)graph_.size() && i <= last_targets; i++){
        ss << "(" << graph_.size()-i << "), " ; 
        ids_to_link_to.push_back(graph_.size()-i);
    }
    if(max_targets < 0) 
        return ids_to_link_to;
    if(max_targets == 0){
        ids_to_link_to.push_back(0);
        return ids_to_link_to; //only compare to first frame
    } else if(max_targets == 1){
        ids_to_link_to.push_back(graph_.size()-2); 
        return ids_to_link_to; //only compare to previous frame
    } 
    
    //End special cases
    /*
    if(max_targets >= 10){ //compare to last 5, then split up evenly among the rest
        for(unsigned int i = graph_.size(); i > graph_.size() -5; i--){
          ids_to_link_to.push_back(i); 
          max_targets--;
          max_id_plus1--;
        }
    } 
    */

    double id = 0.0; //always start with the first
    double increment = max_id_plus1/(double)(max_targets);//max_targets-1 = intermediate steps
    increment = increment < 1.0 ? 1.0 : increment; //smaller steps would select some id's twice
    ROS_DEBUG("preparing loop %f %f", id, increment);
    while((int)id < (int)max_id_plus1){
        ss << "(" << id << "/" << (int)id << ")," ; 
        ids_to_link_to.push_back((int)id); //implicit rounding
        id += increment;
    }
    ROS_DEBUG("%s", ss.str().c_str());
//    while((id+0.5) < max_id_plus1){ //Split up evenly
//        ids_to_link_to.push_back((int)(id+0.5)); //implicit rounding
//        id += (max_id_plus1/(double)max_targets);
//    }
    return ids_to_link_to;
}

void GraphManager::resetGraph(){
    int numLevels = 3;
    int nodeDistance = 2;
    marker_id =0;
    time_of_last_transform_= ros::Time();
    last_batch_update_=std::clock();
    delete optimizer_; 
    optimizer_ = new AIS::HCholOptimizer3D(numLevels, nodeDistance);
    graph_.clear();//TODO: also delete the nodes
    freshlyOptimized_= false;
    reset_request_ = false;
}

// returns true, iff node could be added to the cloud
bool GraphManager::addNode(Node* new_node) {
    std::clock_t starttime=std::clock();

    last_inlier_matches_.clear();
    if(reset_request_) resetGraph(); 

    if (new_node->feature_locations_2d_.size() <= 50){
        ROS_DEBUG("found only %i features on image, node is not included",(int)new_node->feature_locations_2d_.size());
        return false;
    }

    //set the node id only if the node is actually added to the graph
    //needs to be done here as the graph size can change inside this function
    new_node->id_ = graph_.size();
    ROS_DEBUG("New Node with id %i has address %p (GraphManager is %p)", new_node->id_, new_node, this);

    //First Node, so only build its index, insert into storage and add a
    //vertex at the origin, of which the position is very certain
    if (graph_.size()==0){
        new_node->buildFlannIndex(); // create index so that next nodes can use it
        graph_[new_node->id_] = new_node;
        optimizer_->addVertex(0, Transformation3(), 1e9*Matrix6::eye(1.0)); //fix at origin
        QString message;
        Q_EMIT setGUIInfo(message.sprintf("Added first node with %i keypoints to the graph", (int)new_node->feature_locations_2d_.size()));
        pointcloud_type const * the_pc(&(new_node->pc_col));
        Q_EMIT setPointCloud(the_pc, latest_transform_);
        ROS_DEBUG("GraphManager is thread %d, New Node is at (%p, %p)", (unsigned int)QThread::currentThreadId(), new_node, graph_[0]);
        return true;
    }

    unsigned int num_edges_before = optimizer_->edges().size(); 

    ROS_DEBUG("Graphsize: %d", (int) graph_.size());
    marker_id = 0; //overdraw old markers
    last_matching_node_ = -1;


    //MAIN LOOP: Compare node pairs ######################################################################
    //First check if trafo to last frame is big
    Node* prev_frame = graph_[graph_.size()-1];
    ROS_INFO("Comparing new node (%i) with previous node %i / %i", new_node->id_, (int)graph_.size()-1, prev_frame->id_);
    MatchingResult mr = new_node->matchNodePair(prev_frame);
    if(mr.edge.id1 >= 0 && !isBigTrafo(mr.edge.mean)){
        ROS_WARN("Transformation not relevant. Did not add as Node");
        return false;
    } else if(mr.edge.id1 >= 0){
        if (addEdgeToHogman(mr.edge, true)) {
            ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
            last_matching_node_ = mr.edge.id1;
            last_inlier_matches_ = mr.inlier_matches;
            last_matches_ = mr.all_matches;
        }
    }
    //Eigen::Matrix4f ransac_trafo, final_trafo;
    std::vector<int> vertices_to_comp = getPotentialEdgeTargets(new_node, global_connectivity); //vernetzungsgrad
    QList<const Node* > nodes_to_comp;//only necessary for parallel computation
    for (int id_of_id = (int)vertices_to_comp.size()-1; id_of_id >=0;id_of_id--){ 

#ifndef CONCURRENT_EDGE_COMPUTATION
#define QT_NO_CONCURRENT
#endif
#ifndef QT_NO_CONCURRENT
        //First compile a qlist of the nodes to be compared, then run the comparisons in parallel, 
        //collecting a qlist of the results (using the blocking version of mapped).
        nodes_to_comp.push_back(graph_[vertices_to_comp[id_of_id]]);
    }
    ROS_DEBUG("Running node comparisons in parallel");
    QList<MatchingResult> results = QtConcurrent::blockingMapped(nodes_to_comp, 
                                                                 boost::bind(&Node::matchNodePair, new_node, _1));
    for(int i = 0; i <  results.size(); i++){
        MatchingResult& mr = results[i];
#else
        Node* abcd = graph_[vertices_to_comp[id_of_id]];
        ROS_INFO("Comparing new node (%i) with node %i / %i", new_node->id_, vertices_to_comp[id_of_id], abcd->id_);
        MatchingResult mr = new_node->matchNodePair(abcd);
#endif
        if(mr.edge.id1 >= 0){
            if (addEdgeToHogman(mr.edge, isBigTrafo(mr.edge.mean))) { //TODO: result isBigTrafo is not considered
                ROS_INFO("Added Edge between %i and %i. Inliers: %i",mr.edge.id1,mr.edge.id2,(int) mr.inlier_matches.size());
                last_matching_node_ = mr.edge.id1;
                last_inlier_matches_ = mr.inlier_matches;
                last_matches_ = mr.all_matches;
            }
        }
    }
    //END OF MAIN LOOP: Compare node pairs ######################################################################

    if (optimizer_->edges().size() > num_edges_before) { //Success
        new_node->buildFlannIndex();
        graph_[new_node->id_] = new_node;
        ROS_INFO("Added Node, new Graphsize: %i", (int) graph_.size());
        optimizeGraph();
        Q_EMIT updateTransforms(getAllPosesAsMatrixList());
        Q_EMIT setGraphEdges(getGraphEdges());
        //make the transform of the last node known
        broadcastTransform(ros::TimerEvent());
        visualizeGraphEdges();
        visualizeGraphNodes();
        visualizeFeatureFlow3D(marker_id++);
        pointcloud_type const * the_pc(&(new_node->pc_col));
        Q_EMIT setPointCloud(the_pc, latest_transform_);
        ROS_DEBUG("GraphManager is thread %d", (unsigned int)QThread::currentThreadId());
    }else{
        //delete new_node; //is now  done by auto_ptr
        ROS_WARN("Did not add as Node");
    }
    QString message;
    Q_EMIT setGUIInfo(message.sprintf("%s, Graph Size: %iN/%iE, Duration: %f, Inliers: %i, &chi;<sup>2</sup>: %f", 
                                     (optimizer_->edges().size() > num_edges_before) ? "Added" : "Ignored",
                                     (int)optimizer_->vertices().size(), (int)optimizer_->edges().size(),
                                     (std::clock()-starttime) / (double)CLOCKS_PER_SEC, (int)last_inlier_matches_.size(),
                                     optimizer_->chi2()));
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", __FUNCTION__ << " runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
    return (optimizer_->edges().size() > num_edges_before);
}


///Get the norm of the translational part of an affine matrix (Helper for isBigTrafo)
void GraphManager::mat2dist(const Eigen::Matrix4f& t, double &dist){
    dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
}
///Get euler angles from affine matrix (helper for isBigTrafo)
void GraphManager::mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw) {
    roll = atan2(t(2,1),t(2,2));
    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
    yaw = atan2(t(1,0),t(0,0));
}
// true iff edge qualifies for generating a new vertex
bool GraphManager::isBigTrafo(const Eigen::Matrix4f& t){
    double roll, pitch, yaw, dist;

    mat2RPY(t, roll,pitch,yaw);
    mat2dist(t, dist);

    roll = roll/M_PI*180;
    pitch = pitch/M_PI*180;
    yaw = yaw/M_PI*180;

    double max_angle = max(roll,max(pitch,yaw));

    // at least 10cm or 5deg
    return (dist > global_min_translation_meter || max_angle > global_min_rotation_degree);
}

bool GraphManager::isBigTrafo(const Transformation3& t){
    float angle_around_axis = 2.0*acos(t._rotation.w()) *180.0 / M_PI;
    float dist = t._translation.norm();
    QString infostring;
    ROS_INFO("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    infostring.sprintf("Rotation:% 4.2f, Distance: % 4.3fm", angle_around_axis, dist);
    Q_EMIT setGUIInfo2(infostring);
    return (dist > global_min_translation_meter || angle_around_axis > global_min_rotation_degree);
}
void GraphManager::visualizeFeatureFlow3D(unsigned int marker_id,
                                          bool draw_outlier) const{
    std::clock_t starttime=std::clock();
    if (ransac_marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking

        visualization_msgs::Marker marker_lines;

        marker_lines.header.frame_id = "/openni_rgb_optical_frame";
        marker_lines.ns = "ransac_markers";
        marker_lines.header.stamp = ros::Time::now();
        marker_lines.action = visualization_msgs::Marker::ADD;
        marker_lines.pose.orientation.w = 1.0;
        marker_lines.id = marker_id;
        marker_lines.type = visualization_msgs::Marker::LINE_LIST;
        marker_lines.scale.x = 0.002;
        
        std_msgs::ColorRGBA color_red  ;  //red outlier
        color_red.r = 1.0;
        color_red.a = 1.0;
        std_msgs::ColorRGBA color_green;  //green inlier, newer endpoint
        color_green.g = 1.0;
        color_green.a = 1.0;
        std_msgs::ColorRGBA color_yellow;  //yellow inlier, earlier endpoint
        color_yellow.r = 1.0;
        color_yellow.g = 1.0;
        color_yellow.a = 1.0;
        std_msgs::ColorRGBA color_blue  ;  //red-blue outlier
        color_blue.b = 1.0;
        color_blue.a = 1.0;

        marker_lines.color = color_green; //just to set the alpha channel to non-zero

        const AISNavigation::PoseGraph3D::Vertex* earlier_v; //used to get the transform
        const AISNavigation::PoseGraph3D::Vertex* newer_v; //used to get the transform
        AISNavigation::PoseGraph3D::VertexIDMap v_idmap = optimizer_->vertices();
        // end of initialization
        ROS_DEBUG("Matches Visualization start: %lu Matches, %lu Inliers", last_matches_.size(), last_inlier_matches_.size());

        // write all inital matches to the line_list
        marker_lines.points.clear();//necessary?

        if (draw_outlier)
        {
            for (unsigned int i=0;i<last_matches_.size(); i++){
                int newer_id = last_matches_.at(i).queryIdx; //feature id in newer node
                int earlier_id = last_matches_.at(i).trainIdx; //feature id in earlier node

                //earlier_v = static_cast<const AISNavigation::PoseGraph2D::Vertex*>(v_idmap[graph_[last_matching_node_]->id_]);
                earlier_v = optimizer_->vertex(last_matching_node_);
                //newer_v = static_cast<const AISNavigation::PoseGraph3D::Vertex*>(v_idmap[graph_[graph_.size()-1]->id_]);
                newer_v = optimizer_->vertex(graph_.size()-1);

                //Outliers are red (newer) to blue (older)
                marker_lines.colors.push_back(color_red);
                marker_lines.colors.push_back(color_blue);

                Node* last = graph_.find(graph_.size()-1)->second;
                marker_lines.points.push_back(
                        pointInWorldFrame(last->feature_locations_3d_[newer_id], newer_v->transformation));
                Node* prev = graph_.find(last_matching_node_)->second;
                marker_lines.points.push_back(
                        pointInWorldFrame(prev->feature_locations_3d_[earlier_id], earlier_v->transformation));
            }
        }

        for (unsigned int i=0;i<last_inlier_matches_.size(); i++){
            int newer_id = last_inlier_matches_.at(i).queryIdx; //feature id in newer node
            int earlier_id = last_inlier_matches_.at(i).trainIdx; //feature id in earlier node

            //earlier_v = static_cast<AISNavigation::PoseGraph3D::Vertex*>(v_idmap[graph_[last_matching_node_]->id_]);
            earlier_v = optimizer_->vertex(last_matching_node_);
            //newer_v = static_cast<AISNavigation::PoseGraph3D::Vertex*>(v_idmap[graph_[graph_.size()-1]->id_]);
            newer_v = optimizer_->vertex(graph_.size()-1);


            //inliers are green (newer) to blue (older)
            marker_lines.colors.push_back(color_green);
            marker_lines.colors.push_back(color_blue);

            Node* last = graph_.find(graph_.size()-1)->second;
            marker_lines.points.push_back(
                    pointInWorldFrame(last->feature_locations_3d_[newer_id], newer_v->transformation));
            Node* prev = graph_.find(last_matching_node_)->second;
            marker_lines.points.push_back(
                    pointInWorldFrame(prev->feature_locations_3d_[earlier_id], earlier_v->transformation));
        }

        ransac_marker_pub_.publish(marker_lines);
        ROS_DEBUG_STREAM("Published  " << marker_lines.points.size()/2 << " lines");
    }
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}

//do spurious type conversions
geometry_msgs::Point pointInWorldFrame(const Eigen::Vector4f& point3d,
        Transformation3 transf){
    Vector3f tmp(point3d[0], point3d[1], point3d[2]);
    tmp = transf * tmp; //transform to world frame
    geometry_msgs::Point p;
    p.x = tmp.x(); 
    p.y = tmp.y(); 
    p.z = tmp.z();
    return p;
}


QList<QPair<int, int> >* GraphManager::getGraphEdges() const {
    std::clock_t starttime=std::clock();
    QList<QPair<int, int> >* edge_list = new QList<QPair<int, int> >();
    AISNavigation::PoseGraph3D::Vertex *v1, *v2; //used in loop
    AISNavigation::PoseGraph3D::EdgeSet::iterator edge_iter = optimizer_->edges().begin();
    for(;edge_iter != optimizer_->edges().end(); edge_iter++) {
        v1 = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->from());
        v2 = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->to());
        edge_list->append( qMakePair(v1->id(), v2->id()));
    }
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", __FUNCTION__ << " runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
    return edge_list;
}
void GraphManager::visualizeGraphEdges() const {
    std::clock_t starttime=std::clock();

    if (marker_pub_.getNumSubscribers() > 0){ //no visualization for nobody
        visualization_msgs::Marker edges_marker;
        edges_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        edges_marker.header.stamp = ros::Time::now();
        edges_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        edges_marker.id = 0;    // Any marker sent with the same namespace and id will overwrite the old one

        edges_marker.type = visualization_msgs::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        edges_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        edges_marker.scale.x = 0.005; //line width
        //Global pose (used to transform all points)
        edges_marker.pose.position.x = 0;
        edges_marker.pose.position.y = 0;
        edges_marker.pose.position.z = 0;
        edges_marker.pose.orientation.x = 0.0;
        edges_marker.pose.orientation.y = 0.0;
        edges_marker.pose.orientation.z = 0.0;
        edges_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        edges_marker.color.r = 1.0f;
        edges_marker.color.g = 1.0f;
        edges_marker.color.b = 1.0f;
        edges_marker.color.a = 0.5f;//looks smoother
        geometry_msgs::Point point; //start and endpoint for each line segment
        AISNavigation::PoseGraph3D::Vertex* v; //used in loop
        AISNavigation::PoseGraph3D::EdgeSet::iterator edge_iter = optimizer_->edges().begin();
        int counter = 0;
        for(;edge_iter != optimizer_->edges().end(); edge_iter++, counter++) {
            v = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->from());
            point.x = v->transformation.translation().x();
            point.y = v->transformation.translation().y();
            point.z = v->transformation.translation().z();
            edges_marker.points.push_back(point);
            v = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->to());
            point.x = v->transformation.translation().x();
            point.y = v->transformation.translation().y();
            point.z = v->transformation.translation().z();
            edges_marker.points.push_back(point);
        }

        marker_pub_.publish (edges_marker);
        ROS_INFO("published %d graph edges", counter);
    }

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GraphManager::visualizeGraphNodes() const {
    std::clock_t starttime=std::clock();

    if (marker_pub_.getNumSubscribers() > 0){ //don't visualize, if nobody's looking
        visualization_msgs::Marker nodes_marker;
        nodes_marker.header.frame_id = "/openni_rgb_optical_frame"; //TODO: Should be a meaningfull fixed frame with known relative pose to the camera
        nodes_marker.header.stamp = ros::Time::now();
        nodes_marker.ns = "camera_pose_graph"; // Set the namespace and id for this marker.  This serves to create a unique ID
        nodes_marker.id = 1;    // Any marker sent with the same namespace and id will overwrite the old one


        nodes_marker.type = visualization_msgs::Marker::LINE_LIST;
        nodes_marker.action = visualization_msgs::Marker::ADD; // Set the marker action.  Options are ADD and DELETE
        nodes_marker.frame_locked = true; //rviz automatically retransforms the markers into the frame every update cycle
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        nodes_marker.scale.x = 0.002;
        //Global pose (used to transform all points) //TODO: is this the default pose anyway?
        nodes_marker.pose.position.x = 0;
        nodes_marker.pose.position.y = 0;
        nodes_marker.pose.position.z = 0;
        nodes_marker.pose.orientation.x = 0.0;
        nodes_marker.pose.orientation.y = 0.0;
        nodes_marker.pose.orientation.z = 0.0;
        nodes_marker.pose.orientation.w = 1.0;
        // Set the color -- be sure to set alpha to something non-zero!
        nodes_marker.color.r = 1.0f;
        nodes_marker.color.g = 0.0f;
        nodes_marker.color.b = 0.0f;
        nodes_marker.color.a = 1.0f;


        geometry_msgs::Point tail; //same startpoint for each line segment
        geometry_msgs::Point tip;  //different endpoint for each line segment
        std_msgs::ColorRGBA arrow_color_red  ;  //red x axis
        arrow_color_red.r = 1.0;
        arrow_color_red.a = 1.0;
        std_msgs::ColorRGBA arrow_color_green;  //green y axis
        arrow_color_green.g = 1.0;
        arrow_color_green.a = 1.0;
        std_msgs::ColorRGBA arrow_color_blue ;  //blue z axis
        arrow_color_blue.b = 1.0;
        arrow_color_blue.a = 1.0;
        Vector3f origin(0.0,0.0,0.0);
        Vector3f x_axis(0.2,0.0,0.0); //20cm long axis for the first (almost fixed) node
        Vector3f y_axis(0.0,0.2,0.0);
        Vector3f z_axis(0.0,0.0,0.2);
        Vector3f tmp; //the transformed endpoints
        int counter = 0;
        AISNavigation::PoseGraph3D::Vertex* v; //used in loop
        AISNavigation::PoseGraph3D::VertexIDMap::iterator vertex_iter = optimizer_->vertices().begin();
        for(/*see above*/; vertex_iter != optimizer_->vertices().end(); vertex_iter++, counter++) {
            v = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*vertex_iter).second);
            //v->transformation.rotation().x()+ v->transformation.rotation().y()+ v->transformation.rotation().z()+ v->transformation.rotation().w();
            tmp = v->transformation * origin;
            tail.x = tmp.x();
            tail.y = tmp.y();
            tail.z = tmp.z();
            //Endpoints X-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_red);
            tmp = v->transformation * x_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_red);
            //Endpoints Y-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_green);
            tmp = v->transformation * y_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_green);
            //Endpoints Z-Axis
            nodes_marker.points.push_back(tail);
            nodes_marker.colors.push_back(arrow_color_blue);
            tmp = v->transformation * z_axis;
            tip.x  = tmp.x();
            tip.y  = tmp.y();
            tip.z  = tmp.z();
            nodes_marker.points.push_back(tip);
            nodes_marker.colors.push_back(arrow_color_blue);
            //shorten all nodes after the first one
            x_axis.x() = 0.1;
            y_axis.y() = 0.1;
            z_axis.z() = 0.1;
        }

        marker_pub_.publish (nodes_marker);
        ROS_INFO("published %d graph nodes", counter);
    }

    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

bool GraphManager::addEdgeToHogman(AIS::LoadedEdge3D edge, bool largeEdge) {
    std::clock_t starttime=std::clock();
    freshlyOptimized_ = false;

    AIS::PoseGraph3D::Vertex* v1 = optimizer_->vertex(edge.id1);
    AIS::PoseGraph3D::Vertex* v2 = optimizer_->vertex(edge.id2);


    // at least one vertex has to be created, assert that the transformation
    // is large enough to avoid to many vertices on the same spot
    if (!v1 || !v2){
        if (!largeEdge) {
            ROS_INFO("Edge to new vertex is to short, vertex will not be inserted");
            return false; 
        }
    }

    if (!v1) {
        v1 = optimizer_->addVertex(edge.id1, Transformation3(), Matrix6::eye(1.0));
        assert(v1);
    }
    if (!v2) {
        v2 = optimizer_->addVertex(edge.id2, Transformation3(), Matrix6::eye(1.0));
        assert(v2);
    }
    optimizer_->addEdge(v1, v2, edge.mean, edge.informationMatrix);
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 

    return true;

}

void GraphManager::optimizeGraph(){
    std::clock_t starttime=std::clock();
    const int iterations = 10;
    int currentIt = optimizer_->optimize(iterations, true);

    ROS_INFO_STREAM("Hogman Statistics: " << optimizer_->vertices().size() << " nodes, " 
                    << optimizer_->edges().size() << " edges. "
                    << "chi2: " << optimizer_->chi2()
                    << ", Iterations: " << currentIt);

    freshlyOptimized_ = true;

    AISNavigation::PoseGraph3D::Vertex* v = optimizer_->vertex(optimizer_->vertices().size()-1);
    kinect_transform_ =  hogman2TF(v->transformation);
    //pcl_ros::transformAsMatrix(kinect_transform_, latest_transform_);
    latest_transform_ = hogman2QMatrix(v->transformation); 

    /*publish the corrected transforms to the visualization module every five seconds
    if( ((std::clock()-last_batch_update_) / (double)CLOCKS_PER_SEC) > 2){
        publishCorrectedTransforms();
        last_batch_update_ = std::clock();
    }*/
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GraphManager::broadcastTransform(const ros::TimerEvent& ){

    tf::Transform cam2rgb;
    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    cam2rgb.setOrigin(tf::Point(0,-0.04,0));


    world2cam_ = cam2rgb*kinect_transform_;
    //printTransform("kinect", kinect_transform_);
    time_of_last_transform_ = ros::Time::now();
    br_.sendTransform(tf::StampedTransform(world2cam_, time_of_last_transform_,
                      "/openni_camera", "/slam_transform"));

    if(!batch_processing_runs_)
        br_.sendTransform(tf::StampedTransform(cam2rgb, time_of_last_transform_,
                          "/openni_camera", "/batch_transform"));

    //visualize the transformation
    std::stringstream ss;
    Eigen::Matrix4f transform;
    pcl_ros::transformAsMatrix(kinect_transform_, transform);
    ss << "<b>Current Camera Transformation w.r.t. the Initial Frame</b>";
    ss << "<pre>" <<  transform << "</pre>";
    QString mystring(ss.str().c_str());
    Q_EMIT newTransformationMatrix(mystring);

}

/**
 * Publish the updated transforms for the graph node resp. clouds
 *
void GraphManager::publishCorrectedTransforms(){
    std::clock_t starttime=std::clock();
    //fill message
    rgbdslam::CloudTransforms msg;
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        AIS::PoseGraph3D::Vertex* v = optimizer_->vertex(i);
        tf::Transform trans = hogman2TF(v->transformation);
        geometry_msgs::Transform trans_msg;
        tf::transformTFToMsg(trans,trans_msg);
        msg.transforms.push_back(trans_msg);
        msg.ids.push_back(graph_[i]->msg_id_); //msg_id is no more
    }
    msg.header.stamp = ros::Time::now();

    if (transform_pub_.getNumSubscribers() > 0)
        transform_pub_.publish(msg);
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}*/

void GraphManager::reset(){
    reset_request_ = true;
}

void GraphManager::deleteLastFrame(){
    if(graph_.size() <= 1) {
      ROS_INFO("Resetting, as the only node is to be deleted");
      reset_request_ = true;
      Q_EMIT deleteLastNode();
      return;
    }
    AISNavigation::PoseGraph3D::Vertex* v_to_del = optimizer_->vertex(optimizer_->vertices().size()-1);//last vertex
    AISNavigation::PoseGraph3D::Vertex *v1, *v2; //used in loop as temporaries
    AISNavigation::PoseGraph3D::EdgeSet::iterator edge_iter = optimizer_->edges().begin();
    for(;edge_iter != optimizer_->edges().end(); edge_iter++) {
        v1 = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->from());
        v2 = static_cast<AISNavigation::PoseGraph3D::Vertex*>((*edge_iter)->to());
        if(v1->id() == v_to_del->id() || v2->id() == v_to_del->id()) 
          optimizer_->removeEdge((*edge_iter));
    }

    optimizer_->removeVertex(v_to_del);
    graph_.erase(graph_.size()-1);
    optimizeGraph();//s.t. the effect of the removed edge transforms are removed to
    ROS_INFO("Removed most recent node");
    Q_EMIT setGUIInfo("Removed most recent node");
    Q_EMIT setGraphEdges(getGraphEdges());
    Q_EMIT deleteLastNode();
    //updateTransforms needs to be last, as it triggers a redraw
    Q_EMIT updateTransforms(getAllPosesAsMatrixList());
}

QList<QMatrix4x4>* GraphManager::getAllPosesAsMatrixList(){
    std::clock_t starttime=std::clock();
    ROS_DEBUG("Retrieving all transformations from optimizer");
    QList<QMatrix4x4>* result = new QList<QMatrix4x4>();
#if defined(QT_VERSION) && QT_VERSION >= 0x040700
    result->reserve(optimizer_->vertices().size());//only allocates the internal pointer array
#endif

    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        AIS::PoseGraph3D::Vertex* v = optimizer_->vertex(i);
        if(v){ 
            result->push_back(hogman2QMatrix(v->transformation)); 
        } else {
            ROS_ERROR("Nullpointer in graph at position %i!", i);
        }
    }
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
    return result;
}

// If QT Concurrent is available, run the saving in a seperate thread
//TODO: This doesn't work yet (because of method instead of function call?
#ifndef QT_NO_CONCURRENT
using namespace QtConcurrent;
void GraphManager::saveAllClouds(QString filename, bool compact){
  QFuture<void> f1 = run(this, &GraphManager::saveAllCloudsToFile, filename, compact);
  //f1.waitForFinished();
}
#else
// Otherwise just call it without threading
void GraphManager::saveAllClouds(QString filename, bool compact){
      saveAllCloudsToFile(filename, compact);
}
#endif

void GraphManager::saveAllCloudsToFile(QString filename, bool compact){
    std::clock_t starttime=std::clock();
    pointcloud_type aggregate_cloud; ///will hold all other clouds
    ROS_INFO("Saving all clouds to %s, this may take a while as they need to be transformed to a common coordinate frame.", qPrintable(filename));
    batch_processing_runs_ = true;
    tf::Transform  world2cam;
    //fill message
    //rgbdslam::CloudTransforms msg;
    QString message;
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        AIS::PoseGraph3D::Vertex* v = optimizer_->vertex(i);
        if(!v){ 
            ROS_ERROR("Nullpointer in graph at position %i!", i);
            continue;
        }
        tf::Transform transform = hogman2TF(v->transformation);
        tf::Transform cam2rgb;
        cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
        cam2rgb.setOrigin(tf::Point(0,-0.04,0));
        world2cam = cam2rgb*transform;
        transformAndAppendPointCloud (graph_[i]->pc_col, aggregate_cloud, world2cam, Max_Depth, compact);
        Q_EMIT setGUIStatus(message.sprintf("Saving to %s: Transformed Node %i/%i", qPrintable(filename), i, (int)optimizer_->vertices().size()));
    }
    aggregate_cloud.header.frame_id = "/openni_camera";
    if(filename.endsWith(".pcd", Qt::CaseInsensitive))
      pcl::io::savePCDFile(qPrintable(filename), aggregate_cloud, true); //Last arg is binary mode
    if(filename.endsWith(".ply", Qt::CaseInsensitive))
      pointCloud2MeshFile(filename, aggregate_cloud);
    Q_EMIT setGUIStatus(message.sprintf("Saved %d data points to %s", (int)aggregate_cloud.points.size(), qPrintable(filename)));
    ROS_INFO ("Saved %d data points to %s", (int)aggregate_cloud.points.size(), qPrintable(filename));

    if (aggregate_cloud_pub_.getNumSubscribers() > 0){ //if it should also be send out
        sensor_msgs::PointCloud2 cloudMessage_; //this will be send out in batch mode
        pcl::toROSMsg(aggregate_cloud,cloudMessage_);
        cloudMessage_.header.frame_id = "/openni_camera";
        cloudMessage_.header.stamp = ros::Time::now();
        aggregate_cloud_pub_.publish(cloudMessage_);
        ROS_INFO("Aggregate pointcloud sent");
    }
    batch_processing_runs_ = false;
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GraphManager::pointCloud2MeshFile(QString filename, pointcloud_type full_cloud){
  QFile file(filename);//file is closed on destruction
  if(!file.open(QIODevice::WriteOnly|QIODevice::Text)) return; //TODO: Errormessage
  QTextStream out(&file);
	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "element vertex " << (int)full_cloud.points.size() << "\n"; 
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "property uchar red\n";
	out << "property uchar green\n";
	out << "property uchar blue\n";
	out << "end_header\n";
  unsigned char r,g,b;
  float x, y, z ;
  for(unsigned int i = 0; i < full_cloud.points.size() ; i++){
    b = *(  (unsigned char*)&(full_cloud.points[i].rgb));
    g = *(1+(unsigned char*)&(full_cloud.points[i].rgb));
    r = *(2+(unsigned char*)&(full_cloud.points[i].rgb));
    x = full_cloud.points[i].x;
    y = full_cloud.points[i].y;
    z = full_cloud.points[i].z;
    out << qSetFieldWidth(8) << x << " " << y << " " << z << " ";
    out << qSetFieldWidth(3) << r << " " << g << " " << b << "\n";
  }
}
  

void GraphManager::sendAllClouds(){
    std::clock_t starttime=std::clock();
    ROS_INFO("Sending out all clouds");
    batch_processing_runs_ = true;
    tf::Transform  world2cam;
    //fill message
    //rgbdslam::CloudTransforms msg;
    for (unsigned int i = 0; i < optimizer_->vertices().size(); ++i) {
        AIS::PoseGraph3D::Vertex* v = optimizer_->vertex(i);
        if(!v){ 
            ROS_ERROR("Nullpointer in graph at position %i!", i);
            continue;
        }
        tf::Transform transform = hogman2TF(v->transformation);

        tf::Transform cam2rgb;
        cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
        cam2rgb.setOrigin(tf::Point(0,-0.04,0));

        world2cam = cam2rgb*transform;
        ros::Time time_of_transform = ros::Time::now();
        ROS_DEBUG("Sending out transform %i", i);
        br_.sendTransform(tf::StampedTransform(world2cam, time_of_transform,
                          "/openni_camera", "/batch_transform"));
        ROS_DEBUG("Sending out cloud %i", i);
        graph_[i]->publish("/batch_transform", time_of_transform);
    }

    batch_processing_runs_ = false;
    Q_EMIT sendFinished();
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", "function runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GraphManager::setMaxDepth(float max_depth){
	Max_Depth = max_depth;
	ROS_INFO("Max Depth set to: %f", max_depth);
}


//From: /opt/ros/unstable/stacks/perception_pcl/pcl/src/pcl/registration/transforms.hpp
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Apply an affine transform defined by an Eigen Transform
  * \param cloud_in the input point cloud
  * \param cloud_to_append_to the transformed cloud will be appended to this one
  * \param transform a tf::Transform stating the transformation of cloud_to_append_to relative to cloud_in
  * \note The density of the point cloud is lost, since density implies that the origin is the point of view
  * \note Can not(?) be used with cloud_in equal to cloud_to_append_to
  */
//template <typename PointT> void
//transformAndAppendPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_to_append_to,
//                              const tf::Transform transformation)
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, 
                                   pointcloud_type &cloud_to_append_to,
                                   const tf::Transform transformation, float Max_Depth, bool compact)
{
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transformation, eigen_transform);
    unsigned int cloud_to_append_to_original_size = cloud_to_append_to.size();
    if(cloud_to_append_to.points.size() ==0){
        cloud_to_append_to.header   = cloud_in.header;
        cloud_to_append_to.width    = 0;
        cloud_to_append_to.height   = 0;
        cloud_to_append_to.is_dense = false;
    }

    ROS_INFO("Max_Depth = %f", Max_Depth);
    ROS_INFO("cloud_to_append_to_original_size = %i", cloud_to_append_to_original_size);

    //Append all points untransformed
    cloud_to_append_to += cloud_in;

    Eigen::Matrix3f rot   = eigen_transform.block<3, 3> (0, 0);
    Eigen::Vector3f trans = eigen_transform.block<3, 1> (0, 3);
    point_type origin = point_type();
    origin.x = 0;
    origin.y = 0;
    origin.z = 0;
    int j = 0;
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    { 
     Eigen::Map<Eigen::Vector3f> p_in (&cloud_in.points[i].x, 3, 1);
     Eigen::Map<Eigen::Vector3f> p_out (&cloud_to_append_to.points[j+cloud_to_append_to_original_size].x, 3, 1);
     if(compact){
    	 cloud_to_append_to.points[j+cloud_to_append_to_original_size] = cloud_in.points[i];
     }
     //filter out points with a range greater than the given Parameter or do nothing if negativ
     if(Max_Depth >= 0){
		 if(pcl::squaredEuclideanDistance(cloud_in.points[i], origin) > Max_Depth*Max_Depth){
			p_out[0]= numeric_limits<float>::quiet_NaN();
			p_out[1]= numeric_limits<float>::quiet_NaN();
			p_out[2]= numeric_limits<float>::quiet_NaN();
			if(!compact)
				j++; 
			continue;
		  }
      }
      if (pcl_isnan (cloud_in.points[i].x) || pcl_isnan (cloud_in.points[i].y) || pcl_isnan (cloud_in.points[i].z)){
		  if(!compact)
			j++;
    	  continue;
      }
      p_out = rot * p_in + trans;
      j++;
    }
    if(compact){
		cloud_to_append_to.points.resize(j+cloud_to_append_to_original_size);
		cloud_to_append_to.width    = 1;
		cloud_to_append_to.height   = j+cloud_to_append_to_original_size;
	}
}


