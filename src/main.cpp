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

/* registration main:
 * Create 
 * - a Qt Application 
 * - a ROS Node, 
 * - a ROS listener, listening to and processing kinect data
 * - Let them communicate internally via QT Signals
 */
#include "openni_listener.h"
#include "qtros.h"
#include <QApplication>
#include <QObject>
#include "qtcv.h"
#include <Eigen/Core>
#include "globaldefinitions.h"

//TODO:
//try ASIFT
//File dialog for Saving
//even better potential-edge-selection through flann
//subpixel accuracy for feature matches?
//Performance optimization
//Get all Parameters from Param server or command line
//Better separation of function, communication, parameters and gui


int main(int argc, char** argv)
{
  QApplication application(argc,argv);
  QtROS qtRos(argc, argv,global_rosnode_name); //Thread object, to run the ros event processing loop in parallel to the qt loop
  //If one thread receives a exit signal from the user, signal the other thread to quit too
  QObject::connect(&application, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
  QObject::connect(&qtRos, SIGNAL(rosQuits()), &application, SLOT(quit()));

  UserInterface window;
  window.show();
  GraphManager graph_mgr(qtRos.getNodeHandle(), window.getGLViewer());
  //Instantiate the kinect image listener
  OpenNIListener kinect_listener(qtRos.getNodeHandle(), &graph_mgr,
                                 global_topic_image_mono,  
                                 global_topic_image_depth, 
                                 global_topic_points,
                                 global_feature_extractor_type, //FAST is really fast but the Keypoints are not robust
                                 global_feature_detector_type);

  //COMMUNICATION BETWEEN COMPONENTS
  //Route every processed image to the GUI
  QObject::connect(&kinect_listener, SIGNAL(newVisualImage(QImage)), &window, SLOT(setVisualImage(QImage)));
  QObject::connect(&kinect_listener, SIGNAL(newFeatureFlowImage(QImage)), &window, SLOT(setFeatureFlowImage(QImage)));
  QObject::connect(&kinect_listener, SIGNAL(newDepthImage(QImage)), &window, SLOT(setDepthImage(QImage)));
  QObject::connect(&graph_mgr, SIGNAL(newTransformationMatrix(QString)), &window, SLOT(setTransformation(QString)));
  QObject::connect(&window, SIGNAL(reset()), &graph_mgr, SLOT(reset()));
  QObject::connect(&window, SIGNAL(togglePause()), &kinect_listener, SLOT(togglePause()));
  QObject::connect(&window, SIGNAL(getOneFrame()), &kinect_listener, SLOT(getOneFrame()));
  QObject::connect(&window, SIGNAL(deleteLastFrame()), &graph_mgr, SLOT(deleteLastFrame()));
  QObject::connect(&window, SIGNAL(sendAllClouds()), &graph_mgr, SLOT(sendAllClouds()));
  QObject::connect(&window, SIGNAL(saveAllClouds(QString)), &graph_mgr, SLOT(saveAllClouds(QString)));
  QObject::connect(&graph_mgr, SIGNAL(sendFinished()), &window, SLOT(sendFinished()));
  QObject::connect(&graph_mgr, SIGNAL(setGUIInfo(QString)), &window, SLOT(setInfo(QString)));
  QObject::connect(&graph_mgr, SIGNAL(setGUIStatus(QString)), &window, SLOT(setStatus(QString)));
  if(global_use_glwidget){
    QObject::connect(&graph_mgr, SIGNAL(setPointCloud(pointcloud_type const *, QMatrix4x4)), &window, SLOT(addPointCloud(pointcloud_type const *, QMatrix4x4)));//, Qt::DirectConnection);
    QObject::connect(&graph_mgr, SIGNAL(updateTransforms(QList<QMatrix4x4>*)), &window, SLOT(updateTransforms(QList<QMatrix4x4>*)));
    QObject::connect(&graph_mgr, SIGNAL(setGraphEdges(QList<QPair<int, int> >*)), &window, SLOT(setGraphEdges(QList<QPair<int, int> >*)));
  }
  QObject::connect(&kinect_listener, SIGNAL(setGUIInfo(QString)), &window, SLOT(setInfo(QString)));
  QObject::connect(&kinect_listener, SIGNAL(setGUIStatus(QString)), &window, SLOT(setStatus(QString)));
  QObject::connect(&graph_mgr, SIGNAL(setGUIInfo2(QString)), &window, SLOT(setInfo2(QString)));
  QObject::connect(&window, SIGNAL(setMaxDepth(float)), &graph_mgr, SLOT(setMaxDepth(float)));
  QObject::connect(&graph_mgr, SIGNAL(deleteLastNode()), &window, SLOT(deleteLastNode()));

  // Run main loop.
  qtRos.start();
#ifdef USE_ICP_BIN
  ROS_INFO("ICP activated via external binary");
#endif
#ifdef USE_ICP_CODE      
  ROS_INFO("ICP activated via linked library");
#endif
  application.exec();
  if(ros::ok()) {
    ros::shutdown();//If not yet done through the qt connection
  }
  ros::waitForShutdown(); //not sure if necessary. 
}


