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


/* This is the main widget of the application.
* It sets up some not yet useful menus and
* three qlabels in the layout of the central widget
* that can be used to show qimages via the slots
* setDepthImage and setVisualImage.
*/
#ifndef QTCV_H
#define QTCV_H

#include <QMainWindow>
#include <QGridLayout>
#include "glviewer.h"
#include "globaldefinitions.h"

class QAction;
class QActionGroup;
class QLabel;
class QMenu;

//TODO:
//Choice between Binary and ASCII outputfiles
//Buttons for start/stop
//GUI/Commandline options for switching on/off the individual visualizations

//!Constructs a QT GUI for easy control of RGBD-SLAM
/** Small GUI Class to visualize and control rgbdslam
* See Help->About for a short description */
class UserInterface: public QMainWindow
{
    Q_OBJECT

public:
    UserInterface();
    ///GraphManager needs this, to use the render-to-frame functionality
    GLViewer* getGLViewer();
Q_SIGNALS:
    ///User selected to reset the graph
    void reset(); 
    ///User selected to start or resume processing
    void togglePause();
    ///User wants the next frame to be processed
    void getOneFrame();
    ///User wants the last node to be removed from the graph
    void deleteLastFrame();
    void sendAllClouds(); ///< Signifies the sending of the whole model
    ///User wants the current world model to be saved to a pcd-file
    void saveAllClouds(QString filename, bool compact);
    void setMaxDepth(float max_depth);
     
public Q_SLOTS:
    void setVisualImage(QImage);
    void setFeatureFlowImage(QImage);
    void setDepthImage(QImage);
    void setTransformation(QString);
    void sendFinished(); ///< Call to display, that sending finished
    void addPointCloud(pointcloud_type const * pc, QMatrix4x4 transform);
    void updateTransforms(QList<QMatrix4x4>* transforms);
    void setGraphEdges(QList<QPair<int, int> >* list);
    void deleteLastNode();

private Q_SLOTS:
    void resetCmd();
    void sendAll();
    void setMax();
    void saveAll();
    void pause(bool);
    void about();
    void help();
    void lastTransformationMatrix();
    void setInfo(QString);
    void setInfo2(QString);
    void setStatus(QString);
    void getOneFrameCmd();
    void deleteLastFrameCmd();
    void set3DDisplay(bool is_on);
    void set2DStream(bool is_on);
    void toggleTriangulation();
private:
    void createActions();
    void createMenus();

    //QString *menuHelpText;
    QString *mouseHelpText;
    QString *infoText;
    QString *licenseText;
    QString transformationMatrix;
    QMenu *graphMenu;
    QMenu *actionMenu;
    QMenu *viewMenu;
    QMenu *helpMenu;
    QAction *newAct;
    QAction *saveAct;
    QAction *sendAct;
    QAction *pauseAct;
    QAction *maxAct;
    QAction *exitAct;
    QAction *aboutAct;
    QAction *helpAct;
    QAction *transfAct;
    QAction *oneFrameAct;
    QAction *delFrameAct;
    QAction *toggleGLViewerAct;
    QAction *toggleStreamAct;
    QAction *toggleTriangulationAct;
    QLabel *infoLabel;
    QLabel *infoLabel2;
    QLabel *tmpLabel;
    QLabel *visual_image_label;
    QLabel *feature_flow_image_label;
    QLabel *depth_image_label;
    QLabel *stats_image_label;
    //QLabel *transform_label;
    QGridLayout* gridlayout;
    GLViewer* glviewer;
    bool pause_on;
    QString filename;
};

#endif
