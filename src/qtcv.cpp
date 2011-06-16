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
#include <QtGui>
#include <QPixmap>
#include <QFont>
#include <QIcon>
#include <QKeySequence>
#include "qtcv.h"
#include <limits>

///Constructs a QT GUI for easy control of RGBDSLAM
UserInterface::UserInterface() : filename("quicksave.pcd")
{
    QWidget *widget = new QWidget;
    setCentralWidget(widget);

    //QWidget *topFiller = new QWidget;
    //topFiller->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    infoText = new QString(tr(
                "<p><b>RGBDSLAM</b> uses visual features to identify corresponding 3D locations "
                "in RGBD data. The correspondences are used to reconstruct the camera motion. "
                "The SLAM-backend HOG-MAN is used to integrate the transformations between"
                "the RGBD-images and compute a globally consistent 6D trajectory.</p>"
                "<p></p>"));
    licenseText = new QString(tr(
                 "<p>RGBDSLAM is free software: you can redistribute it and/or modify"
                 "it under the terms of the GNU General Public License as published by"
                 "the Free Software Foundation, either version 3 of the License, or"
                 "(at your option) any later version.</p>"
                 "<p>RGBDSLAM is distributed in the hope that it will be useful,"
                 "but WITHOUT ANY WARRANTY; without even the implied warranty of"
                 "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the"
                 "GNU General Public License for more details.</p>"
                 "<p>You should have received a copy of the GNU General Public License"
                 "along with RGBDSLAM.  If not, refer to <a href=\"http://www.gnu.org/licenses/\">http://www.gnu.org/licenses</a>.</p>"));
    /*menuHelpText = new QString(tr(
                "<p><b>Menu Commands:</b>"
                "<ul><li><b>Graph</b></li><ul>"
                "<li><i>Save Model</i> saves the aggregated cloud to file.</li>"
                "<li><i>Save Model</i> saves the aggregated cloud to file.</li>"
                "<li><i>Send Model</i> sends the clouds and their transforms via ROS.</li>"
                "</ul><li><b>Processing</b></li><ul>"
                "<li><i>Process</i> toggles the processing.</li>"
                "<li><i>Reset</i> to clear the collected information.</li>"
                "<li><i>Capture One Frame</i> to Process one camera image.</li>"
                "<li><i>Delete Last Node</i> to clear the information about the last processed image.</li>" 
                "</ul><li><b>Information</b></li><ul>"
                "<li><i>Current Transformation Matrix</i> displays the transformation from the first to the last node.</li>"
                "<li><i>About RGBDSLAM</i> displays information about this software.</li>"
                "<li><i>Usage Help</i> displays this text</li></ul></ul><p>"));*/
    mouseHelpText = new QString(tr(
                "<p><b>3D Viewer Mouse Commands:</b>"
                "<ul><li><i>Left/Right button:</i> rotation.</li>"
                "    <li><i>Middle button:</i> shift.</li>"
                "    <li><i>Wheel:</i> zoom.</li>"
                "    <li><i>Double click on object:</i> set pivot to clicked point.</li>"
                "    <li><i>Double click on background:</i> reset view to camera pose.</li><ul></p>")); feature_flow_image_label = new QLabel(*mouseHelpText);
    feature_flow_image_label->setWordWrap(true);
    feature_flow_image_label->setMargin(10);
    feature_flow_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    visual_image_label = new QLabel("<i>Waiting for monochrome image...</i>");
    visual_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    visual_image_label->setAlignment(Qt::AlignCenter);
    //visual_image_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    depth_image_label = new QLabel(tr("<i>Waiting for depth image...</i>"));
    depth_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    depth_image_label->setAlignment(Qt::AlignCenter);
    //depth_image_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    //transform_label = new QLabel(tr("<i>Waiting for transformation matrix...</i>"));
    //transform_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    //transform_label->setAlignment(Qt::AlignCenter);
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer = new GLViewer(this);//displays the cloud in 3d

    //QFont typewriter_font;
    //typewriter_font.setStyleHint(QFont::TypeWriter);
    //transform_label->setFont(typewriter_font);

    //QWidget *bottomFiller = new QWidget;
    //bottomFiller->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    gridlayout = new QGridLayout;
    gridlayout->setMargin(5);
    //gridlayout->addWidget(infoLabel, 0,0);
    //gridlayout->addWidget(transform_label, 0,0,1,0); //with rowspan
    //gridlayout->addWidget(transform_label, 0,0);
    if(ParameterServer::instance()->get<bool>("use_glwidget")) gridlayout->addWidget(glviewer, 0,0,1,0);
    gridlayout->addWidget(visual_image_label, 1,0);
    gridlayout->addWidget(depth_image_label, 1,1);
    gridlayout->addWidget(feature_flow_image_label, 1,2);
    widget->setLayout(gridlayout);

    createActions();
    createMenus();

    tmpLabel = new QLabel();
    statusBar()->insertWidget(0,tmpLabel, 0);
    QString message = tr("Ready for RGBDSLAM");
    statusBar()->showMessage(message);

    infoLabel2 = new QLabel(tr("Waiting for motion information..."));
    infoLabel2->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    infoLabel2->setAlignment(Qt::AlignRight);
    statusBar()->addPermanentWidget(infoLabel2, 0);

    infoLabel = new QLabel(tr("<i>Press Enter or Space to Start</i>"));
    infoLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    infoLabel->setAlignment(Qt::AlignRight);
    statusBar()->addPermanentWidget(infoLabel, 0);

    setWindowTitle(tr("RGBDSLAM"));
    setMinimumSize(790, 590);
    //resize(1024, 800);
}

GLViewer* UserInterface::getGLViewer(){
    return this->glviewer;
}

void UserInterface::setFeatureFlowImage(QImage qimage){
  feature_flow_image_label->setAlignment(Qt::AlignCenter);
  feature_flow_image_label->setPixmap(QPixmap::fromImage(qimage));
}
void UserInterface::setVisualImage(QImage qimage){
  if(visual_image_label->isVisible())
      visual_image_label->setPixmap(QPixmap::fromImage(qimage));
}

void UserInterface::setDepthImage(QImage qimage){
  if(depth_image_label->isVisible())
      depth_image_label->setPixmap(QPixmap::fromImage(qimage));
}

void UserInterface::setTransformation(QString transf){
  transformationMatrix = transf;
}

void UserInterface::resetCmd() {
    Q_EMIT reset();
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer->reset();
    QString message = tr("Graph Reset");
    statusBar()->showMessage(message);
    infoLabel->setText("A fresh new graph is waiting");
}

void UserInterface::setStatus(QString message){
    statusBar()->showMessage(message);
}
void UserInterface::setInfo2(QString message){
    infoLabel2->setText(message);
}
void UserInterface::setInfo(QString message){
    infoLabel->setText(message);
}

void UserInterface::quickSaveAll() {
    Q_EMIT saveAllClouds(filename);
    QString message = tr("Saving Whole Model to ");
    message.append(filename);
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}
void UserInterface::saveAll() {
    filename = QFileDialog::getSaveFileName(this, "Save Point CLoud to File", filename, tr("PCD (*.pcd);;PLY (*ply)"));
    Q_EMIT saveAllClouds(filename);
    QString message = tr("Saving Whole Model");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}
void UserInterface::saveIndividual() {
    QString tmpfilename(filename);
    tmpfilename.remove(".pcd", Qt::CaseInsensitive);
    tmpfilename.remove(".ply", Qt::CaseInsensitive);
    filename = QFileDialog::getSaveFileName(this, "Save point cloud to one file per node", tmpfilename, tr("PCD (*.pcd)"));
    Q_EMIT saveIndividualClouds(filename);
    QString message = tr("Saving Model Node-Wise");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void UserInterface::sendAll() {
    Q_EMIT sendAllClouds();
    QString message = tr("Sending Whole Model");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}
void UserInterface::setMax() {
    bool ok;
    double value = QInputDialog::getDouble(this, tr("Set Max Depth"),
                                            tr("Enter Max Depth [in cm]\n (negativ if no Filtering is required):"), -100.00, -10000000, 10000000, 2, &ok);
    if(ok){
    	Q_EMIT setMaxDepth(value/100.0);
    }
}
void UserInterface::sendFinished() {
    
    QString message = tr("Finished Sending");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void UserInterface::getOneFrameCmd() {
    Q_EMIT getOneFrame();
    QString message = tr("Getting a single frame");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}
void UserInterface::deleteLastFrameCmd() {
    Q_EMIT deleteLastFrame();
    QString message = tr("Deleting the last node from the graph");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}
void UserInterface::bagRecording(bool pause_on) {
    Q_EMIT toggleBagRecording();
    if(pause_on) {
        QString message = tr("Recording Bagfile.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    } else {
        QString message = tr("Stopped Recording.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    }
}
void UserInterface::pause(bool pause_on) {
    Q_EMIT togglePause();
    if(pause_on) {
        QString message = tr("Processing.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    } else {
        QString message = tr("Stopped processing.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    }
}

void UserInterface::help() {
    QMessageBox::about(this, tr("Help Menu"), /**menuHelpText +*/ *mouseHelpText );
}
void UserInterface::about() {
    QMessageBox::about(this, tr("About RGBDSLAM"), *infoText + *licenseText);
}
void UserInterface::lastTransformationMatrix() {
    QMessageBox::information(this, tr("Current Transformation Matrix"), transformationMatrix);
}

void UserInterface::toggleTriangulation() {
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer->toggleTriangulation();
}

void UserInterface::set2DStream(bool is_on) {
    if(is_on){ 
        visual_image_label->show();
        depth_image_label->show(); 
        feature_flow_image_label->show(); 
    } else { 
        visual_image_label->hide(); 
        depth_image_label->hide(); 
        feature_flow_image_label->hide(); 
    } 
}

void UserInterface::set3DDisplay(bool is_on) {
    if(!ParameterServer::instance()->get<bool>("use_glwidget")) return;
    if(is_on){ glviewer->show(); } 
    else { glviewer->hide(); } 
}


void UserInterface::createActions() {
    newAct = new QAction(tr("&Reset"), this);
    newAct->setShortcut(QString("Ctrl+R"));
    newAct->setStatusTip(tr("Reset the graph, clear all data collected"));
    newAct->setIcon(QIcon::fromTheme("document-new"));//doesn't work (for gnome?)
    connect(newAct, SIGNAL(triggered()), this, SLOT(resetCmd()));

    quickSaveAct = new QAction(tr("&Save"), this);
    quickSaveAct->setShortcuts(QKeySequence::Save);
    quickSaveAct->setStatusTip(tr("Save all stored point clouds with common coordinate frame to a pcd file"));
    connect(quickSaveAct, SIGNAL(triggered()), this, SLOT(quickSaveAll()));

    saveAct = new QAction(tr("&Save as..."), this);
    saveAct->setShortcuts(QKeySequence::SaveAs);
    saveAct->setStatusTip(tr("Save all stored point clouds with common coordinate frame"));
    connect(saveAct, SIGNAL(triggered()), this, SLOT(saveAll()));

    saveIndiAct = new QAction(tr("&Save Node-Wise..."), this);
    saveIndiAct->setShortcut(QString("Ctrl+N"));
    saveIndiAct->setStatusTip(tr("Save stored point clouds in individual files"));
    connect(saveIndiAct, SIGNAL(triggered()), this, SLOT(saveIndividual()));

    sendAct = new QAction(tr("&Send Model"), this);
    sendAct->setShortcut(QString("Ctrl+M"));
    sendAct->setStatusTip(tr("Send out all stored point clouds with corrected transform"));
    connect(sendAct, SIGNAL(triggered()), this, SLOT(sendAll()));

    pauseAct = new QAction(tr("&Process"), this);
    pauseAct->setShortcut(QString(" "));
    pauseAct->setCheckable(true);
    pauseAct->setChecked(!ParameterServer::instance()->get<bool>("start_paused"));
    pauseAct->setStatusTip(tr("Start/stop processing of frames"));
    pauseAct->setIcon(QIcon::fromTheme("media-playback-start"));//doesn't work (for gnome?)
    connect(pauseAct, SIGNAL(toggled(bool)), this, SLOT(pause(bool)));

    bagRecordingAct = new QAction(tr("&Bagfile Recording"), this);
    bagRecordingAct->setShortcut(QString("R"));
    bagRecordingAct->setCheckable(true);
    bagRecordingAct->setChecked(false);
    bagRecordingAct->setStatusTip(tr("Start/stop recording of frames to bagfile"));
    bagRecordingAct->setIcon(QIcon::fromTheme("media-playback-start"));//doesn't work (for gnome?)
    connect(bagRecordingAct, SIGNAL(toggled(bool)), this, SLOT(bagRecording(bool)));

    maxAct = new QAction(tr("Set Maximum &Depth"), this);
    maxAct->setShortcut(QString("Ctrl+D"));
    maxAct->setStatusTip(tr("Set the Maximum Depth a Point can have (negativ if no Filtering is required)"));
    connect(maxAct, SIGNAL(triggered()), this, SLOT(setMax()));

    oneFrameAct = new QAction(tr("Capture One& Frame"), this);
    oneFrameAct->setShortcuts(QKeySequence::InsertParagraphSeparator);
    oneFrameAct->setStatusTip(tr("Process one frame only"));
    connect(oneFrameAct, SIGNAL(triggered()), this, SLOT(getOneFrameCmd()));

    delFrameAct = new QAction(tr("&Delete Last Node"), this);
    delFrameAct->setShortcut(QString("Backspace"));
    delFrameAct->setStatusTip(tr("Remove last node from graph"));
    connect(delFrameAct, SIGNAL(triggered()), this, SLOT(deleteLastFrameCmd()));

    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    exitAct->setStatusTip(tr("Exit the application"));
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    helpAct = new QAction(tr("&Usage Help"), this);
    helpAct->setShortcuts(QKeySequence::HelpContents);
    helpAct->setStatusTip(tr("Show usage information"));
    connect(helpAct, SIGNAL(triggered()), this, SLOT(help()));

    aboutAct = new QAction(tr("&About RGBDSLAM"), this);
    aboutAct->setShortcut(QString("Ctrl+A"));
    aboutAct->setStatusTip(tr("Show information about RGBDSLAM"));
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

    transfAct = new QAction(tr("Current Transformation &Matrix"), this);
    transfAct->setShortcut(QString("Ctrl+T"));
    transfAct->setStatusTip(tr("Show the Current Transformation Matrix"));
    connect(transfAct, SIGNAL(triggered()), this, SLOT(lastTransformationMatrix()));

    toggleGLViewerAct = new QAction(tr("Toggle &3D Display"), this);
    toggleGLViewerAct->setShortcut(QString("3"));
    toggleGLViewerAct->setCheckable(true);
    toggleGLViewerAct->setChecked(true);
    toggleGLViewerAct->setStatusTip(tr("Turn off the OpenGL Display of the Accumulated PointClouds"));
    connect(toggleGLViewerAct, SIGNAL(toggled(bool)), this, SLOT(set3DDisplay(bool)));

    toggleFollowAct = new QAction(tr("&Follow Camera"), this);
    toggleFollowAct->setShortcut(QString("F"));
    toggleFollowAct->setCheckable(true);
    toggleFollowAct->setChecked(true);
    toggleFollowAct->setStatusTip(tr("Always use viewpoint of last frame (except zoom)"));
    connect(toggleFollowAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleFollowMode(bool)));

    toggleTriangulationAct = new QAction(tr("&Toggle Rendering"), this);
    toggleTriangulationAct->setShortcut(QString("T"));
    toggleTriangulationAct->setStatusTip(tr("Switch between surface, wireframe and point cloud"));
    connect(toggleTriangulationAct, SIGNAL(triggered(bool)), this, SLOT(toggleTriangulation()));

    toggleStreamAct = new QAction(tr("Toggle &2D Stream"), this);
    toggleStreamAct->setShortcut(QString("2"));
    toggleStreamAct->setCheckable(true);
    toggleStreamAct->setChecked(true);
    toggleStreamAct->setStatusTip(tr("Turn off the Image Stream"));
    connect(toggleStreamAct, SIGNAL(toggled(bool)), this, SLOT(set2DStream(bool)));
}

void UserInterface::createMenus() {
    graphMenu = menuBar()->addMenu(tr("&Graph"));
    graphMenu->addAction(quickSaveAct);
    graphMenu->addAction(saveAct);
    graphMenu->addAction(saveIndiAct);
    graphMenu->addAction(sendAct);
    graphMenu->addSeparator();
    graphMenu->addAction(exitAct);

    actionMenu = menuBar()->addMenu(tr("&Processing"));
    actionMenu->addAction(newAct);
    actionMenu->addAction(pauseAct);
    actionMenu->addAction(bagRecordingAct);
    actionMenu->addAction(maxAct);
    actionMenu->addAction(oneFrameAct);
    actionMenu->addAction(delFrameAct);

    viewMenu = menuBar()->addMenu(tr("&View"));
    viewMenu->addAction(toggleGLViewerAct);
    viewMenu->addAction(toggleStreamAct);
    viewMenu->addAction(toggleTriangulationAct);
    viewMenu->addAction(toggleFollowAct);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(transfAct);
    helpMenu->addAction(helpAct);
    helpMenu->addAction(aboutAct);
}
void UserInterface::addPointCloud(pointcloud_type const * pc, QMatrix4x4 transform){
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer->addPointCloud(pc, transform);
}
void UserInterface::deleteLastNode(){
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer->deleteLastNode();
}
void UserInterface::setGraphEdges(QList<QPair<int, int> >* list){
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer->setEdges(list);
}
void UserInterface::updateTransforms(QList<QMatrix4x4>* transforms){
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer->updateTransforms(transforms);
}
