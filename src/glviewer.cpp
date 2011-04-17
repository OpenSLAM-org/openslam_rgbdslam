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


#include "ros/ros.h"
#include <QtGui>
#include <QtOpenGL>
#include <QThread>
#include <GL/gl.h>
#include <cmath>

#include "glviewer.h"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

const double pi= 3.14159265358979323846;

GLViewer::GLViewer(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
      xRot(180*16.0),
      yRot(0),
      zRot(0),
      xTra(0),
      yTra(0),
      zTra(-120),
      polygon_mode(GL_FILL),
      cloud_list_indices(),
      edge_list_(NULL),
      cloud_matrices(new QList<QMatrix4x4>()){
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding); //can make good use of more space
    viewpoint_tf_.setToIdentity();
}

GLViewer::~GLViewer() { }

QSize GLViewer::minimumSizeHint() const {
    return QSize(400, 400);
}

QSize GLViewer::sizeHint() const {
    return QSize(640, 480);
}

static void qNormalizeAngle(int &angle) {
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

void GLViewer::setXRotation(int angle) { 
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        updateGL();
    }
}


void GLViewer::setYRotation(int angle) {
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        updateGL();
    }
}

void GLViewer::setZRotation(int angle) {
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        updateGL();
    }
}

void GLViewer::initializeGL() {
    //glClearColor(0,0,0,0); 
    glEnable (GL_BLEND); 
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    //glShadeModel(GL_SMOOTH);
    //glEnable(GL_LIGHTING);
    //glEnable(GL_LIGHT0);
    //glEnable(GL_MULTISAMPLE);
    //gluPerspective(99.0/180.0*pi, 1.00, 0.01, 1e9); //1.38 = tan(57/2°)/tan(43/2°)
    ////gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
    //static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    //glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}
//assumes gl mode for triangles
//TODO:Performance, Make inline
inline
bool GLViewer::drawTriangle(const point_type& p1, const point_type& p2, const point_type& p3){
    unsigned char b,g,r;
    b = *(  (unsigned char*)(&p1.rgb));
    g = *(1+(unsigned char*)(&p1.rgb));
    r = *(2+(unsigned char*)(&p1.rgb));
    glColor3ub(r,g,b); //glColor3f(1.0,1.0,1.0);
    glColor3ub(255,0,0); 
    glVertex3f(p1.x, p1.y, p1.z);

    b = *(  (unsigned char*)(&p2.rgb));
    g = *(1+(unsigned char*)(&p2.rgb));
    r = *(2+(unsigned char*)(&p2.rgb));
    glColor3ub(r,g,b); //glColor3f(1.0,1.0,1.0);
    glColor3ub(0,255,0); 
    glVertex3f(p2.x, p2.y, p2.z);

    b = *(  (unsigned char*)(&p3.rgb));
    g = *(1+(unsigned char*)(&p3.rgb));
    r = *(2+(unsigned char*)(&p3.rgb));
    //glColor3ub(r,g,b); //glColor3f(1.0,1.0,1.0);
    glColor3ub(0,0,255); 
    glVertex3f(p3.x, p3.y, p3.z);
    return true;
}

void GLViewer::drawAxis(float scale){
    glBegin(GL_LINES);
    glColor4f (1, 0, 0, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (1, 0, 0, 0.0);
    glVertex3f(scale, 0, 0);
    glColor4f (0, 1, 0, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0, 1, 0, 0.0);
    glVertex3f(0, scale, 0);
    glColor4f (0, 0, 1, 1.0);
    glVertex3f(0, 0, 0);
    glColor4f (0, 0, 1, 0.0);
    glVertex3f(0, 0, scale);
    glEnd();
}

void GLViewer::paintGL() {
    if(!this->isVisible()) return;
    //ROS_INFO("This is paint-thread %d", (unsigned int)QThread::currentThreadId());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    //Camera transformation
    glTranslatef(xTra, yTra, zTra);
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);
    glMultMatrixd(static_cast<GLdouble*>( viewpoint_tf_.data() ));//works as long as qreal and GLdouble are typedefs to double (might depend on hardware)
    drawAxis(0.5);//Show origin as big axis
    drawEdges();
    ROS_DEBUG("Drawing %i PointClouds", cloud_list_indices.size());
    for(int i = 0; i<cloud_list_indices.size() && i<cloud_matrices->size(); i++){
        //GLdouble* entry = static_cast<GLdouble*>( cloud_matrices[i].data() );
        //for(int j = 0; j < 16; j++, entry++){
        //    ROS_INFO("Matrix[%d]: %f", j, *entry);
        //}
        glPushMatrix();
        glMultMatrixd(static_cast<GLdouble*>( (*cloud_matrices)[i].data() ));//works as long as qreal and GLdouble are typedefs to double (might depend on hardware)
        glCallList(cloud_list_indices[i]);
        drawAxis(0.15);
        glPopMatrix();
    }
}
// Shape For Debuging 
void GLViewer::drawCoil() {
    const float nbSteps = 200.0;
    glBegin(GL_QUAD_STRIP);
    for (int i=0; i<nbSteps; ++i) {
        const float ratio = i/nbSteps;
        const float angle = 21.0*ratio;
        const float c = cos(angle);
        const float s = sin(angle);
        const float r1 = 1.0 - 0.8f*ratio;
        const float r2 = 0.8f - 0.8f*ratio;
        const float alt = ratio - 0.5f;
        const float nor = 0.5f;
        const float up = sqrt(1.0-nor*nor);
        glColor3f(1.0-ratio, 0.2f , ratio);
        glNormal3f(nor*c, up, nor*s);
        glVertex3f(r1*c, alt, r1*s+2);
        glVertex3f(r2*c, alt+0.05f, r2*s+2); }
    glEnd();
}

void GLViewer::resizeGL(int width, int height)
{
    //int side = qMin(width, height);
    glViewport(0, 0, width, height);
    //glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
//#ifdef QT_OPENGL_ES_1
//    glOrthof(-0.5, +0.5, -0.5, +0.5, 1.0, 15.0);
//#else
//    glOrtho(-0.5, +0.5, -0.5, +0.5, 1.0, 15.0);
//#endif
    //gluPerspective(57.0/180.0*pi, 1.38, 0.01, 1e9); //1.38 = tan(57/2°)/tan(43/2°) as kinect has viewing angles 57 and 43
    float ratio = (float)width / (float) height;
    gluPerspective(pi/4.0, ratio, 0.1, 1e4); 
    glMatrixMode(GL_MODELVIEW);
}

void GLViewer::mouseDoubleClickEvent(QMouseEvent *event) {
    xRot=180*16.0;
    yRot=0;
    zRot=0;
    xTra=0;
    yTra=0;
    zTra=-120;
    if(cloud_matrices->size()>0){
      if (event->buttons() & Qt::LeftButton) {
        int id = cloud_matrices->size()-1;
        setViewPoint((*cloud_matrices)[id]);
      } else if (event->buttons() & Qt::RightButton) {
        int id = 0;
        setViewPoint((*cloud_matrices)[id]);
      } else if (event->buttons() & Qt::MidButton) { 
        viewpoint_tf_.setToIdentity();
      }
      setClickedPosition(event->x(), event->y());
    }
    updateGL();
}
void GLViewer::mousePressEvent(QMouseEvent *event) {
    lastPos = event->pos();
}

void GLViewer::wheelEvent(QWheelEvent *event) {
    zTra += ((float)event->delta())/25.0; 
    updateGL();
}
void GLViewer::mouseMoveEvent(QMouseEvent *event) {//TODO: consolidate setRotation methods
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot - 8 * dy);
        setYRotation(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot - 8 * dy);
        setZRotation(zRot + 8 * dx);
    } else if (event->buttons() & Qt::MidButton) {
        xTra += dx/200.0;
        yTra -= dy/200.0;
        updateGL();
    }
    lastPos = event->pos();
}

void GLViewer::updateTransforms(QList<QMatrix4x4>* transforms){
    ROS_WARN_COND(transforms->size() < cloud_matrices->size(), "Got less transforms than before!");
    // This doesn't deep copy, but should work, as qlist maintains a reference count 
    delete cloud_matrices;
    cloud_matrices = transforms; 
    ROS_DEBUG("New Cloud matrices size: %d", cloud_matrices->size());
    updateGL();
}

void GLViewer::addPointCloud(pointcloud_type const * pc, QMatrix4x4 transform){
    ROS_DEBUG("pc pointer in addPointCloud: %p (this is %p in thread %d)", pc, this, (unsigned int)QThread::currentThreadId());
    pointCloud2GLStrip(pc);
    cloud_matrices->push_back(transform); //keep for later
    updateGL();
}

void GLViewer::pointCloud2GLStrip(pointcloud_type const * pc){
    std::clock_t starttime=std::clock();
    ROS_DEBUG("Making GL list from point-cloud pointer %p in thread %d", pc, (unsigned int)QThread::currentThreadId());
    GLuint cloud_list_index = glGenLists(1);
    if(!cloud_list_index) {
        ROS_ERROR("No display list could be created");
        return;
    }
    
    glNewList(cloud_list_index, GL_COMPILE);
    cloud_list_indices.push_back(cloud_list_index);
    //ROS_INFO_COND(!pc->is_dense, "Expected dense cloud for opengl drawing");
    const point_type origin = {{{ 0.0 }}, {{0.0}}};
    float depth;
    bool strip_on = false, flip = false; //if flip is true, first the lower then the upper is inserted
    unsigned int w=pc->width, h=pc->height;
    unsigned char b,g,r;
    for(unsigned int y = 0; y < h-1; y++){ //go through every point and make two triangles 
        for(unsigned int x = 0; x < w-1; x++){//for it and its neighbours right and/or down
            using namespace pcl;
            if(!strip_on){ //Generate vertices for new triangle
                const point_type* ll = &pc->points[(x)+(y+1)*w]; //one down (lower left corner)
                if(!hasValidXYZ(*ll)) continue; // both new triangles in this step would use this point
                const point_type* ur = &pc->points[(x+1)+y*w]; //one right (upper right corner)
                if(!hasValidXYZ(*ur)) continue; // both new triangles in this step would use this point
          
                const point_type* ul = &pc->points[x+y*w]; //current point (upper right)
                if(hasValidXYZ(*ul)){ //ul, ur, ll all valid
                  depth = squaredEuclideanDistance(*ul,origin);
                  if (squaredEuclideanDistance(*ul,*ll)/depth <= global_squared_meshing_threshold  and //threshold is defined in globaldefinitions.h
                      squaredEuclideanDistance(*ul,*ll)/depth <= global_squared_meshing_threshold  and
                      squaredEuclideanDistance(*ur,*ll)/depth <= global_squared_meshing_threshold){
                    glBegin(GL_TRIANGLE_STRIP);
                    strip_on = true;
                    flip = false; //correct order, upper first
                    //Prepare the first two vertices of a triangle strip
                    //drawTriangle(*ul, *ll, *ur);
                    b = *(  (unsigned char*)(&ul->rgb));
                    g = *(1+(unsigned char*)(&ul->rgb));
                    r = *(2+(unsigned char*)(&ul->rgb));
                    glColor3ub(r,g,b);

                    //glColor3ub(255,0,0);
                    glVertex3f(ul->x, ul->y, ul->z);
                  }
                } 
                if(!strip_on) { //can't use the point on the upper left, should I still init a triangle?
                  const point_type* lr = &pc->points[(x+1)+(y+1)*w]; //one right-down (lower right)
                  if(!hasValidXYZ(*lr)) {
                    //if this is not valid, there is no way to make a new triangle in the next step
                    //and one could have been drawn starting in this step, only if ul had been valid
                    x++;
                    continue;
                  } else { //at least one can be started at the lower left
                    depth = squaredEuclideanDistance(*ur,origin);
                    if (squaredEuclideanDistance(*ur,*ll)/depth <= global_squared_meshing_threshold  and //threshold is defined in globaldefinitions.h
                        squaredEuclideanDistance(*lr,*ll)/depth <= global_squared_meshing_threshold  and
                        squaredEuclideanDistance(*ur,*lr)/depth <= global_squared_meshing_threshold){
                      glBegin(GL_TRIANGLE_STRIP);
                      strip_on = true;
                      flip = true; //but the lower has to be inserted first, for correct order
                    }
                  }
                }
                if(strip_on) { //Be this the second or the first vertex, insert it
                  b = *(  (unsigned char*)(&ll->rgb));
                  g = *(1+(unsigned char*)(&ll->rgb));
                  r = *(2+(unsigned char*)(&ll->rgb));
                  glColor3ub(r,g,b);

                  //glColor3ub(0,255,0);
                  glVertex3f(ll->x, ll->y, ll->z);
                }
                continue; //not relevant but demonstrate that nothing else is done in this iteration
            } // end strip was off
            else 
            {//neighbours to the left and left down are already set
              const point_type* ul;
              if(flip){ ul = &pc->points[(x)+(y+1)*w]; } //one down (lower left corner) 
              else { ul = &pc->points[x+y*w]; } //current point (upper right)
              if(hasValidXYZ(*ul)){ //Neighbours to the left are prepared
                depth = squaredEuclideanDistance(*ul,origin);
                if (squaredEuclideanDistance(*ul,*(ul-1))/depth > global_squared_meshing_threshold){
                  glEnd();
                  strip_on = false;
                  continue;
                }
                //Complete the triangle with both leftern neighbors
                //drawTriangle(*ul, *ll, *ur);
                b = *(  (unsigned char*)(&ul->rgb));
                g = *(1+(unsigned char*)(&ul->rgb));
                r = *(2+(unsigned char*)(&ul->rgb));
                glColor3ub(r,g,b);

                //glColor3ub(255,0,0);
                glVertex3f(ul->x, ul->y, ul->z);
              } else {
                glEnd();
                strip_on = false;
                continue; //TODO: Could restart with next point instead
              }
              //The following point connects one to the left with the other on this horizontal level
              const point_type* ll;
              if(flip){ ll = &pc->points[x+y*w]; } //current point (upper right)
              else { ll = &pc->points[(x)+(y+1)*w]; } //one down (lower left corner) 
              if(hasValidXYZ(*ll)){ 
                depth = squaredEuclideanDistance(*ll,origin);
                if (squaredEuclideanDistance(*ul,*ll)/depth > global_squared_meshing_threshold or
                    squaredEuclideanDistance(*ul,*(ul-1))/depth > global_squared_meshing_threshold or
                    squaredEuclideanDistance(*ll,*(ll-1))/depth > global_squared_meshing_threshold){
                  glEnd();
                  strip_on = false;
                  continue;
                }
                b = *(  (unsigned char*)(&ll->rgb));
                g = *(1+(unsigned char*)(&ll->rgb));
                r = *(2+(unsigned char*)(&ll->rgb));
                glColor3ub(r,g,b);

                glVertex3f(ll->x, ll->y, ll->z);
              } else {
                glEnd();
                strip_on = false;
                continue;
              }
            }//completed triangles if strip is running
        }
        if(strip_on) glEnd();
        strip_on = false;
    }
    ROS_DEBUG("Compiled pointcloud into list %i",  cloud_list_index);
    glEndList();
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", __FUNCTION__ << " runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GLViewer::deleteLastNode(){
  if(cloud_list_indices.size() <= 1){
    this->reset();
    return;
  }
	GLuint nodeId = cloud_list_indices.back();
	cloud_list_indices.pop_back();
	glDeleteLists(nodeId,1);
}

void GLViewer::pointCloud2GLList(pointcloud_type const * pc){
    std::clock_t starttime=std::clock();
    ROS_DEBUG("Making GL list from point-cloud pointer %p in thread %d", pc, (unsigned int)QThread::currentThreadId());
    GLuint cloud_list_index = glGenLists(1);
    if(!cloud_list_index) {
        ROS_ERROR("No display list could be created");
        return;
    }
    cloud_list_indices.push_back(cloud_list_index);
    glNewList(cloud_list_index, GL_COMPILE);
    glBegin(GL_TRIANGLE_STRIP);
    //ROS_INFO_COND(!pc->is_dense, "Expected dense cloud for opengl drawing");
    const point_type origin = {{{ 0.0 }}, {{0.0}}};
    float depth;
    unsigned int w=pc->width, h=pc->height;
    for(unsigned int x = 0; x < w-1; x++){
        for(unsigned int y = 0; y < h-1; y++){
            using namespace pcl;

            const point_type* pi = &pc->points[x+y*w]; //current point

            if(!(hasValidXYZ(*pi))) continue;
            depth = squaredEuclideanDistance(*pi,origin);

            const point_type* pl = &pc->points[(x+1)+(y+1)*w]; //one right-down
            if(!(hasValidXYZ(*pl)) or squaredEuclideanDistance(*pi,*pl)/depth > global_squared_meshing_threshold)  //threshold is defined in globaldefinitions.h
              continue;

            const point_type* pj = &pc->points[(x+1)+y*w]; //one right
            if(hasValidXYZ(*pj)
               and squaredEuclideanDistance(*pi,*pj)/depth <= global_squared_meshing_threshold  //threshold is defined in globaldefinitions.h
               and squaredEuclideanDistance(*pj,*pl)/depth <= global_squared_meshing_threshold){
              drawTriangle(*pi, *pj, *pl);
            }
            const point_type* pk = &pc->points[(x)+(y+1)*w]; //one down
            
            
            
            if(hasValidXYZ(*pk)
               and squaredEuclideanDistance(*pi,*pk)/depth <= global_squared_meshing_threshold  //threshold is defined in globaldefinitions.h
               and squaredEuclideanDistance(*pk,*pl)/depth <= global_squared_meshing_threshold){
              drawTriangle(*pi, *pk, *pl);
            }
        }
    }
    glEnd();
    ROS_DEBUG("Compiled pointcloud into list %i",  cloud_list_index);
    glEndList();
    ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > global_min_time_reported, "timings", __FUNCTION__ << " runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec"); 
}

void GLViewer::reset(){
    glDeleteLists(1,cloud_list_indices.back());
    cloud_list_indices.clear();
    cloud_matrices->clear();
    if(edge_list_) {
      delete edge_list_;
      edge_list_ = NULL;
    }
    updateGL();
}
QImage GLViewer::renderList(QMatrix4x4 transform, int list_id){
    return QImage();
}

void GLViewer::setEdges(QList<QPair<int, int> >* edge_list){
  if(edge_list_) delete edge_list_;
  edge_list_ = edge_list;
}

void GLViewer::drawEdges(){
  if(edge_list_ == NULL) return;
  glBegin(GL_LINES);
  glColor4f(1,1,1,0.4);
  for(int i = 0; i < edge_list_->size(); i++){
    int id = (*edge_list_)[i].first;
    if(cloud_matrices->size() > id){//only happens in weird circumstances
      float x = (*cloud_matrices)[id](0,3);
      float y = (*cloud_matrices)[id](1,3);
      float z = (*cloud_matrices)[id](2,3);
      glVertex3f(x,y,z);
      id = (*edge_list_)[i].second;
      x = (*cloud_matrices)[id](0,3);
      y = (*cloud_matrices)[id](1,3);
      z = (*cloud_matrices)[id](2,3);
      glVertex3f(x,y,z);
    }
  }
  glEnd();
}


void GLViewer::setViewPoint(QMatrix4x4 new_vp){
    ///Moving the camera is inverse to moving the points to draw
    viewpoint_tf_ = new_vp.inverted();
}

void GLViewer::setClickedPosition(int x, int y) {
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posX, posY, posZ;

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );

    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    if(winZ != 1){ //default value, where nothing was rendered
      gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
      ROS_INFO_STREAM((float)winZ << ", " << posX << "," << posY << "," << posZ);
      viewpoint_tf_(0,3) = -posX;
      viewpoint_tf_(1,3) = -posY;
      viewpoint_tf_(2,3) = -posZ;
    }
}

void GLViewer::toggleTriangulation() {
    ROS_INFO("Toggling Triangulation");
    if(polygon_mode == GL_FILL){ // Turn on Pointcloud mode
        polygon_mode = GL_POINT;
    } else if(polygon_mode == GL_POINT){ // Turn on Wireframe mode
        polygon_mode = GL_LINE;
    } else { // Turn on Surface mode
        polygon_mode = GL_FILL;
    }
    glPolygonMode(GL_FRONT_AND_BACK, polygon_mode);
    updateGL();
}

