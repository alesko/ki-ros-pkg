/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexander Skoglund, Karolinska Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Karolinska Institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>
#include <GL/freeglut.h>
#include "image_transport/image_transport.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"


namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
int ghighInt;
void switch_callback_h( int position ){
  ghighInt = position;
}

int gblur;
void swith_callback( int position ){
  gblur = position;
}

int galpha;
void swith_callback_alpha( int position ){
  galpha = position;
}

GLvoid OnReshape(GLint w, GLint h)
  {
    glViewport(0, 0, w, h);
  }

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_r_sub_;
  image_transport::Subscriber image_l_sub_;
  image_transport::Publisher image_r_pub_;
  image_transport::Publisher image_l_pub_;
  
  int maxstretch_; // = 400;
  int trackpos_; // = maxstretch;
  int  blurpos_;
  int maxblur_;
  int  alphapos_;
  int maxalpha_;
  float inc_; // = (((float)pp_size*2)/maxstretch)/2;
  int percent_; //1 = (long) percent;
  
  //frame = cvQueryFrame( c_capture );
  IplImage* warp_; // = cvCreateImage (cvGetSize(frame), IPL_DEPTH_8U, 3);
  IplImage* frame_; // = GetThresholdedImage(warp);
  cv::Mat background_; // = GetThresholdedImage(warp);
  bool first_time_;

  //cvBridge bridge_;
  //sensor_msgs::CvBridge bridge_;
  //IplImage *cv_input_;

  CvPoint2D32f frameTri_[3], warpTri_[3];		
  float l_;// = 0.00;
  float r_;// = 0.00;
  
  int pp_size_; // = 80;// percentage size of participant compared to mannequin x2


  int viewport_width_ ;
  int viewport_height_ ;

  cv::Mat out_mat_;
  cv::Mat stereoDisplay_;
  ros::Rate publish_rate_;

public:

  ImageConverter()
    : it_(nh_),publish_rate_(60)
  {
    image_l_pub_ = it_.advertise("/opencvimg/left", 1);
    image_r_pub_ = it_.advertise("/opencvimg/right", 1);
    image_r_sub_ = it_.subscribe("/right/camera/image_rect_color", 1, &ImageConverter::imageCbr, this);
    image_l_sub_ = it_.subscribe("/left/camera/image_rect_color", 1, &ImageConverter::imageCbl, this);

    //cv::namedWindow(WINDOW);

    maxstretch_= 400;
    trackpos_ =  maxstretch_/2; 
    //cvCreateTrackbar("Streching:", WINDOW, &trackpos_, maxstretch_, switch_callback_h);
    l_ = 0.00;
    r_ = 0.00;

    inc_ = (((float)pp_size_*2)/maxstretch_)/2;
    pp_size_ = 80;

    blurpos_ = 50;
    maxblur_ = 100;

    alphapos_ = 100;
    maxalpha_ = 100;

    first_time_ = true;
    //out_mat_(240,320,CV_32F);

    viewport_width_ = 640;
    viewport_height_ = 240;

  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void myblur(cv::Mat mat  )
  {
    //IplImage* image(mat);
    cv::namedWindow( "In" );
    cv::namedWindow( "Out" );
    cv::imshow( "In", mat );
    //    cv::cvSmooth
    CvSize b_size=cvSize(15,15); //=cvSize(3,3);
    cv::Mat out(mat.size(), mat.type() );
    cvCreateTrackbar("Blur", "Out", &blurpos_, maxblur_,swith_callback );
    cv::GaussianBlur( mat, out, b_size, ((float)gblur/100) );

    cv::imshow( "Out", out );


  }

  void myalpha(cv::Mat mat  )
  {
    //IplImage* image(mat);
    //cv::namedWindow( "In" );
    cv::namedWindow( "Blend" );
    //cv::imshow( "In", mat );
    //    cv::cvSmooth
    //CvSize b_size=cvSize(15,15);
    cv::Mat out(mat.size(), mat.type() );
    //cvCreateTrackbar("Alpha", "Blend", &blurpos_, maxblur_,swith_callback );
    cvCreateTrackbar("Alpha", "Blend", &alphapos_, maxalpha_,swith_callback_alpha );
    float beta = 1-((float)galpha/100);
    cv::addWeighted(mat, ((float)galpha/100), background_, beta,0.0,out);
    //cv::GaussianBlur( mat, out, b_size, ((float)gblur/100) );

    cv::imshow( "Blend", out );
  }

  int right_eye(cv::Mat mat)
  {

    cv::Range col_range_r(mat.cols,mat.cols*2);  
    cv::Mat roi_mat=stereoDisplay_.colRange(col_range_r);   
    cv::addWeighted(mat, 1.0, roi_mat, 0.0 ,0.0,roi_mat);

    cv::namedWindow( "Right" );
    cv::imshow( "Right", stereoDisplay_);

    
    //    IplImage cv_input_= stereoDisplay_; // No data copying
    // Create Texture
    cv::Mat clone_mat = stereoDisplay_.clone();
    cv::cvtColor(clone_mat,clone_mat, CV_BGR2RGB);
    IplImage* cv_input_;
    cv_input_ = &IplImage(clone_mat);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, cv_input_->width, cv_input_->height, GL_RGB, GL_UNSIGNED_BYTE, cv_input_->imageData);
    
    
    //cv::cvtColor(cv_input_,cv_input_, CV_BGR2RGB);
    //IplImage cv_input_ = stereoDisplay_;
    //    cv::cvtColor(cv_input_,cv_input_, CV_BGR2RGB);
    
    

    return 1;    
  }

  int left_eye(cv::Mat mat)
  {

    cv::Range col_range_l(0,mat.cols);  
    cv::Mat roi_mat=stereoDisplay_.colRange(col_range_l);   
    cv::addWeighted(mat, 1.0, roi_mat, 0.0 ,0.0,roi_mat);

    return 1;    
  }

  int mystereo(cv::Mat left, cv::Mat right )
  {

    
    /*cv::namedWindow( "Right" );
    cv::imshow( "Right", right);
    cv::namedWindow( "Left" );
    cv::imshow( "Left", left);*/
  
    return 1;    
  }


  void imageCbr(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    //CvMat* warp_mat = cvCreateMat(2,3,CV_32FC1);

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
      //cv_input_ = cv_bridge_.imgMsgToCv (msg_ptr, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if ( first_time_ == true)
      {
	// Init matrix with double size
	cv::Mat mat(cv_ptr->image);
	cv::Mat temp(mat.rows , mat.cols*2, mat.type(), cvScalar(0));
	stereoDisplay_ = temp.clone();
	first_time_ = false;
	background_ = cv_ptr->image;
      }


    right_eye(cv_ptr->image);
    //myblur(cv_ptr->image);
    //myblur(cv_ptr->image);
    //myalpha(cv_ptr->image);
    //mystereo(cv_ptr->image, cv_ptr->image);
    //    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    //    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    image_r_pub_.publish(cv_ptr->toImageMsg());
  }

  void imageCbl(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if ( first_time_ == true)
      {
	return;
      }
    left_eye(cv_ptr->image);

    cv::waitKey(3);
    
    image_l_pub_.publish(cv_ptr->toImageMsg());
  }
  
  void display()
  {
    cv::Mat temp;
    temp = stereoDisplay_.clone(); 
    //(mat.rows , mat.cols*2, mat.type(), cvScalar(0));

    //cv::namedWindow( "Stereo" );
    //cv::imshow( "Stereo", temp);
  }

  void displayGL(void)
  {
    
    
    //cv::Mat temp;
    //temp = stereoDisplay_.clone(); 
    //(mat.rows , mat.cols*2, mat.type(), cvScalar(0));

    //cv::namedWindow( "Stereo" );
    //cv::imshow( "Stereo", temp);
  }

  
  
  void spin()
  {
    while(ros::ok())
      {
	//cv::namedWindow( "Right" );
	//cv::imshow( "Right", stereoDisplay_);
	//display();
	//glutDisplayFunc(displayGL);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);

	// Set Projection Matrix
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, viewport_width_ , viewport_height_, 0);

	// Switch to Model View Matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Draw a textured quad
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f); glVertex2f(0.0f, 0.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex2f(viewport_width_ , 0.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex2f(viewport_width_ , viewport_height_ );
	glTexCoord2f(0.0f, 1.0f); glVertex2f(0.0f, viewport_height_ );
	glEnd();

	glFlush();
	glutSwapBuffers();


	glutReshapeFunc(OnReshape);

	glutPostRedisplay();
	glutMainLoopEvent();
	ros::spinOnce();
	publish_rate_.sleep(); //sleep(100);
	//ros::spin();
      }
  }

};

int main(int argc, char** argv)
{

  glutInit(&argc,argv); 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB); // | GLUT_DEPTH);
  glutCreateWindow("Test OpenGL");
  glutInitWindowSize(640,240);
  //  glutReshapeWindow(tap_counter->get_screen_width(),tap_counter->get_screen_height());
  glutInitWindowPosition(0,0); 
  //glutReshapeFunc(HandleReshape);
  //glutKeyboardFunc(HandleKeyboard);
  //glutIdleFunc(HandleIdle);

  //glutDisplayFunc(display);
  //glutDisplayFunc(IsRunning);
  //init();
  //glutMainLoop();

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  //ros::spin();
  ic.spin();  // Loop
  return 0;
}
