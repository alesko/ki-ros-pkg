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
#include <string.h>
#include <fstream> //for file saving etc
#include <stdio.h>
#include <string>
#include <sys/time.h>  //For time operations
#include <time.h>
#include <unistd.h>

#include "std_msgs/String.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;

static const char C_WINDOW[] = "Control window";
static const char F_WINDOW[] = "Filter window";

int ghighInt;
void switch_callback_h( int position ){
  ghighInt = position;
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
  ros::Subscriber keyboard_sub_;

  cv::Mat warp_mat_; 
  cv::Mat clone_mat_;
  cv::Mat stereoDisplay_;

  IplImage* cv_input_;
  IplImage* warp_; 
  //  IplImage* frame_;

  ros::Rate publish_rate_;

  int maxstretch_;
  int trackpos_; 
  float inc_;
  int percent_;
  int percent1_;
  float tp_;
  int cond_;
  int  maxcond_;
  float factor_;
  int newpos_; //resets track position between judgments

  float left_corner_pos_;
  float right_corner_pos_;

  float top_left_;
  float top_right_;
  bool first_time_;

  cv::Point2f frameTri_[3], warpTri_[3];		
  
  int pp_size_; // percentage size of participant compared to mannequin x2
  int viewport_width_ ;
  int viewport_height_ ;

  ofstream data_file_; // data file for logging
  cv::Mat filter_mat_;

  int num_of_trails_;
  int trial_;
  
  // Font stuff
  // Init the text:
  /*cv::Scalar red_ ;
  double hscale_ ;
  double vscale_ ;
  double shear_ ;
  int thickness_ ;
  int line_type_ ;*/
  
  // Create a buffer to put the text:
  //char text_[100];
  
  // Init the font:
  //cv::Font font1_;
  //  cvInitFont(&font1,CV_FONT_HERSHEY_DUPLEX,hscale_,vscale_,shear_,thickness_,line_type_);
  
  // Location of text:
  //cv::Point pt_;
  //pt_ = cvPoint(10,30); // (0,0) is uppel left corner, figures are pixels

public:

  ImageConverter()
    : it_(nh_),publish_rate_(60)
  {

    image_r_sub_ = it_.subscribe("/right/camera/image_rect_color", 1, &ImageConverter::imageCbr, this);
    image_l_sub_ = it_.subscribe("/left/camera/image_rect_color", 1, &ImageConverter::imageCbl, this);
    keyboard_sub_ = nh_.subscribe("/key", 1, &ImageConverter::keyboard, this); 

    maxstretch_= 400;
    maxcond_ = 3;
    trackpos_ =  maxstretch_/2; 

    left_corner_pos_ = 0.00; //left position on picture
    right_corner_pos_ = 0.00;//right position on picture    

    //cond_ = 3;  
    factor_ = 0;

    top_left_ = -0.00;
    top_right_ = 0;

    pp_size_ = 100; // percentage size of participant compared to mannequin x2

    first_time_ = true;

    //this gives file name to saved file basing it on time and date
    time_t t = time(0);
    struct tm* lt = localtime(&t);
    char time_str[256];
    sprintf(time_str, "myloggfile_%04d%02d%02d_%02d%02d%02d.log",
	    lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
	    lt->tm_hour, lt->tm_min, lt->tm_sec);
    //creates a file
    data_file_.open(time_str);
    trial_ = 0;
    num_of_trails_ = 6;
    newpos_ = maxstretch_;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(C_WINDOW);
    cv::destroyWindow(F_WINDOW);
  }
  
  //IplImage* GetThresholdedImage(IplImage* img)
  void GetThresholdedImage(cv::Mat mat)
  {
    cv::namedWindow( F_WINDOW );
    // Convert the image into an HSV image - function to be used to get a filtered images below
    //IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
    //cvCvtColor(img, imgHSV, CV_BGR2HSV);

    filter_mat_ = mat.clone();
    cv::Mat imgHSV =  mat.clone();
    cv::cvtColor(imgHSV,imgHSV, CV_BGR2HSV); 

    //IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

    //    cvInRangeS(imgHSV, cvScalar(30, 100, 100), cvScalar(50, 255, 255), imgThreshed);//play with these numbers and different colours in the right environment
    cv::Scalar lowerb(30, 100, 100);
    cv::Scalar upperb(50, 255, 255);
    inRange(imgHSV,lowerb, upperb, filter_mat_);
    
    cv::imshow(F_WINDOW, filter_mat_);
      //cvReleaseImage(&imgHSV);

      //return imgThreshed;


  }
  
  int control_window(cv::Mat mat)
  {
    
    cv::Size size = mat.size();
    int w = size.width;
    int h = size.height;    

    cv::namedWindow( C_WINDOW );
    
    
    cvCreateTrackbar("Streching:", C_WINDOW, &trackpos_, maxstretch_, switch_callback_h);
    //    cvCreateTrackbar("Condition", WINDOW, &cond_,  maxcond_ ,switch_callback_cond);

    /*switch(cond_){
    case 0:
      tp_ = (80/(100/(float)pp_size_));
      trackpos_ = (long)tp_ *2;
      cvSetTrackbarPos("Streching:", "Control window", trackpos_ );
      break;
    case 1:
      trackpos_ = pp_size_ *2;
      cvSetTrackbarPos("Streching:", "Control window", trackpos_ );
      break;
    case 2:
      tp_ = (120/(100/(float)pp_size_));
      trackpos_ = (long)tp_ *2;
      cvSetTrackbarPos("Streching:", "Control window", trackpos_ );
      }

    factor_ = (float)trackpos_/400;
    top_left_ = 0 - factor_;
    top_right_ = 1 + factor_;
    left_corner_pos_ = top_left_ + 0.5;
    right_corner_pos_ = top_right_ - 0.5;

    percent_ = (100/(float)pp_size_)* ((float)trackpos_/2);
*/
      inc_ = (((float)pp_size_*2)/maxstretch_)/2;
    left_corner_pos_ = 0.5 - ((inc_/100) *trackpos_);
    right_corner_pos_ = 0.5 + ((inc_/100)* trackpos_);
    percent_ = trackpos_/2;
    percent1_ = (long) percent_;//percent change as an int to be displayed on the original image

    // Set up transformation matices 
    frameTri_[0].x = 0;
    frameTri_[0].y = 0;
    frameTri_[1].x = w - 1;//frame->width - 1;
    frameTri_[1].y = 0;
    frameTri_[2].x = 0;
    frameTri_[2].y = h - 1;

    warpTri_[0].x = w*left_corner_pos_; //topleft
    warpTri_[0].y = 0.00;
    warpTri_[1].x = w*right_corner_pos_; //topright
    warpTri_[1].y = 0.00;    
    warpTri_[2].x = w*left_corner_pos_;//bottom;left
    warpTri_[2].y = h;
    
    cv::imshow(C_WINDOW, mat);

    return 1;
  }


  void stretchMat(cv::Mat mat, bool left)
  {

    cv::Size size = mat.size();
    cv::Mat stretched_mat = mat.clone();

    // Do the transformation
    warp_mat_ = cv::getAffineTransform( frameTri_, warpTri_);
    cv::warpAffine( mat, stretched_mat, warp_mat_, size);
    
    cv::Range col_range;

    if(left)
      {
	// Set ROI to be the left view 
	col_range.start=mat.cols;
	col_range.end = mat.cols*2;  
      }
    else
      {
	// Set ROI to be the right view 
	col_range.start = 0;
	col_range.end = mat.cols;
      }

    // Put the transformed image in the correct location
    cv::Mat roi_mat = stereoDisplay_.colRange(col_range);
    cv::addWeighted(stretched_mat, 1.0, roi_mat, 0.0 ,0.0,roi_mat);

    roi_mat.release();
    stretched_mat.release();
    
  }
  

  int map2texture(void)
  {
    // Create OpenGL texture
    clone_mat_ = stereoDisplay_.clone();
    cv::cvtColor(clone_mat_,clone_mat_, CV_BGR2RGB);    
    cv_input_ = &IplImage(clone_mat_);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, cv_input_->width, cv_input_->height, GL_RGB, GL_UNSIGNED_BYTE, cv_input_->imageData);

    return 1;    
  }

  void keyboard(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
  }

  void imageCbr(const sensor_msgs::ImageConstPtr& msg)
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
	// Init matrix with double size
	cv::Mat mat(cv_ptr->image);
	cv::Mat temp(mat.rows , mat.cols*2, mat.type(), cvScalar(0));

	// Change according to camera
	viewport_width_ = mat.cols*2; // For stereo twice the width is needed
	viewport_height_ = mat.rows;	

	stereoDisplay_ = temp.clone();
	first_time_ = false;
	//background_ = cv_ptr->image;
      }

    control_window(cv_ptr->image);
    GetThresholdedImage(cv_ptr->image);
    stretchMat(cv_ptr->image, false);
    map2texture();

    cv::waitKey(2);
    
    // image_r_pub_.publish(cv_ptr->toImageMsg());
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
    stretchMat(cv_ptr->image, true);

    cv::waitKey(2);
    
    //image_l_pub_.publish(cv_ptr->toImageMsg());
  }

  
  void spin()
  {
    while(ros::ok())
      {

	char c = cvWaitKey(20);
	//13 = carriage return - press to select judgement and move onto next trial
	if (c == 13) 
	  {
	    c = 0;
	    data_file_ << trial_ << "\t" << percent_ << "\n";
	    ROS_INFO("That was trial no: %d", trial_);
	    if (newpos_ == 0)
	      newpos_ = maxstretch_;
	    else
	      newpos_ = 0;
	    trial_++;
	    if(trial_ <= num_of_trails_)
	      {
		
		ros::shutdown(); // A gentle shutdown when trials are done
	      }
	  }

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
      }
  }

};

int main(int argc, char** argv)
{

  glutInit(&argc,argv); 
  //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH |GLUT_STEREO);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );
  
  glutInitWindowSize(800,600);
  glutInitWindowPosition(0,0); 
  glutCreateWindow("Test OpenGL");

  ros::Time::init();
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ic.spin();  // Loop
  return 0;
}
