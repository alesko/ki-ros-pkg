/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Alexander Skoglund, Karolinska Institute
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


//ROS include
#include <math.h>
#include <ros/ros.h>

//generic C/C++ include
#include <string>
#include <sstream>

#include "dataglove/dataglove_publisher.h"

// Globals
boost::mutex g_fifth_mutex;
float g_scaled[18];
short unsigned int g_raw[18];
ros::Time g_callbacktime;

using namespace ros;

namespace dataglove_publisher{

  // Not a GloveNode member
  static void GloveCallback(void* param)
  {
    ros::Time oldtime;

    //GloveNode* MySelf = (GloveNode*)param;
    DataglovePublisher* MySelf = (DataglovePublisher*)param;
  

    oldtime = g_callbacktime;
    g_fifth_mutex.lock();
    // Retrive data and put into global variables
    g_callbacktime = ros::Time::now();
    fdGetSensorRawAll(MySelf->pGloveA_, g_raw);
    fdGetSensorScaledAll(MySelf->pGloveA_, g_scaled);    
    g_fifth_mutex.unlock();

  }

  /////////////////////////////////
  //    CONSTRUCTOR/DESTRUCTOR   //
  /////////////////////////////////

  DataglovePublisher::DataglovePublisher()
    : n_tilde_("~"), publish_rate_(0.0), publishing_(true) 
  {

    //std::string path_to_calibration;
    //n_tilde_.param("path_to_calibration", path_to_calibration, std::string("/etc/robot/calibration.d/cyberglove.cal"));
    //ROS_INFO("Calibration file loaded for the Cyberglove: %s", path_to_calibration.c_str());

    // set publish frequency
    double publish_freq;
    if (n_tilde_.getParam("/dataglove/publish_frequency", publish_freq))
      {
	ROS_INFO("/dataglove/publish_frequency is %d", publish_freq);
      }

    //n_tilde_.param("publish_frequency", publish_freq, 75.0);
    publish_rate_ = Rate(publish_freq);

    // set path to glove
    if (n_tilde_.getParam("/dataglove/path_to_glove", szPort_))
      {
	ROS_INFO("Opening glove on port: %s", szPort_.c_str());
      }

  }
 
  //int DataglovePublisher::GloveInit(std::string path_to_glove)
  int DataglovePublisher::GloveInit()
  {

    // Initialize glove
    ROS_INFO( "Attempting to open glove A on %s ... ", szPort_.c_str() );
    if (NULL == (pGloveA_ = fdOpen( (char *)szPort_.c_str() )))
      {
	ROS_ERROR( "   failed.\nCouldn't initialize the glove, is the glove plugged in?" );
	return -1;
      }
    ROS_INFO( "   succeeded." );

    glovetype_ = fdGetGloveType(pGloveA_);

    ROS_INFO("Glove A: ");

    switch (glovetype_) {
    case FD_GLOVENONE:    szType_ = "None"; break;
    case FD_GLOVE5U:      szType_ = "Data Glove 5 Ultra"; break;
    case FD_GLOVE5UW:     szType_ = "Data Glove 5 Ultra W"; break;
    case FD_GLOVE5U_USB:  szType_ = "Data Glove 5 Ultra USB"; break;
    case FD_GLOVE7:       szType_ = "Data Glove 5"; break;
    case FD_GLOVE7W:      szType_ = "Data Glove 5W"; break;
    case FD_GLOVE16:      szType_ = "Data Glove 16"; break;
    case FD_GLOVE16W:     szType_ = "Data Glove 16W"; break;
    case FD_GLOVE14U:     szType_ = "DG14 Ultra serial"; break;
    case FD_GLOVE14UW:    szType_ = "DG14 Ultra W"; break;
    case FD_GLOVE14U_USB: szType_ = "DG14 Ultra USB"; break;
    }
	  
    ROS_INFO("Glove type: %s\n", szType_.c_str() );
    ROS_INFO("Glove handedness: %s\n", fdGetGloveHand(pGloveA_)==FD_HAND_RIGHT?"Right":"Left" );
    ROS_INFO("Glove data rate: %i\n", fdGetPacketRate(pGloveA_));
    // Display glove info
    unsigned char buf[64];
    fdGetGloveInfo( pGloveA_, buf );
    ROS_INFO("Glove info: %s\n", (char*)buf );

    //publishes calibrated JointState messages
    std::string prefix;
    std::string searched_param;
    n_tilde_.searchParam("dataglove_prefix", searched_param);
    n_tilde_.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "/calibrated/joint_states";
    dataglove_pub_ = n_tilde_.advertise<sensor_msgs::JointState>(full_topic, 2);
    
    //publishes raw JointState messages
    n_tilde_.searchParam("dataglove_prefix", searched_param);
    n_tilde_.param(searched_param, prefix, std::string());
    full_topic = prefix + "/raw/joint_states";
    dataglove_raw_pub = n_tilde_.advertise<sensor_msgs::JointState>(full_topic, 2);
   
    // Initialized the joint names
    jointstate_msg_.name.push_back("FD_THUMBNEAR");	  
    jointstate_msg_.name.push_back("FD_THUMBFAR");
    jointstate_msg_.name.push_back("FD_THUMBINDEX");  
    jointstate_msg_.name.push_back("FD_INDEXNEAR");	  
    jointstate_msg_.name.push_back("FD_INDEXFAR");	  
    jointstate_msg_.name.push_back("FD_INDEXMIDDLE");
    jointstate_msg_.name.push_back("FD_MIDDLENEAR");
    jointstate_msg_.name.push_back("FD_MIDDLEFAR");	  
    jointstate_msg_.name.push_back("FD_MIDDLERING");  
    jointstate_msg_.name.push_back("FD_RINGNEAR");	  
    jointstate_msg_.name.push_back("FD_RINGFAR"); 
    jointstate_msg_.name.push_back("FD_RINGLITTLE");
    jointstate_msg_.name.push_back("FD_LITTLENEAR"); 
    jointstate_msg_.name.push_back("FD_LITTLEFAR");
    jointstate_msg_.name.push_back("FD_THUMBPALM");
    jointstate_msg_.name.push_back("FD_WRISTBEND");
    jointstate_msg_.name.push_back("FD_PITCH");
    jointstate_msg_.name.push_back("FD_ROLL");  
	  
    jointstate_raw_msg_.name = jointstate_msg_.name;  

    return 0;

  }

  int DataglovePublisher::initCallback()
  {

    //DataglovePublisher glove_object;
    //shadow_sensor_sub = nh.subscribe("/datglove/sensor_msg", 5, &DataglovePublisher::GloveCallback, &glove_object);

  // Call the service to start publishing 
  /*publishing_state_.request.start = true;
  if (client_pub.call(publishing_state_))
    ROS_INFO("Data publishing service started" );
  else
    ROS_ERROR("Failed to call service shadow publishing service");
  */


      // Set up callback function
    fdSetCallback(pGloveA_, (void*)&(GloveCallback), this); // workimng code
    //fdSetCallback(pGloveA_, (void*)&(call_back), pGloveA_); original
    //fdSetCallback(pGloveA_, (void*)&(call_back), pGloveA_); original


    //fifthDT_pub = n.advertise<std_msgs::String>("pub_fifthDT", 10);
    //fifthDT_raw_pub = n.advertise<sensor_msgs::JointState>("pub_fifthDT_raw", 10);
    //fifthDT_scaled_pub = n_tilde_.advertise<sensor_msgs::JointState>("pub_fifthDT_scaled", 10);
  

    //sleep 1s to be sure the glove had enough time to start 
    sleep(1);    

    for(int i=0;i<GLOVE_SIZE;i++){
      raw_old_[i] = 0;
      scaled_old_[i] = 0.0;
      raw_vel_[i] = 0;
      scaled_vel_[i] = 0.0;
    }


    return 0;

  }

  DataglovePublisher::~DataglovePublisher()
  {
    // Close glove
    ROS_INFO("closing glove(s)" );
    fdClose( pGloveA_ );
    ROS_INFO("glove(s) closed, goodbye" );
    /*if( glovePositions_ != NULL )
      {
	delete glovePositions_;
	glovePositions_ = NULL;
	}*/
  }

  bool DataglovePublisher::isPublishing()
  {
    if (publishing_)
      {
	return true;
      }
    else
      {
	ros::spinOnce();
	publish_rate_.sleep();
	return false;
      }
  }

  void DataglovePublisher::setPublishing(bool value){
    publishing_ = value;
  }

  /////////////////////////////////
  //       PUBLISH METHOD        //
  /////////////////////////////////
  void DataglovePublisher::publish()
  {

    //if (!publishing_) return;
    //read the state of the glove button
    int gloveButtonState = 1;
    //gloveButtonState =  read_button_value();

    //check if the value was read
    if(gloveButtonState == -1)
      {
	ROS_ERROR("The glove button state value couldn't be read.");
	ros::spinOnce();
	publish_rate_.sleep();
	return;
      }

    //if the glove button is off, then we don't read / sent position values
    if(gloveButtonState == 0)
      {
	publishing_ = false;
	ROS_DEBUG("The glove button is off, no data will be read / sent");
	ros::spinOnce();
	publish_rate_.sleep();
	return;
      }
    publishing_ = true;
    //read data from the glove
    try
      {
	//glovePositions_ = glove_get_values(); // Rewrite to 5DT
	//fifth_mutex_.unlock();
	g_fifth_mutex.unlock(); // Unlock before going to sleep
	publish_rate_.sleep();
	g_fifth_mutex.lock();
	//fifth_mutex_.lock();
      }
    catch(int e)
      {
	ROS_ERROR("The glove values can't be read");
	ros::spinOnce();
	publish_rate_.sleep();
	return;
      }

    //reset the messages
    jointstate_msg_.effort.clear();
    jointstate_msg_.position.clear();
    jointstate_msg_.velocity.clear();
    jointstate_raw_msg_.effort.clear();
    jointstate_raw_msg_.position.clear();
    jointstate_raw_msg_.velocity.clear();

    //ros::Duration dt = callbacktime - last_time;
    ros::Duration dt = g_callbacktime - last_time_;

    for(int i=0;i<GLOVE_SIZE;i++){
      //raw_vel_[i] = (short unsigned int)((double)(GLOBAL_RAW_[i] - raw_old_[i]) / ((double) dt.toSec()));
      //scaled_vel_[i] = (float)((GLOBAL_SCALED_[i] - scaled_old_[i]) / (double) dt.toSec());
      raw_vel_[i] = (short unsigned int)((double)(g_raw[i] - raw_old_[i]) / ((double) dt.toSec()));
      scaled_vel_[i] = (float)((g_raw[i] - scaled_old_[i]) / (double) dt.toSec());
    }


    for(int i=0;i<GLOVE_SIZE;i++){
      //jointstate_raw_msg_.position.push_back(GLOBAL_RAW_[i]);
      jointstate_raw_msg_.position.push_back(g_raw[i]);
      jointstate_raw_msg_.effort.push_back(0.0);
      jointstate_raw_msg_.velocity.push_back(raw_vel_[i]);
      //jointstate_msg_.position.push_back(GLOBAL_SCALED_[i]);
      jointstate_msg_.position.push_back(g_scaled[i]);
      jointstate_msg_.effort.push_back(0.0);
      jointstate_msg_.velocity.push_back(scaled_vel_[i]);
    }
    jointstate_msg_.header.stamp = g_callbacktime ;
    jointstate_raw_msg_.header.stamp = g_callbacktime ;

    //publish the msgs 
    dataglove_pub_.publish(jointstate_msg_);
    dataglove_raw_pub.publish(jointstate_raw_msg_);

    for(int i=0;i<GLOVE_SIZE;i++){
      //raw_old_[i] = GLOBAL_RAW_[i];
      //scaled_old_[i] = GLOBAL_SCALED_[i];
      raw_old_[i] = g_raw[i];
      scaled_old_[i] = g_scaled[i];
    }

    //last_time = callbacktime;
    last_time_ = g_callbacktime;
    ros::spinOnce();
    //publish_rate_.sleep();
  }

  void DataglovePublisher::add_jointstate(float position, std::string joint_name)
  {
    //can't read the effort from the glove
    jointstate_msg_.effort.push_back(0.0);

    //get the calibration value
    //float calibration_value = calibration_parser.get_calibration_value(position, joint_name);
    //std::cout << calibration_value << std::endl;

    //publish the glove position
    //jointstate_msg_.position.push_back(calibration_value);
    //set velocity to 0. 
    //@TODO : send the correct velocity ?
    jointstate_msg_.velocity.push_back(0.0);
  }

}// end namespace


