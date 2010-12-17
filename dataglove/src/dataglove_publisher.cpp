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

/* This code is built from Ugo Cupcic <ugo@shadowrobot.com> code for
   Cyberglove in ROS 
*/


//ROS include
#include <math.h>
#include <ros/ros.h>

//generic C/C++ include
#include <string>
#include <sstream>

#include "dataglove/dataglove_publisher.h"

// Globals
boost::mutex g_fifth_mutex ;
//float GLOBAL_SCALED_[18];
//short unsigned int GLOBAL_RAW_[18];
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
  
    //oldtime = callbacktime;
    oldtime = g_callbacktime;
    //fifth_mutex_.lock();
    g_fifth_mutex.lock();
    // Retrive data and put into global variables
    //callbacktime = ros::Time::now();
    g_callbacktime = ros::Time::now();
    //fdGetSensorRawAll(MySelf->pGloveA,GLOBAL_RAW_);
    //fdGetSensorScaledAll(MySelf->pGloveA, GLOBAL_SCALED_);
    fdGetSensorRawAll(MySelf->pGloveA, g_raw);
    fdGetSensorScaledAll(MySelf->pGloveA, g_scaled);    
    //fifth_mutex_.unlock();
    g_fifth_mutex.unlock();
  }

  /////////////////////////////////
  //    CONSTRUCTOR/DESTRUCTOR   //
  /////////////////////////////////

  DataglovePublisher::DataglovePublisher()
    : n_tilde("~"), publish_rate(0.0), path_to_glove("/dev/usb/hiddev1"), publishing(true)
  {

    //std::string path_to_calibration;
    //n_tilde.param("path_to_calibration", path_to_calibration, std::string("/etc/robot/calibration.d/cyberglove.cal"));
    //ROS_INFO("Calibration file loaded for the Cyberglove: %s", path_to_calibration.c_str());

    //initialize_calibration(path_to_calibration);

    // set publish frequency
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 75.0);
    publish_rate = Rate(publish_freq);

    // set path to glove
    n_tilde.param("path_to_glove", path_to_glove, std::string("/dev/usb/hiddev1"));
    ROS_INFO("Opening glove on port: %s", path_to_glove.c_str());

    //int error = setup_glove( path_to_glove.c_str() ); // Replace with call to init 
    int error = GloveInit( path_to_glove.c_str() ); 
    //sleep 1s to be sure the glove had enough time to start 
    sleep(1);

    if( error != 0 )
      ROS_ERROR("Couldn't initialize the glove, is the glove plugged in?");
    else
      {
	//publishes calibrated JointState messages
	std::string prefix;
	std::string searched_param;
	n_tilde.searchParam("cyberglove_prefix", searched_param);
	n_tilde.param(searched_param, prefix, std::string());
	std::string full_topic = prefix + "/calibrated/joint_states";
	dataglove_pub = n_tilde.advertise<sensor_msgs::JointState>(full_topic, 2);

	//publishes raw JointState messages
	n_tilde.searchParam("cyberglove_prefix", searched_param);
	n_tilde.param(searched_param, prefix, std::string());
	full_topic = prefix + "/raw/joint_states";
	dataglove_raw_pub = n_tilde.advertise<sensor_msgs::JointState>(full_topic, 2);
      }


    // Initialized the joint names
    jointstate_msg.name.push_back("FD_THUMBNEAR");	  
    jointstate_msg.name.push_back("FD_THUMBFAR");
    jointstate_msg.name.push_back("FD_THUMBINDEX");  
    jointstate_msg.name.push_back("FD_INDEXNEAR");	  
    jointstate_msg.name.push_back("FD_INDEXFAR");	  
    jointstate_msg.name.push_back("FD_INDEXMIDDLE");
    jointstate_msg.name.push_back("FD_MIDDLENEAR");
    jointstate_msg.name.push_back("FD_MIDDLEFAR");	  
    jointstate_msg.name.push_back("FD_MIDDLERING");  
    jointstate_msg.name.push_back("FD_RINGNEAR");	  
    jointstate_msg.name.push_back("FD_RINGFAR"); 
    jointstate_msg.name.push_back("FD_RINGLITTLE");
    jointstate_msg.name.push_back("FD_LITTLENEAR"); 
    jointstate_msg.name.push_back("FD_LITTLEFAR");
    jointstate_msg.name.push_back("FD_THUMBPALM");
    jointstate_msg.name.push_back("FD_WRISTBEND");
    jointstate_msg.name.push_back("FD_PITCH");
    jointstate_msg.name.push_back("FD_ROLL");	
	  
    jointstate_raw_msg.name = jointstate_msg.name;    

    for(int i=0;i<GLOVE_SIZE;i++){
      raw_old[i] = 0;
      scaled_old[i] = 0.0;
      raw_vel[i] = 0;
      scaled_vel[i] = 0.0;
    }
  }

  int DataglovePublisher::GloveInit(std::string path_to_glove)
  {

    szPort = path_to_glove;
    // Initialize glove
    ROS_INFO( "Attempting to open glove A on %s ... ", szPort.c_str() );
    if (NULL == (pGloveA = fdOpen( (char *)szPort.c_str() )))
      {
	ROS_ERROR( "   failed." );
	return -1;
      }
    ROS_INFO( "   succeeded." );

    glovetype = fdGetGloveType(pGloveA);

    ROS_INFO("Glove A: ");

    switch (glovetype) {
    case FD_GLOVENONE:    szType = "None"; break;
    case FD_GLOVE5U:      szType = "Data Glove 5 Ultra"; break;
    case FD_GLOVE5UW:     szType = "Data Glove 5 Ultra W"; break;
    case FD_GLOVE5U_USB:  szType = "Data Glove 5 Ultra USB"; break;
    case FD_GLOVE7:       szType = "Data Glove 5"; break;
    case FD_GLOVE7W:      szType = "Data Glove 5W"; break;
    case FD_GLOVE16:      szType = "Data Glove 16"; break;
    case FD_GLOVE16W:     szType = "Data Glove 16W"; break;
    case FD_GLOVE14U:     szType = "DG14 Ultra serial"; break;
    case FD_GLOVE14UW:    szType = "DG14 Ultra W"; break;
    case FD_GLOVE14U_USB: szType = "DG14 Ultra USB"; break;
    }
	  
    ROS_INFO("Glove type: %s\n", szType.c_str() );
    ROS_INFO("Glove handedness: %s\n", fdGetGloveHand(pGloveA)==FD_HAND_RIGHT?"Right":"Left" );
    ROS_INFO("Glove data rate: %i\n", fdGetPacketRate(pGloveA));
    // Display glove info
    unsigned char buf[64];
    fdGetGloveInfo( pGloveA, buf );
    ROS_INFO("Glove info: %s\n", (char*)buf );

    // Set up callback function
    fdSetCallback(pGloveA, (void*)&(GloveCallback), this);

    //fifthDT_pub = n.advertise<std_msgs::String>("pub_fifthDT", 10);
    //fifthDT_raw_pub = n.advertise<sensor_msgs::JointState>("pub_fifthDT_raw", 10);
    //fifthDT_scaled_pub = n_tilde.advertise<sensor_msgs::JointState>("pub_fifthDT_scaled", 10);
  
   
    return 0;

  }

  DataglovePublisher::~DataglovePublisher()
  {
    // Close glove
    ROS_INFO("closing glove(s)" );
    fdClose( pGloveA );
    ROS_INFO("glove(s) closed, goodbye" );
    /*if( glovePositions != NULL )
      {
	delete glovePositions;
	glovePositions = NULL;
	}*/
  }

  bool DataglovePublisher::isPublishing()
  {
    if (publishing)
      {
	return true;
      }
    else
      {
	ros::spinOnce();
	publish_rate.sleep();
	return false;
      }
  }

  void DataglovePublisher::setPublishing(bool value){
    publishing = value;
  }

  /////////////////////////////////
  //       PUBLISH METHOD        //
  /////////////////////////////////
  void DataglovePublisher::publish()
  {

    //if (!publishing) return;
    //read the state of the glove button
    int gloveButtonState = 1;
    //gloveButtonState =  read_button_value();

    //check if the value was read
    if(gloveButtonState == -1)
      {
	ROS_ERROR("The glove button state value couldn't be read.");
	ros::spinOnce();
	publish_rate.sleep();
	return;
      }

    //if the glove button is off, then we don't read / sent position values
    if(gloveButtonState == 0)
      {
	publishing = false;
	ROS_DEBUG("The glove button is off, no data will be read / sent");
	ros::spinOnce();
	publish_rate.sleep();
	return;
      }
    publishing = true;
    //read data from the glove
    try
      {
	//glovePositions = glove_get_values(); // Rewrite to 5DT
	//fifth_mutex_.unlock();
	g_fifth_mutex.unlock(); // Unlock before going to sleep
	publish_rate.sleep();
	g_fifth_mutex.lock();
	//fifth_mutex_.lock();
      }
    catch(int e)
      {
	ROS_ERROR("The glove values can't be read");
	ros::spinOnce();
	publish_rate.sleep();
	return;
      }

    //reset the messages
    jointstate_msg.effort.clear();
    jointstate_msg.position.clear();
    jointstate_msg.velocity.clear();
    jointstate_raw_msg.effort.clear();
    jointstate_raw_msg.position.clear();
    jointstate_raw_msg.velocity.clear();

    //ros::Duration dt = callbacktime - last_time;
    ros::Duration dt = g_callbacktime - last_time;

    for(int i=0;i<GLOVE_SIZE;i++){
      //raw_vel[i] = (short unsigned int)((double)(GLOBAL_RAW_[i] - raw_old[i]) / ((double) dt.toSec()));
      //scaled_vel[i] = (float)((GLOBAL_SCALED_[i] - scaled_old[i]) / (double) dt.toSec());
      raw_vel[i] = (short unsigned int)((double)(g_raw[i] - raw_old[i]) / ((double) dt.toSec()));
      scaled_vel[i] = (float)((g_raw[i] - scaled_old[i]) / (double) dt.toSec());
    }


    for(int i=0;i<GLOVE_SIZE;i++){
      //jointstate_raw_msg.position.push_back(GLOBAL_RAW_[i]);
      jointstate_raw_msg.position.push_back(g_raw[i]);
      jointstate_raw_msg.effort.push_back(0.0);
      jointstate_raw_msg.velocity.push_back(raw_vel[i]);
      //jointstate_msg.position.push_back(GLOBAL_SCALED_[i]);
      jointstate_msg.position.push_back(g_scaled[i]);
      jointstate_msg.effort.push_back(0.0);
      jointstate_msg.velocity.push_back(scaled_vel[i]);
    }
    jointstate_msg.header.stamp = g_callbacktime ;
    jointstate_raw_msg.header.stamp = g_callbacktime ;

 
    
    //publish the msgs 
    dataglove_pub.publish(jointstate_msg);
    dataglove_raw_pub.publish(jointstate_raw_msg);

    for(int i=0;i<GLOVE_SIZE;i++){
      //raw_old[i] = GLOBAL_RAW_[i];
      //scaled_old[i] = GLOBAL_SCALED_[i];
      raw_old[i] = g_raw[i];
      scaled_old[i] = g_scaled[i];
    }

    //last_time = callbacktime;
    last_time = g_callbacktime;
    ros::spinOnce();
    //publish_rate.sleep();
  }

  void DataglovePublisher::add_jointstate(float position, std::string joint_name)
  {
    //can't read the effort from the glove
    jointstate_msg.effort.push_back(0.0);

    //get the calibration value
    //float calibration_value = calibration_parser.get_calibration_value(position, joint_name);
    //std::cout << calibration_value << std::endl;

    //publish the glove position
    //jointstate_msg.position.push_back(calibration_value);
    //set velocity to 0. 
    //@TODO : send the correct velocity ?
    jointstate_msg.velocity.push_back(0.0);
  }

}// end namespace

