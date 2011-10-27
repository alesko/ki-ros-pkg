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

//#include <vector>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
//#include <tf/transform_broadcaster.h>



#include <servo_base.h>
#include <servo_commands.h>
#include <servo_io.h>

#include "servo_node.h"

/*
// messages
//#include <shadow/Sensors.h>
#include <shadow/Valves.h>

// services
#include <shadow/GetStatus.h>
#include <shadow/GetSensors.h>
#include <shadow/SetTargets.h>
#include <shadow/SetValves.h>
#include <shadow/PulseValves.h>
#include <shadow/StartPublishing.h>
*/
using namespace ros;

ServoClass::ServoClass(): node_("~"), publish_rate_(50)
{
  std::string dev;
  std::string searched_param;
  double publish_freq;

  ROS_INFO("Creating a Servo node");
    
  servo_ = servoInitialize();
  
  // Find the servo prefix
  node_.searchParam("servo_prefix", searched_param);
  node_.param(searched_param, prefix_, std::string());

  // set path to SPCU
  std::string full_topic = prefix_ + "/path_to_servo";
  if (node_.getParam(full_topic, dev))
    {
      ROS_INFO("Path to servo is: %s", dev.c_str());
    }
  else
    {
      ROS_ERROR("Unable to determine path to servo, full_topic=%s", full_topic.c_str());
      return;
    }

  strcpy(servo_->dev.ttyport, dev.c_str());

  //private_nh_.searchParam("/spcu_publish_frequency", searched_param);
  //private_nh_.param(searched_param, prefix_, std::string());


  // set publish frequency from parameter server
  full_topic = prefix_ + "/servo_publish_frequency";    

  if (node_.getParam(full_topic, publish_freq))
    {      
      ROS_INFO("Frequency from %s is %f", full_topic.c_str(), publish_freq);
      publish_rate_ = Rate(publish_freq);      
    }
  
  
  ROS_INFO("Servo node is created");

}

ServoClass::~ServoClass()
{
  ROS_INFO("Closing servo" );
  servoReset(&servo_->dev);
  servoDeviceClosePort(&servo_->dev);
  ROS_INFO("Servo closed, goodbye" );

}
/*
ShadowNode::ShadowNode(std::string dev) : private_nh_("~"), publish_rate_(60)//publish_rate_(60)
{

  shadow_ = shadowInitialize();
  strcpy(shadow_->dev.ttyport, dev.c_str());
 
}
*/
 
void ServoClass::ServoInit()
{
  ROS_INFO("Initializing Servo");
  int msg_que_len = 5;    

  //servoDeviceSetParams(&servo_->dev);

  
  if (servoDeviceConnectPort(&servo_->dev) < 0) 
    {
      ROS_FATAL("Unable to connect shadow at %s\n", servo_->dev.ttyport);
      node_.shutdown();
      return;   
    }
  else
    {
      ROS_INFO("Connected to device");
      //publishes sensor readings
      //std::string prefix;
      //std::string searched_param;
      //private_nh_.searchParam("shadow_prefix", searched_param);
      //private_nh_.param(searched_param, prefix_, std::string());
      std::string full_topic = prefix_ + "/positions";
      ROS_INFO("Starting publisher!");
      position_pub_ = node_.advertise<zxservo::positions>(full_topic,msg_que_len );
      //shadow_pub_ = private_nh_.advertise<shadow::ShadowSensors>(full_topic,msg_que_len );
      //full_topic = prefix_ + "/target_msg";
      //target_pub_ = private_nh_.advertise<shadow::ShadowTargets>(full_topic,msg_que_len );
    }
    
  // ***** Parameters *****
  /*
  ROS_INFO("Retrieving module state");
  shadowHexStatus(shadow_);  
  shadowPrintParams(&shadow_->par);
     
  //sensor_reading_pub_ = private_nh_.advertise<shadow::Sensors>("sensors_pub", 100);

  ROS_INFO("Starting services!");
  system_status_srv_ = private_nh_.advertiseService("get_status", &ShadowNode::getStatus,this);
  set_valves_srv_ = private_nh_.advertiseService("set_valves", &ShadowNode::setValves,this);
  pulse_valves_srv_ = private_nh_.advertiseService("pulse_valves", &ShadowNode::pulseValves,this);
  sensor_reading_srv_ = private_nh_.advertiseService("get_sensor_readings", &ShadowNode::getSensorReading,this);
  targets_srv_ = private_nh_.advertiseService("set_targets", &ShadowNode::setTargets,this);
  contoller_srv_ = private_nh_.advertiseService("enable_controller", &ShadowNode::setController,this);
  contoller_target_srv_ = private_nh_.advertiseService("enable_controller_target", &ShadowNode::setControllerwTarget,this);
  disable_contoller_srv_ = private_nh_.advertiseService("disable_controller", &ShadowNode::disController,this);
  publishing_srv_ = private_nh_.advertiseService("publishing_service", &ShadowNode::setPublishing,this);
*/
  
  servoSetBaudrate(&servo_->dev);
  publishing_ = true; //false;
  ROS_INFO("Servo is ready!");
  /*int left=0;

  int pos0;
  int pos2;

  for(int i =0; i < 5; i++){   
    if( left == 0)
      {
	servoSetPosition(&servo_->dev, 0x00, 0x00, 1200);
	servoSetPosition(&servo_->dev, 0x02, 0x00, 300);
	left = 1;
      }
    else
      {
	servoSetPosition(&servo_->dev, 0x00, 0x00, 300);
	servoSetPosition(&servo_->dev, 0x02, 0x00, 1200);
	left = 0;
      }
    usleep(10000);
    servoReportPosition(&servo_->dev, 0x00, &pos0);
    ROS_INFO("Posistion ch 0: %d",pos0 );
    servoReportPosition(&servo_->dev, 0x02, &pos2);
    ROS_INFO("Posistion ch 2: %d",pos2 );
  }
  servoSetPosition(&servo_->dev, 0x00, 0x00, 300);
  servoSetPosition(&servo_->dev, 0x02, 0x00, 1200);
  servoFirmwareVersion(&servo_->dev);
  */

}


/*
bool ShadowNode::setValves(shadow::SetValves::Request& req, shadow::SetValves::Response& resp)
{   
  int set_valve[NUM_VALVES]={0,0,0,0,0,0,0,0};
  //if( req.valve0_state > 
  set_valve[0]=req.valve0_state; 
  set_valve[1]=req.valve1_state;
  set_valve[2]=req.valve2_state; 
  set_valve[3]=req.valve3_state;
  set_valve[4]=req.valve4_state; 
  set_valve[5]=req.valve5_state;
  set_valve[6]=req.valve6_state; 
  set_valve[7]=req.valve7_state;

  shadowHexSetValves(&shadow_->dev, set_valve);
  return true;

}
*/
/*
bool ShadowNode::pulseValves(shadow::PulseValves::Request& req, shadow::PulseValves::Response& resp)
{   
  //  int i;
  int p_time_ms[NUM_VALVES]={req.valve0_time_ms, req.valve1_time_ms,
			     req.valve2_time_ms, req.valve3_time_ms,
			     req.valve4_time_ms, req.valve5_time_ms,
			     req.valve6_time_ms, req.valve7_time_ms};  
  //ROS_INFO("SHADOW: Pulsing values");
  shadowHexPulseValves(&shadow_->dev,p_time_ms);
  return true;
}

bool ShadowNode::getSensorReading(shadow::GetSensors::Request& req, shadow::GetSensors::Response& resp)
{
  int i;
  unsigned short  sensor_val[8];

  shadow_mutex_.lock();
  // Read the sensor values
  shadowHexReadSensors(&shadow_->dev,sensor_val);
  resp.header.stamp = ros::Time::now();    
  // Send the values
  for(i=0;i < 8;i++)
    resp.Sensors[i] = sensor_val[i];

  shadow_mutex_.unlock();
   
  return true;
}
*/
/*
bool ShadowNode::setController(shadow::SetController::Request& req, shadow::SetController::Response& resp)
{   
  
  //ROS_INFO("Got: v=%d, s=%d, P=%d, I=%d, D=%d",(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
   // (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);
  shadow_mutex_.lock();
  //shadowHexSetController(&shadow_->dev,(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
  //		 (char)-1, (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);
  shadowAsciiSetController(&shadow_->dev,(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
  		 (char)-1, (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);
  shadow_mutex_.unlock();
  ROS_INFO("SHADOW: Setting controller values for valve %d using sensor %d: P=%d, I=%d, D=%d",
	   (unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
	   (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);

  //shadowHexPulseValves(&shadow_->dev,p_time_ms);

  return true;
}
*/
/*
bool ShadowNode::setControllerwTarget(shadow::SetControllerwTarget::Request& req, shadow::SetControllerwTarget::Response& resp)
{   
  
//ROS_INFO("Got: v=%d, s=%d, P=%d, I=%d, D=%d",(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
  //  (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);
  shadow_mutex_.lock();
  //shadowHexSetController(&shadow_->dev,(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
  //		 (char)-1, (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);
  shadowAsciiSetController(&shadow_->dev,(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
  		 (char)req.Controller_target, (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);
  shadow_mutex_.unlock();
  ROS_INFO("SHADOW: Setting controller values for valve %d using sensor %d: P=%d, I=%d, D=%d",
	   (unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
	   (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);

  //shadowHexPulseValves(&shadow_->dev,p_time_ms);

  return true;
}
*/
/*
bool ShadowNode::setTargets(shadow::SetTargets::Request& req, shadow::SetTargets::Response& resp)
{
  // TODO: check why this function makes the hardware "hang"
  // Impossible to retrive the status after the target is set
  
  //int  set_target[NUM_VALVES];
  int i;
  for(i=0; i < NUM_VALVES; i++)
    set_target_[i]= req.target[i];
  for(i=0; i < NUM_VALVES; i++)
    ROS_INFO("SHADOW: Setting targets:[%d]=%d",i,set_target_[i]);
 
  ROS_INFO("SHADOW: Setting targets");

  shadow_mutex_.lock();
  //shadowHexSetTargets(&shadow_->dev,set_target);
  shadowAsciiSetTargets(&shadow_->dev,set_target_);
  shadow_mutex_.unlock();


  return true;
}
*/
/*
bool ShadowNode::disController(shadow::DisableController::Request& req, shadow::DisableController::Response& resp)
{   
 
  shadow_mutex_.lock();
  shadowHexDisableController(&shadow_->dev, req.Controller_valve);
  shadow_mutex_.unlock();
  ROS_INFO("SHADOW: Disabled controller for valve: %d",
	   (unsigned short)req.Controller_valve);

  return true;
}
*/
/*
bool ShadowNode::getStatus(shadow::GetStatus::Request& req, shadow::GetStatus::Response& resp)
{
  int i;

  ROS_INFO("SHADOW: Retrieving module state");

  shadow_mutex_.lock();
  if( shadowHexStatus(shadow_) != 1){      
    ROS_ERROR("SHADOW: Error when retrieving module state");
    return false;
  }
  shadow_mutex_.unlock();
  shadowPrintParams(&shadow_->par);

  resp.TimeStamp[0] = shadow_->par.TimeStamp[0];
  resp.TimeStamp[1] = shadow_->par.TimeStamp[1];
  for(i=0;i<8;i++){
    resp.Sensors[i]           = shadow_->par.Sensors[i];
    resp.Targets[i]           = shadow_->par.Targets[i];
    resp.ValveStates[i]       = shadow_->par.ValveStates[i];
    resp.Controller_Sensor[i] = shadow_->par.Controller_Sensor[i];
    resp.Controller_Target[i] = shadow_->par.Controller_Target[i];
    resp.Controller_P[i]      = shadow_->par.Controller_P[i];
    resp.Controller_I[i]      = shadow_->par.Controller_I[i];
    resp.Controller_D[i]      = shadow_->par.Controller_D[i];
  }

  resp.SetValveStates    = shadow_->par.SetValveStates;
  resp.ActualValveStates = shadow_->par.ActualValveStates;
  resp.LATAreg           = shadow_->par.LATAreg;
  resp.LATBreg           = shadow_->par.LATBreg; 
  resp.ForceStates       = shadow_->par.ForceStates;

  return true;
}
*/
/*
bool ShadowNode::setPublishing(shadow::StartPublishing::Request& req, shadow::StartPublishing::Response& resp)
{
  if(req.start)
    {
      ROS_INFO("SPCU is now publishing sensor data");
      publishing_ = true;
      resp.state = true;
      //this->pub->setPublishing(true);    
    }
  else
    {
      ROS_INFO("SPCU has stopped publishing");
      publishing_ = false;
      resp.state = false;
      //this->pub->setPublishing(false);
    }
  //this->pub->cyberglove_pub.shutdown();
  return true;
}

*/


bool  ServoClass::isPublishing()
{
  if (publishing_)
    {
      return true;
    }
  else
    {
      //ros::spinOnce();
      //publish_rate_.sleep();
      return false;
    }
}



void ServoClass::publish()
{
 
  int i;
  //unsigned short  sensor_val[8];
  //shadow_mutex_.lock();
  // Read the sensor values
  //shadowHexReadSensors(&shadow_->dev,sensor_val);
  //shadow_mutex_.unlock();
  //sensor_msg_.header.stamp = ros::Time::now();    
  //target_msg_.header.stamp = ros::Time::now();
  positions_msg_.header.stamp = ros::Time::now();
  int pos[NUM_CHANNELS];
  for(i=0; i < NUM_CHANNELS; i++)
    {
      // Measure the time for this function!!!
      servoReportPosition(&servo_->dev, 0x00, &pos[i]);
      positions_msg_.pos[i] = 0; //pos[i];
    }
  // Put the values into a message
  //for(i=0; i < NUM_VALVES; i++)
  //  sensor_msg_.sensor[i] = sensor_val[i];

  // Put the values into a message
  //for(i=0; i < NUM_VALVES; i++)
  //  target_msg_.target[i] = set_target_[i];

   
  //publish the msgs
  //shadow_pub_.publish(sensor_msg_);
  //target_pub_.publish(target_msg_);
  position_pub_.publish(positions_msg_);

}



bool ServoClass::spin()
{
  //unsigned short  sensor_val[8];
  //ros::Rate r(10); // 10 ms or 100 Hz ??

  while (node_.ok())
    {
      if (publishing_)
	publish();
      ros::spinOnce();
      publish_rate_.sleep();
    }
  return true;
}


/*int main(int argc, char **argv)
{

  ros::init(argc, argv, "shadow");

  if (argc == 2)
    {
      std::string shadow_dev = argv[1];  
      ShadowNode s(shadow_dev);
      ROS_INFO("ShadowNode created");
      s.spin();
    }
  else
    {
      ShadowNode s;
      ROS_INFO("ShadowNode created");
      s.spin();
    }

  return 0;
  }*/
