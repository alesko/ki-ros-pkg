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

//#include <vector>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
//#include <tf/transform_broadcaster.h>



#include <shadow_base.h>
#include <shadow_commands.h>
#include <shadow_io.h>

#include "shadow_node.h"

// messages
#include <shadow/Sensors.h>
#include <shadow/Valves.h>

// services
#include <shadow/GetStatus.h>
#include <shadow/GetSensors.h>
#include <shadow/SetTargets.h>
#include <shadow/PulseValves.h>
#include <shadow/StartPublishing.h>


ShadowNode::ShadowNode(std::string dev) : private_nh("~"), publish_rate(60)
{

  int msg_que_len = 5;    

  ROS_INFO("Initializing Shadow SPCU");
    
  shadow_ = shadowInitialize();

  strcpy(shadow_->dev.ttyport, dev.c_str());
    
  if (shadowDeviceConnectPort(&shadow_->dev) < 0) {
    ROS_FATAL("Unable to connect shadow at %s\n", shadow_->dev.ttyport);
    private_nh.shutdown();
    return;   
  }
  else{
    ROS_INFO("Connected to device");
    //publishes sensor readings
    std::string prefix;
    std::string searched_param;
    private_nh.searchParam("shadow_prefix", searched_param);
    private_nh.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "/shadow/sensor_msg";
    ROS_INFO("Starting publisher!");
    shadow_pub = private_nh.advertise<shadow::ShadowSensors>(full_topic,msg_que_len );
  }
    
  // ***** Parameters *****

  ROS_INFO("Retrieving module state");
  shadowHexStatus(shadow_);  
  shadowPrintParams(&shadow_->par);
     
  //sensor_reading_pub_ = private_nh.advertise<shadow::Sensors>("sensors_pub", 100);

  ROS_INFO("Starting services!");
  system_status_srv_ = private_nh.advertiseService("get_status", &ShadowNode::getStatus,this);
  set_valves_srv_ = private_nh.advertiseService("set_valves", &ShadowNode::setValves,this);
  pulse_valves_srv_ = private_nh.advertiseService("pulse_valves", &ShadowNode::pulseValves,this);
  sensor_reading_srv_ = private_nh.advertiseService("get_sensor_readings", &ShadowNode::getSensorReading,this);
  targets_srv_ = private_nh.advertiseService("set_targets", &ShadowNode::setTargets,this);
  contoller_srv_ = private_nh.advertiseService("enable_controller", &ShadowNode::setController,this);
  contoller_target_srv_ = private_nh.advertiseService("enable_controller_target", &ShadowNode::setControllerwTarget,this);
  disable_contoller_srv_ = private_nh.advertiseService("disable_controller", &ShadowNode::disController,this);
  publishing_srv_ = private_nh.advertiseService("publishing_service", &ShadowNode::setPublishing,this);

  publishing = false;
  ROS_INFO("Shadow SPCU is ready!");

}

ShadowNode::~ShadowNode()
{
  ROS_INFO("Closing SPCU" );
  shadowClear(shadow_);
  ROS_INFO("SPCU closed, goodbye" );

}

bool ShadowNode::setValves(shadow::GetSensors::Request& req, shadow::GetSensors::Response& resp)
{   
  return true;
}

bool ShadowNode::pulseValves(shadow::PulseValves::Request& req, shadow::PulseValves::Response& resp)
{   
  int i;
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

bool ShadowNode::setController(shadow::SetController::Request& req, shadow::SetController::Response& resp)
{   
  
  /*ROS_INFO("Got: v=%d, s=%d, P=%d, I=%d, D=%d",(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
    (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);*/
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

bool ShadowNode::setControllerwTarget(shadow::SetControllerwTarget::Request& req, shadow::SetControllerwTarget::Response& resp)
{   
  
  /*ROS_INFO("Got: v=%d, s=%d, P=%d, I=%d, D=%d",(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
    (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);*/
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
  
bool ShadowNode::setTargets(shadow::SetTargets::Request& req, shadow::SetTargets::Response& resp)
{
  // TODO: check why this function makes the hardware "hang"
  // Impossible to retrive the status after the target is set
  
  int  set_target[NUM_VALVES];
  int i;
  for(i=0; i < NUM_VALVES; i++)
    set_target[i]= req.target[i];
  for(i=0; i < NUM_VALVES; i++)
    ROS_INFO("SHADOW: Setting targets:[%d]=%d",i,set_target[i]);
 
  ROS_INFO("SHADOW: Setting targets");

  shadow_mutex_.lock();
  //shadowHexSetTargets(&shadow_->dev,set_target);
  shadowAsciiSetTargets(&shadow_->dev,set_target);
  shadow_mutex_.unlock();


  return true;
}

bool ShadowNode::disController(shadow::DisableController::Request& req, shadow::DisableController::Response& resp)
{   
 
  shadow_mutex_.lock();
  shadowHexDisableController(&shadow_->dev, req.Controller_valve);
  shadow_mutex_.unlock();
  ROS_INFO("SHADOW: Disabled controller for valve: %d",
	   (unsigned short)req.Controller_valve);

  return true;
}

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


bool ShadowNode::setPublishing(shadow::StartPublishing::Request& req, shadow::StartPublishing::Response& resp)
{
  if(req.start){
    ROS_INFO("SPCU is now publishing sensor data");
    publishing = true;
    resp.state = true;
    //this->pub->setPublishing(true);    
  }
  else{
    ROS_INFO("SPCU has stopped publishing");
    publishing = false;
    resp.state = false;
    //this->pub->setPublishing(false);
  }
  //this->pub->cyberglove_pub.shutdown();
  return true;
}

bool  ShadowNode::isPublishing()
{
  if (publishing)
    {
      return true;
    }
  else
    {
      //ros::spinOnce();
      //publish_rate.sleep();
      return false;
    }
}

void ShadowNode::publish()
{
 
  int i;
  unsigned short  sensor_val[8];
  shadow_mutex_.lock();
  // Read the sensor values
  shadowHexReadSensors(&shadow_->dev,sensor_val);
  shadow_mutex_.unlock();
  sensor_msg.header.stamp = ros::Time::now();    
  // Put the values into a message
  for(i=0; i < NUM_VALVES; i++)
    sensor_msg.sensor[i] = sensor_val[i];
   
  //publish the msgs
  shadow_pub.publish(sensor_msg);
  //ros::spinOnce();
  //publish_rate.sleep();

}



bool ShadowNode::spin()
{
  //unsigned short  sensor_val[8];
  //ros::Rate r(10); // 10 ms or 100 Hz ??

  while (node_.ok())
    {
      if (publishing)
	publish();
      /*{
	  publish();
	}
      else
	{
	  ros::spinOnce();
	  //publish_rate.sleep();
	}*/
      ros::spinOnce();
      publish_rate.sleep();
    }
  return true;
}


int main(int argc, char **argv)
{

  if (argc != 2)
    {
      printf("Usage: shadow_node DEVICE\n");
      return 1;
    }
  std::string shadow_dev = argv[1];  

  ros::init(argc, argv, "shadow");
  ShadowNode s(shadow_dev);
  ROS_INFO("ShadowNode created");

  s.spin();
  return 0;
}
