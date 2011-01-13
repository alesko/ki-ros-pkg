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

#ifndef SHADOW_PID
#define SHADOW_PID

/*
//#include <vector>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <string>

#include <ros/ros.h>

//#include <tf/transform_broadcaster.h>

#include <shadow_base.h>
#include <shadow_commands.h>
#include <shadow_io.h>
*/

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

#include <control_toolbox/pid.h>

// messages
#include <shadow/ShadowSensors.h>
#include <shadow/Valves.h>
#include <shadow/ShadowSensors.h>

// services
#include <shadow/SetController.h>
#include <shadow/GetStatus.h>
#include <shadow/GetSensors.h>
#include <shadow/SetTargets.h>
//#include <shadow/DisableController.h>
#include <shadow/PulseValves.h>
#include <shadow/StartPublishing.h>
//#include <cyberglove/Start.h>
#include <dataglove/Start.h>

#include <control_toolbox/pid.h>


#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

class ShadowPID
{
 private:
  boost::mutex shadow_mutex_ ; //GCLOBAL
  boost::mutex glove_mutex_ ; //GCLOBAL
  unsigned int sensordata_[8];
  float glovedata_[18];

  //ros::NodeHandle node_;
  
  //shadow_spcu_p shadow_;

  //ros node handle
  ros::NodeHandle n_tilde;
  //ros::NodeHandle private_nh; //("~");

  ros::NodeHandle nh;  
  ros::ServiceClient client_pub;
  ros::ServiceClient client_valve;
  ros::ServiceClient client_glove;
  ros::ServiceClient client_target;
  ros::ServiceClient client_controller;
  
  ros::Subscriber shadow_sensor_sub;
  ros::Subscriber glove_sub;
  ros::Publisher setpoint_pub_;

  shadow::StartPublishing publishing_state_;
  shadow::PulseValves valve_states_;
  //cyberglove::Start glove_publishing_state_;
  dataglove::Start glove_publishing_state_;
  shadow::SetTargets target_values;
  shadow::SetController control_val;

  ros::Rate loop_rate;

  // Controller stuff
  control_toolbox::Pid pid_controller_[8];
  ros::Time time_of_last_cycle_;
  ros::ServiceServer controller_srv_;

  // Internal variables

  shadow::ShadowSensors sensor_msg;
  shadow::Valves valve_states;
  ros::Time last_time;
  double control_loop_rate;


 public:
  double set_point[4];
  double set_point_dir;
  double track_point;
  shadow::ShadowSensors setpoint_msg;

  void alterSetPoint();
  //void publishSetPoint();

  ShadowPID(); //double P,double I,double D,double I1,double I2);
  ~ShadowPID();

  void initPID(double P, double I, double D, double I1, double I2);

  //void ShadowPID::ShadowSensorMsgCallback(const boost::shared_ptr<const shadow::ShadowSensors> &msg);
  void ShadowSensorMsgCallback(const boost::shared_ptr<const shadow::ShadowSensors> &msg);
  void GloveSensorMsgCallback(const boost::shared_ptr<const sensor_msgs::JointState> &msg);
  //void ShadowSensorMsgCallback(const shadow::ShadowSensors &msg);
  //bool spin();  
  void controlspin();
  
  // Contoller commands
  //bool controllerInit();
  void initCallback();
  void initGloveCallback();
  void controllerStarting();
  void controllerUpdate(double position_desi_[4]);
  void setTarget(double position_desi_[4]);
  void controllerStopping();


};

#endif
