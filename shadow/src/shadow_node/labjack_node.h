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
#ifndef LABJACK_NODE
#define LABJACK_NODE


#include <u6.h>
//#include <labjackusb.h>

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
//#include <shadow_base.h>

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>


// messages
/*#include <shadow/ShadowSensors.h>
#include <shadow/Valves.h>
#include <shadow/ShadowTargets.h>

// services
#include <shadow/SetController.h>
#include <shadow/SetControllerwTarget.h>
#include <shadow/GetStatus.h>
#include <shadow/GetSensors.h>

#include <shadow/DisableController.h>
#include <shadow/PulseValves.h>
#include <shadow/SetValves.h>  */

#include <shadow/LJSetTargets.h>
#include <shadow/StartPublishing.h> //StartPublishing is a service, .h is generated
#define NUM_VALVES 20

class LabjackNode
{
 private: //Only class can operate below

  ros::NodeHandle node_; //Underscore == class parameters
  boost::mutex labjack_mutex_ ;
  //shadow_spcu_p shadow_;

  std::string prefix_; // For parameter server

  ros::NodeHandle private_nh_; //("~");
  std::string device;
  
  bool publishing_;
  ros::Rate publish_rate_;
  int number_of_valves_;
  ros::Time last_update_;
  /*ros::Publisher shadow_pub_;
  ros::Publisher valve_state_pub_;
  ros::Publisher target_pub_;
  ros::Publisher sensor_reading_pub_;

  ros::ServiceServer sensor_reading_srv_;  
  ros::ServiceServer system_status_srv_;
  ros::ServiceServer set_valves_srv_;
  ros::ServiceServer pulse_valves_srv_;*/
  //ros::ServiceServer contoller_srv_;
  ros::ServiceServer contoller_target_srv_;
  ros::ServiceServer disable_contoller_srv_;
  ros::ServiceServer targets_srv_;
  ros::ServiceServer publishing_srv_;

  // SPCU commands
  /*bool setValves(shadow::SetValves::Request& req, shadow::SetValves::Response& resp);
  bool pulseValves(shadow::PulseValves::Request& req, shadow::PulseValves::Response& resp);
  bool getSensorReading(shadow::GetSensors::Request& req, shadow::GetSensors::Response& resp);*/
  //bool setController(shadow::SetController::Request& req, shadow::SetController::Response& resp);
  //bool setControllerwTarget(shadow::SetControllerwTarget::Request& req, shadow::SetControllerwTarget::Response& resp);
  bool setTargets(shadow::LJSetTargets::Request& req, shadow::LJSetTargets::Response& resp);
  //bool disController(shadow::DisableController::Request& req, shadow::DisableController::Response& resp);
  //bool getStatus(shadow::GetStatus::Request& req, shadow::GetStatus::Response& resp);
  
  bool setPublishing(shadow::StartPublishing::Request &req, shadow::StartPublishing::Response &resp);
  
  // Controller stuff
  /*control_toolbox::Pid pid_controller_;
  ros::Time time_of_last_cycle_;
  ros::ServiceServer controller_srv_;*/

  // Interna variables
  /*shadow::ShadowSensors sensor_msg_;
  shadow::ShadowTargets target_msg_;
  shadow::Valves valve_states;

  int  set_target_[NUM_VALVES];
  */
  double target_values_[NUM_VALVES];
  bool getSensorReading(void);
  bool updateValves();

  // LabJack 
  HANDLE            h_device_;
  u6CalibrationInfo cali_info_;
  int               local_ID_;
  long              error_;
  bool do_state_;

 public:
  LabjackNode(); //Constructor
  LabjackNode(std::string dev); //Constuctor with args
  ~LabjackNode(); //Destructor
  void init();

  bool spin();
  
  bool getNodeStateOK();
  ros::Rate getPublishRate();
  void publish();
  bool isPublishing();


  // Contoller commands
  /*bool controllerInit();
  void controllerStarting();
  void controllerUpdate();
  void controllerStopping();*/


};

#endif