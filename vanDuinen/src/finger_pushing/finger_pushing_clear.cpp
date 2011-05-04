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
//#include "shadow_node.h"
//#include <gtk/gtk.h>
/*#include <stdio.h>
#include <unistd.h>
#include <pthread.h>*/

//#include "shadow_PID.h"
//#include <fstream>
//#include <cmath>
//#include <sys/time.h>

//#include <iostream>
//#include <iomanip>
//#include <ios_base>
//#include <algorithm> // sort, max_element, random_shuffle, remove_if, lower_bound 
//#include <cstdlib>
//#include <vector>
#include <ros/ros.h>
//#include <boost/thread/mutex.hpp>

// services
/*
#include <shadow/SetController.h>
#include <shadow/GetStatus.h>
#include <shadow/GetSensors.h>
#include <shadow/SetTargets.h>
#include <shadow/DisableController.h>
#include <shadow/PulseValves.h>
#include <shadow/StartPublishing.h>

#include <shadow/ShadowSensors.h>
*/
/*
boost::mutex g_shadow_mutex ; //GCLOBAL
unsigned int g_sensordata[8];

#define AIRMUSCLE_FILL_VALVE       0
#define AIRMUSCLE_FILL_CONTROLLER  0
#define AIRMUSCLE_EMPTY_VALVE      1
#define AIRMUSCLE_EMPTY_CONTROLLER 1

#define AIRPRESSURE_SENSOR         0
#define FLEXIFORCE_FINGER          1 //1
#define FLEXIFORCE_AIRMUSCLE       2 //2
#define RIGHT_FINGER_ANGLE         3
#define PUSH_BUTTON                7 //2
*/

int main(int argc, char **argv)
{


  ros::init(argc, argv, "hiske_finger_push");
  ros::NodeHandle nh;

  if (nh.hasParam("/shadow/max_push_force"))
    {
      if( nh.deleteParam("/shadow/max_push_force") )
	ROS_INFO("Param named '/shadow/max_push_force' was deleted");
    }

  if (nh.hasParam("/shadow/max_pull_force"))
    {
      if( nh.deleteParam("/shadow/max_pull_force") )
	ROS_INFO("Param named '/shadow/max_pull_force' was deleted");
    }

  if (nh.hasParam("/shadow/baseline_air_pressure"))
    {
      if( nh.deleteParam("/shadow/baseline_air_pressure"))
	ROS_INFO("Param named '/shadow/baseline_air_pressure' was deleted");
    }

  if (nh.hasParam("/shadow/baseline_force"))
    {
      if( nh.deleteParam("/shadow/baseline_force"))
	ROS_INFO("Param named '/shadow/baseline_force' was deleted");
    }
  
  if (nh.hasParam("/shadow/empty_pressure"))
    {
      if( nh.deleteParam("/shadow/empty_pressure") )
	ROS_INFO("Param named '/shadow/empty_pressure' was deleted");
    }

  return 0;

}
