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

#include "finger_pushing_class.h"

#define AIRMUSCLE_FILL_VALVE       0
#define AIRMUSCLE_FILL_CONTROLLER  0
#define AIRMUSCLE_EMPTY_VALVE      1
#define AIRMUSCLE_EMPTY_CONTROLLER 1

#define AIRPRESSURE_SENSOR         0
#define FLEXIFORCE_FINGER          1 
#define FLEXIFORCE_AIRMUSCLE       2 
#define RIGHT_FINGER_ANGLE         3
#define PUSH_BUTTON                7 


int main(int argc, char **argv)
{
  int baseline_force;
  int force_pull_max = 0;
  int force_push_max = 0;
  int max_force;

  
  ros::init(argc, argv, "hiske_finger_push");
  ros::start();

  FingerPushing finger_pushing;

  finger_pushing.init();
  finger_pushing.empty_pam();

  ROS_INFO("Baseline force will now be measured. Loosen screws, relax and push the button to start");
  finger_pushing.wait_button_push(PUSH_BUTTON, 0.0);

  // Record baseline
  baseline_force = (int) finger_pushing.record_sensor_data( FLEXIFORCE_AIRMUSCLE, 100);
  ROS_INFO("Baseline force %d",baseline_force);
  const int bf=baseline_force;
  finger_pushing.nh_.setParam("/shadow/baseline_force", bf);  
  ros::spinOnce();

  // Get valuse from parameter server
  if (finger_pushing.nh_.hasParam("/shadow/max_pull_force"))
    finger_pushing.nh_.getParam("/shadow/max_pull_force", force_pull_max); 
  else
    force_pull_max=0;

  if (finger_pushing.nh_.hasParam("/shadow/max_push_force"))
    finger_pushing.nh_.getParam("/shadow/max_push_force", force_push_max); 
  else
    force_push_max=0;

  // Record max push force 
  ROS_INFO("Max force will now be measured");
  ROS_INFO("Loosen screws and KEEP rubber bands, then push the button to start");
  finger_pushing.wait_button_push(PUSH_BUTTON, 0.0);
  max_force = finger_pushing.record_max_sensor_data_time(FLEXIFORCE_AIRMUSCLE,6.0000);
  if( force_pull_max < max_force)
    {
      force_pull_max = max_force;
      ROS_INFO("New /shadow/max_pull_force on paramter server %d", force_pull_max);
    }
  else
    {
      ROS_INFO("Keeping old /shadow/max_pull_force on paramter server %d", force_pull_max);
    }
  int const fmpull = force_pull_max;
  // Put value on parameter server
  finger_pushing.nh_.setParam("/shadow/max_pull_force", fmpull);

  ros::spinOnce();

  // Record max push force 
  ROS_INFO("Max push force will now be measured");
  ROS_INFO("Fixate the pushing screw (KEEP rubberbands) and push the button to start");
  finger_pushing.wait_button_push(PUSH_BUTTON, 0.0);
  max_force = finger_pushing.record_max_sensor_data_time(FLEXIFORCE_AIRMUSCLE,6.0000);
  if( force_push_max < max_force)
    {
      force_push_max = max_force;
      ROS_INFO("New /shadow/max_push_force on paramter server %d", force_push_max);
    }
  else
    {
      ROS_INFO("Keeping old /shadow/max_push_force on paramter server %d", force_push_max);
    }
  int const fmpush = force_push_max;
  // Put value on parameter server
  finger_pushing.nh_.setParam("/shadow/max_push_force", fmpush);

  ROS_INFO("End of initialization");
  return 0;

}
