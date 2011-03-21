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

#include <algorithm> // sort, max_element, random_shuffle, remove_if, lower_bound 
#include <cstdlib>
#include <vector>
#include <ros/ros.h>
//#include <boost/thread/mutex.hpp>

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
  int i,j;
  double baseline_force=0;
  double force_max = 0;
  int number_of_trials = 4;
  std::vector<unsigned int> test_case_code;
  int type;
  double scale;
  unsigned int len ;
  double exp_duration=0.1;
  int tol = 50;
  int playback_force;

  // Init ROS
  ros::init(argc, argv, "hiske_finger_push");

  // Create the object
  FingerPushing finger_pushing;

  // Init Shadow contoller and PAM
  finger_pushing.init();
  finger_pushing.empty_pam();
  
  // Check for parameters on parameter server
  if (finger_pushing.nh_.hasParam("/shadow/max_pull_force"))
    {
      finger_pushing.nh_.getParam("/shadow/max_pull_force", force_max); 
      ROS_INFO("/shadow/max_pull_force is %f", force_max );
    }
  else
    {
      ROS_ERROR("/shadow/max_pull_force does not exits!");
      ROS_ERROR("Run the finger_pushing_ini program!");
      exit(0);
    }

  if (finger_pushing.nh_.hasParam("/shadow/baseline_force"))
    {
      finger_pushing.nh_.getParam("/shadow/baseline_force", baseline_force); 
      ROS_INFO("/shadow/baseline_force %f", baseline_force );
    }
  else
    {
      ROS_ERROR("/shadow/baseline_force does not exits!");
      ROS_ERROR("Run the finger_pushing_ini program!");
      exit(0);
    }
 

  if (finger_pushing.nh_.hasParam("/shadow/number_of_trials"))
    finger_pushing.nh_.getParam("/shadow/number_of_trials", number_of_trials); 
  else
    ROS_INFO("Number of trials is not set on the parameter server, using default = %d",number_of_trials);    

  // Generate the vector with test cases in radom order
  for(i =0; i < number_of_trials;i++)
    {
      for( j =0; j < 9;j++)
	{
	  test_case_code.push_back(j);
	}
    }
  srand((unsigned)time(NULL)); // Seed random function
  random_shuffle( test_case_code.begin(), test_case_code.end());

  // Start the experimets
  len = test_case_code.size();
  for(i = len; i > 0; i--)
    {
      // Take one test case
      finger_pushing.interprete_case(test_case_code[i-1], type, scale);

      switch (type)
	{
	case 0:
	  ROS_INFO("Isometric force, scale %f\nFixate the screw and push the button to start",scale);
	  finger_pushing.wait_button_push(PUSH_BUTTON, 0.0);
	  finger_pushing.record_max_sensor_data_time(0,exp_duration);
	  break;
	case 1:
	  ROS_INFO("Dynamic force, scale %f\nLoosen the screw and push the button to start",scale);
	  finger_pushing.wait_button_push(PUSH_BUTTON, 0.0);
	  finger_pushing.record_max_sensor_data_time(0,exp_duration);
	  break;
	case 2:
	  ROS_INFO("Loosen the screw!\nActive force will now be produced at %.1f%c",100*scale, 0x25);
	  ROS_INFO("Push the button to active PAM");
	  finger_pushing.wait_button_push(PUSH_BUTTON, 0.0);
	 
	  playback_force = (int) (scale * (force_max-baseline_force)  + baseline_force);
	  ROS_INFO("playback_force = scale * (force_max-baseline_force) + baseline_force: %d",playback_force);
	  ROS_INFO("Activation PAM!");
	  
	  // Start the PID controllers on SPCU
	  finger_pushing.set_controller(AIRMUSCLE_FILL_VALVE, FLEXIFORCE_AIRMUSCLE, 4, 2, 0);
	  finger_pushing.set_controller(AIRMUSCLE_EMPTY_VALVE, FLEXIFORCE_AIRMUSCLE, -4, -2, 0);
	  finger_pushing.set_target(AIRMUSCLE_FILL_VALVE, playback_force, tol);

	  ROS_INFO("Push the button to stop");
	  finger_pushing.wait_button_push(PUSH_BUTTON, 2.0);
	  
	  // Disable the controllers
	  ROS_INFO("Dectivation PAM...");
	  finger_pushing.disable_controller(AIRMUSCLE_FILL_VALVE);
	  finger_pushing.disable_controller(AIRMUSCLE_EMPTY_VALVE);
	  finger_pushing.empty_pam();
	  ROS_INFO("Done!\n");
	  break;
	}
      test_case_code.pop_back(); // Remove this test case
      if( !finger_pushing.measure_baseline(FLEXIFORCE_AIRMUSCLE,baseline_force) )
	ROS_WARN("Baseline seems to have changed");
    }
}
