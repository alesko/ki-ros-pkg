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
#include <fstream>
#include <iostream>
#include <sys/time.h>
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
  double force_push_max = 0;
  int number_of_trials = 4;
  std::vector<unsigned int> test_case_code;
  int type;
  double scale;
  unsigned int len ;
  double exp_duration=0.1;
  int tol = 50;
  int playback_force;
  bool adaptive_baseline;
  double baseline_tolerance;
  //double baseline_tolerance;

  // Init ROS
  ros::init(argc, argv, "hiske_finger_push");

  // Create the object
  FingerPushing finger_pushing;
  std::ofstream data_file; // Logfile

  // Init Shadow contoller and PAM
  finger_pushing.init();
  finger_pushing.empty_pam();

  // Open data file for printing
  time_t t = time(0);
  struct tm* lt = localtime(&t);
  char time_str[256];
  sprintf(time_str, "../data/datalog_%04d%02d%02d_%02d%02d%02d.txt",
          lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
          lt->tm_hour, lt->tm_min, lt->tm_sec);
 
  data_file.open(time_str);
  
  // Check for parameters on parameter server
  if (finger_pushing.nh_.hasParam("/shadow/max_pull_force"))
    {
      finger_pushing.nh_.getParam("/shadow/max_pull_force", force_max); 
      ROS_INFO("/shadow/max_pull_force is %f", force_max );
      data_file << "/shadow/max_pull_force is " << force_max << std::endl;
    }
  else
    {
      ROS_ERROR("/shadow/max_pull_force does not exits!");
      ROS_ERROR("Run the finger_pushing_ini program!");
      exit(0);
    }
  if (finger_pushing.nh_.hasParam("/shadow/adaptive_baseline"))
    {
      finger_pushing.nh_.getParam("/shadow/adaptive_baseline", adaptive_baseline);
      if (true == adaptive_baseline)
	{
	  ROS_INFO("/shadow/adaptive_baseline will be used");
	  data_file << "/shadow/adaptive_baseline true" << std::endl;
	  if (finger_pushing.nh_.hasParam("/shadow/baseline_force"))
	    {
	      finger_pushing.nh_.getParam("/shadow/baseline_force", baseline_force); 
	      ROS_INFO("Using /shadow/baseline_force %f as initial baseline force", baseline_force );
	    }
	  else
	    {
	      // New measure!!!
	    }
	}
      else
	{
	  ROS_INFO("/shadow/adaptive_baseline will NOT be used");
	  data_file << "/shadow/adaptive_baseline false" << std::endl;
	}
    }
  else
    {
      ROS_WARN("/shadow/adaptive_baseline not found!");
      if (finger_pushing.nh_.hasParam("/shadow/baseline_force"))
	{
	  finger_pushing.nh_.getParam("/shadow/baseline_force", baseline_force); 
	  ROS_INFO("/shadow/baseline_force %f", baseline_force );
	  data_file << "/shadow/baseline_force " << baseline_force << std::endl;

	  if (finger_pushing.nh_.hasParam("/shadow/baseline_tolerance"))
	    {
	      finger_pushing.nh_.getParam("/shadow/baseline_tolerance", baseline_tolerance);
	      if( (baseline_tolerance > 1.0)||(baseline_tolerance < 0.0) )
		{
		  ROS_ERROR("/shadow/baseline_tolerance is out of bound");
		}
	      ROS_INFO("/shadow/baseline_tolerance %f", baseline_tolerance );
	      data_file << "/shadow/baseline_tolerance " << baseline_tolerance << std::endl;
	      
	    }
	  else
	    {
	      ROS_ERROR("Neither /shadow/baseline_force nor /shadow/adaptive_baseline does not exits!");
	      ROS_ERROR("Run the finger_pushing_ini program and/or set the /shadow/adaptive_baseline parameter!");
	     
	    }

	}
      else
	{
	  ROS_ERROR("Neither /shadow/baseline_force nor /shadow/adaptive_baseline does not exits!");
	  ROS_ERROR("Run the finger_pushing_ini program and/or set the /shadow/adaptive_baseline parameter!");
	  exit(0);
	}
    }
  if (finger_pushing.nh_.hasParam("/shadow/max_push_force"))
    {
      finger_pushing.nh_.getParam("/shadow/max_push_force", force_push_max); 
      ROS_INFO("/shadow/max_push_force is %f", force_push_max );
      data_file << "/shadow/max_push_force is " << force_push_max << std::endl;
    }
  else
    {
      ROS_WARN("/shadow/max_pull_force does not exits!");
      ROS_WARN("Run the finger_pushing_ini program!");
    }

  if (finger_pushing.nh_.hasParam("/shadow/number_of_trials"))
    finger_pushing.nh_.getParam("/shadow/number_of_trials", number_of_trials); 
  else
    ROS_INFO("Number of trials is not set on the parameter server, using default = %d",number_of_trials);    

  data_file << "------------------------------------" << std::endl;
  
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
          data_file << i << "\t"<< "Isometric force, scale " << scale << std::endl;
	  finger_pushing.wait_button_push(PUSH_BUTTON, 0.0);
	  finger_pushing.record_max_sensor_data_time(0,exp_duration);
	  break;
	case 1:
	  ROS_INFO("Dynamic force, scale %f\nLoosen the screw and push the button to start",scale);
          data_file << i << "\t"<< "Dynamic force, scale " << scale << std::endl;
	  finger_pushing.wait_button_push(PUSH_BUTTON, 0.0);
	  finger_pushing.record_max_sensor_data_time(0,exp_duration);
	  break;
	case 2:
	  ROS_INFO("Loosen the screw!\nActive force will now be produced at %.1f%c",100*scale, 0x25);
          data_file << i << "\t"<< "Active force, scale " << scale << std::endl;
	  ROS_INFO("Push the button to active PAM");
	  finger_pushing.wait_button_push(PUSH_BUTTON, 0.0);
	 
	  playback_force = (int) (scale * (force_max-baseline_force)  + baseline_force);
	  ROS_INFO("playback_force = scale * (force_max-baseline_force) + baseline_force: %d",playback_force);
	  ROS_INFO("Activation of PAM!");
	  
	  // Start the PID controllers on SPCU
	  finger_pushing.set_controller(AIRMUSCLE_FILL_VALVE, FLEXIFORCE_AIRMUSCLE, 4, 2, 0);
	  finger_pushing.set_controller(AIRMUSCLE_EMPTY_VALVE, FLEXIFORCE_AIRMUSCLE, -4, -2, 0);
	  finger_pushing.set_target(AIRMUSCLE_FILL_VALVE, playback_force, tol);

	  ROS_INFO("Push the button to stop");
	  finger_pushing.wait_button_push(PUSH_BUTTON, 2.0);
	  
	  // Disable the controllers
	  ROS_INFO("Dectivating PAM...");
	  finger_pushing.disable_controller(AIRMUSCLE_FILL_VALVE);
	  finger_pushing.disable_controller(AIRMUSCLE_EMPTY_VALVE);
	  finger_pushing.empty_pam();
	  ROS_INFO("Done!\n");
	  break;
	}
      test_case_code.pop_back(); // Remove this test case
      /*baseline_force = finger_pushing.measure_baseline(FLEXIFORCE_AIRMUSCLE,baseline_force);
      if (false == adaptive_baseline)
	{	  	  
	  if( baseline_force > ( (1.0 + baseline_tolerance) * (double)current_baseline) )
	    {
	      ROS_WARN("Baseline seems to have increased");
	      return false;
	    }
	  if( baseline_force < ( (1.0-baseline_tolerance) * (double)current_baseline) )
	    {
	      ROS_WARN("Baseline seems to have decreased, mean %");
	      return false;
	    }
	}
      else
      {*/
      if (false == adaptive_baseline)
	{
	  if(!finger_pushing.measure_baseline(FLEXIFORCE_AIRMUSCLE,baseline_force, baseline_tolerance))
	     ROS_WARN("Baseline seems to have changed");	     
	}
      else
	{
	  baseline_force = (int)finger_pushing.record_sensor_data(FLEXIFORCE_AIRMUSCLE, 100);
	  data_file << "Baseline force updated: " << "\t" << baseline_force << std::endl;
	  //current_baseline=(int)baseline_force;
	}
    }
  data_file.close();
  
}



