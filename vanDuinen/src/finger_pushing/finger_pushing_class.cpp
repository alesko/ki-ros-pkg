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
#include <boost/thread/mutex.hpp>

// services
#include <shadow/SetController.h>
#include <shadow/GetStatus.h>
#include <shadow/GetSensors.h>
#include <shadow/SetTargets.h>
#include <shadow/DisableController.h>
#include <shadow/PulseValves.h>
#include <shadow/StartPublishing.h>
#include <shadow/SetValves.h>
#include <shadow/ShadowSensors.h>

#include "finger_pushing_class.h"

boost::mutex g_shadow_mutex ; //GCLOBAL
unsigned int g_sensordata[8];

// Callback function, not part of class!
void ShadowSensorMsgCallback(const boost::shared_ptr<const shadow::ShadowSensors> &msg)
{
  int i;
  ros::Time t = msg->header.stamp;
  
  //g_shadow_mutex.lock();
  for(i=0; i < 8;i++)
    g_sensordata[i] = msg->sensor[i];
  //g_shadow_mutex.unlock();

}

FingerPushing::FingerPushing():loop_rate_(100)
{
  //ros::init(argc, argv, "hiske_finger_push");
  // Just for testing
  //labjack_temperature_client_ = nh_.serviceClient<shadow::LJGetTemperature>("/labjack/temperature");
}

FingerPushing::~FingerPushing()
{

}

bool FingerPushing::init()
{
  // Initialization
  spcu_client_publisher_ = nh_.serviceClient<shadow::StartPublishing>("/shadow/publishing_service");
  spcu_client_targets_ = nh_.serviceClient<shadow::SetTargets>("/shadow/set_targets");
  spcu_client_controller_ = nh_.serviceClient<shadow::SetController>("/shadow/enable_controller");
  spcu_disable_controller_ = nh_.serviceClient<shadow::DisableController>("/shadow/disable_controller");
  spcu_client_valves_ = nh_.serviceClient<shadow::SetValves>("/shadow/set_valves");
 
  

  // Call the service to start publishing 
  spcu_publishing_state_.request.start = true;
  if (spcu_client_publisher_.call(spcu_publishing_state_))
    ROS_INFO("Shadow publishing service started" );
  else
    ROS_ERROR("Failed to call service shadow publishing service");
  
  // Start callback to monitor sensor values
  spcu_shadow_sensor_sub_ = nh_.subscribe("/shadow/sensor_msg",5,ShadowSensorMsgCallback);

  return true;
}

bool FingerPushing::fill_pam()
{
  spcu_valve_states_.request.valve0_state = 1; // Open valve 1 = empty PAM
  spcu_client_valves_.call(spcu_valve_states_);
  sleep(2); 
  spcu_valve_states_.request.valve0_state = 0; // Close valve 1 now it sould be empty 
  spcu_client_valves_.call(spcu_valve_states_);
  return true;
}

bool FingerPushing::empty_pam()
{
  spcu_valve_states_.request.valve1_state = 1; // Open valve 1 = empty PAM
  spcu_client_valves_.call(spcu_valve_states_);
  sleep(2); 
  spcu_valve_states_.request.valve1_state = 0; // Close valve 1 now it sould be empty 
  spcu_client_valves_.call(spcu_valve_states_);
  return true;
}

bool FingerPushing::wait_button_push(int number, double delay)
{
  bool run = true;
  double running_time;
  double start_time = ros::Time::now().toSec();

  while( run )
    {
      loop_rate_.sleep();
      ros::spinOnce();
      running_time = ros::Time::now().toSec() - start_time;      
      if( running_time > delay )
	{
	  if( g_sensordata[number] > 2000  )
	    run = false;
	}
    }
	  
  ROS_INFO("Button pushed...");
  loop_rate_.sleep();
  ros::spinOnce();
  
  return true;
}

double FingerPushing::record_sensor_data(int sensor,int number_of_recordings)
{
  int i;
  double sum=0;
  double mean;

  for(i = 0; i < number_of_recordings;i++)
    {
      sum = sum + (double) g_sensordata[sensor];
      loop_rate_.sleep();
      ros::spinOnce();
    }
  mean = (sum/number_of_recordings);
  ROS_INFO("Sum %f, mean %f",sum, mean);
  return mean;

}

int FingerPushing::record_max_sensor_data_time(int sensor,float duration)
{

  double start_time = ros::Time::now().toSec();
  bool run = true;
  int max = 0;
  int sensor_val;
  double running_time; 

  
  while( run )
    {
      loop_rate_.sleep();
      ros::spinOnce();
      sensor_val = g_sensordata[sensor];
      if ( max < sensor_val )
	max = sensor_val; 
      running_time = ros::Time::now().toSec() - start_time;
      if( running_time > duration )
	run = false;

    }

  return max;
  
}

bool FingerPushing::record_maxmin_sensor_data_time(int sensor,float duration, int& min, int& max)
{

  double start_time = ros::Time::now().toSec();
  bool run = true;
  int l_max = 0;
  int l_min = 100000;
  int sensor_val;
  double running_time; 
  
  while( run )
    {
      loop_rate_.sleep();
      ros::spinOnce();
      sensor_val = g_sensordata[sensor];
      if ( l_max < sensor_val )
	l_max = sensor_val;
      if ( l_min > sensor_val )
	l_min = sensor_val;
      running_time = ros::Time::now().toSec() - start_time;
      if( running_time > duration )
	run = false;
    }
  min = l_min;
  max = l_max;

  return true;
  
}


//bool measure_baseline(int sensor, int current_baseline)
bool FingerPushing::measure_baseline(int sensor, int current_baseline, double tol)
{
  double start_time = ros::Time::now().toSec();
  double duration = 1.0;
  bool run = true;
  double running_time; 
  double sum = 0;
  double mean;
  int i=0;
  if( tol > 1.0 )
    {
      ROS_ERROR("Tolerance parameter in function measure_baseline is too high!");
      return false;
    }
  if( tol < 0.0 )
    {
      ROS_ERROR("Tolerance parameter in function measure_baseline is too low!");
      return false;
    }

  while( run )
    {
      loop_rate_.sleep();
      ros::spinOnce();
      sum = sum + (double) g_sensordata[sensor];
      i++;

      running_time = ros::Time::now().toSec() - start_time;
      if( running_time > duration )
	run = false;

    }

  mean = sum/i;
  
  if( mean > ( (1.0 + tol) * (double)current_baseline) )
    {
      ROS_WARN("Baseline seems to have increased");
      return false;
    }
  if( mean < ( (1.0-tol) * (double)current_baseline) )
    {
      ROS_WARN("Baseline seems to have decreased");
      return false;
    }

  return true;
  
}
/*
//bool measure_baseline(int sensor, int current_baseline)
double FingerPushing::measure_baseline(int sensor, int current_baseline)
{
  double start_time = ros::Time::now().toSec();
  double duration = 1.0;
  bool run = true;
  double running_time; 
  double sum = 0;
  double mean;
  int i=0;

  while( run )
    {
      loop_rate_.sleep();
      ros::spinOnce();
      sum = sum + (double) g_sensordata[sensor];
      i++;

      running_time = ros::Time::now().toSec() - start_time;
      if( running_time > duration )
	run = false;

    }

  mean = sum/i;
  
  
  if( mean > ( (1.0 + tol) * (double)current_baseline) )
    {
      ROS_WARN("Baseline seems to have increased");
      return false;
    }
  if( mean < ( (1.0-tol) * (double)current_baseline) )
    {
      ROS_WARN("Baseline seems to have decreased, mean %");
      return false;
    }
  
  return mean;
  
}
*/

bool FingerPushing::set_controller(int controller, int sensor, int p, int i, int d)
{
  int j;
  for(j=0; j < 8; j++)
    spcu_target_values_.request.target[j]= 0;

  spcu_controller_values_.request.Controller_valve  = controller;//AIRMUSCLE_FILL_VALVE;       
  spcu_controller_values_.request.Controller_sensor = sensor; //FLEXIFORCE_AIRMUSCLE;
  spcu_controller_values_.request.Controller_P = p;
  spcu_controller_values_.request.Controller_I = i;
  spcu_controller_values_.request.Controller_D = d;
  
  if (spcu_client_controller_.call(spcu_controller_values_))
    {
      //ROS_INFO("PID contoller %d enabled", controller);//AIRMUSCLE_FILL_VALVE);
      return true;
    }
  else
    {
      ROS_ERROR("Failed to enable PID controller %d", controller);
      return false;
    }
}

bool FingerPushing::set_target(int controller, int target, int tol)
{
  spcu_target_values_.request.target[controller] = target - tol;
  spcu_target_values_.request.target[controller+1]= target +tol;
  if( spcu_client_targets_.call(spcu_target_values_) )
    {     
      return true;
    }
  else
    {
      ROS_ERROR("Failed to set target");
      return false;
    }


}

bool FingerPushing::disable_controller(int controller)
{
  // Disable the controllers
  spcu_controller_disable_.request.Controller_valve = controller; //AIRMUSCLE_FILL_VALVE;
  if (spcu_disable_controller_.call(spcu_controller_disable_))
    {
      //ROS_INFO("Controller %d disabled", controller);
      return true;
    }
  else
    {
      ROS_ERROR("Failed to disable controller %d", controller);
      return false;
    }
	  
}

void FingerPushing::interprete_case(unsigned int test_case, int& type, double& scale)
{
  switch (test_case)
    {
    case 0:
      type = 0;
      scale = 0.1;
      break;
    case 1:
      type = 0;
      scale = 0.3;
      break;
    case 2:
      type = 0;
      scale = 0.5;
      break;
    case 3:
      type = 1;
      scale = 0.1;
      break;
    case 4:
      type = 1;
      scale = 0.3;
      break;
    case 5:
      type = 1;
      scale = 0.5;
      break;
    case 6:
      type = 2;
      scale = 0.1;
      break;
    case 7:
      type = 2;
      scale = 0.3;
      break;
    case 8:
      type = 2;	  
      scale = 0.5;
      break;
    } 
}

/*
double FingerPushing::get_temperature(int ain_ch, int curr_n)
{
  double t;
  // Resistance voltage conversion table from 
  // http://www.advindsys.com/ApNotes/YSI400SeriesProbesRvsT.htm

  double resistance[35] = {4273, 4074, 3886, 3708, 3539, 3378, 3226, 
			   3081, 2944, 2814, 2690, 2572, 2460, 2354, 
			   2252, 2156, 2064, 1977, 1894, 1815, 1739,
			   1667, 1599, 1533, 1471, 1412, 1355, 1301,
			   1249, 1200, 1152, 1107, 1064, 1023, 983.8 };
  double temp[35] = {11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,
		     28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45};
  int i=0;
  // Call the service to start publishing 
  labjack_temperature_.request.ain_number = ain_ch;     // Temperature sensor is on channel AIN3
  labjack_temperature_.request.current_number = curr_n; // Temperature sensor supplied with 200 uA
  if (!labjack_temperature_client_.call(labjack_temperature_))
    ROS_ERROR("Failed to call labjack temperature service");
  double temp_res = labjack_temperature_.response.temp_res;

  
  if( temp_res > 42733 )
    {
      ROS_WARN("Temp too low! Below 11 deg");
      return 0;
    }
  if( temp_res < 1023 )
    {
      ROS_WARN("Temp too high! Above 45 deg");
      return 0;
    }

  while(temp_res < resistance[i])
    i++;
 
  // Picse wise linear approximation:
  t=((temp_res-resistance[i])*(temp[i+1]-temp[i])/(resistance[i+1]-resistance[i]))+temp[i];
  //ROS_INFO("Res %f, i=%d  %f %f temp %f",meas_res,i,resistance[i],resistance[i+1], t);

  return t;
  
}
*/
