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

//#include <shadow/ShadowSensors.h>

 
#include "LJ_temp_measure_class.h"

// Gobal variables
// 
boost::mutex g_labjack_mutex ; //GCLOBAL
double g_ain_data[14];
ros::Time g_time;

// Callback function, not part of class!
void LabjackMsgCallback(const boost::shared_ptr<const labjack::Sensors> &msg)
{
  int i;

  g_labjack_mutex.lock();
  g_time = msg->header.stamp;  
  for(i=0; i < 14;i++)
    {
      g_ain_data[i] = msg->ain[i];
      //g_data_file << g_time - time_.toSec() << "\t" << volt2temperature(g_ain_data[3],true) << std::endl;
    }
  g_labjack_mutex.unlock();
  //ROS_INFO("%lf",g_ain_data[3]);
  //ain_msg_.header.stamp = ros::Time::now();

}
/*void TemperatureMeasure::LabjackMsgCallback(const boost::shared_ptr<const labjack::Sensors> &msg)
{
  int i;

  g_labjack_mutex.lock();
  g_time = msg->header.stamp;  
  for(i=0; i < 14;i++)
    {
      g_ain_data[i] = msg->ain[i];
      //g_data_file << g_time - time_.toSec() << "\t" << volt2temperature(g_ain_data[3],true) << std::endl;
    }
  g_labjack_mutex.unlock();
  //ROS_INFO("%lf",g_ain_data[3]);
  //ain_msg_.header.stamp = ros::Time::now();

  }*/

// Current channel is set to one or 2
TemperatureMeasure::TemperatureMeasure(int cur_n, int ch_start, int ch_num):loop_rate_(1)
//TemperatureMeasure::TemperatureMeasure():loop_rate_(5)
{
 
  current_channel_ = cur_n;
  starting_channel_ = ch_start;
  number_of_channels_ = ch_num;

}

bool TemperatureMeasure::init(void)
{
  std::string the_path;

  ROS_INFO("Creating a temperature node");

  // TODO: make callback part of class
  //TemperatureMeasure object_copy(current_channel_,starting_channel_,number_of_channels_);
  //labjack_ain_sub_ = nh_.subscribe("/labjack/ain_msg", 100 , &TemperatureMeasure::LabjackMsgCallback, &object_copy );
  labjack_ain_sub_ = nh_.subscribe("/labjack/ain_msg", 100 , LabjackMsgCallback );
  
  start_time_ = ros::Time::now();

  // Call the service get the calibrated currents
  ros::ServiceClient labjack_current_client = nh_.serviceClient<labjack::GetCurrents>("/labjack/cal_currents");
  labjack::GetCurrents cal_currents;
  if (!labjack_current_client.call(cal_currents))
    ROS_ERROR("Failed to call labjack calibrated current service");
  //double temp_res = labjack_temperature_.response.temp_res;

  cal_10uA_ = cal_currents.response.cal_current_10uA;
  cal_200uA_= cal_currents.response.cal_current_200uA;
  //ROS_INFO("200 uA is: %lf", cal_200uA_);

 // Call the service get the calibrated currents
  ros::ServiceClient labjack_publishing_client = nh_.serviceClient<labjack::StartPublishing>("/labjack/publishing_service");
  labjack::StartPublishing start_labjack;
  start_labjack.request.start = true;
  if (!labjack_publishing_client.call(start_labjack))
    ROS_ERROR("Failed to call labjack publishing service");

  publishing_ = true;

  data_pub_ =  nh_.advertise<vanDuinen::temp>("/temperature", 10);

  return true;

}


TemperatureMeasure::~TemperatureMeasure(void)
{
  // Close log file
  data_file_.close();
}

void TemperatureMeasure::publish()
{

  int i;
  bool c200uA;
  double du;
  if( current_channel_ == 0)
    c200uA = false;
  else
    c200uA = true;
  // Put the values into a message
  temp_msg_.header.stamp = g_time; //ros::Time::now();

  g_labjack_mutex.lock();
  for( i=starting_channel_; i < (number_of_channels_+starting_channel_); i++)
    {
      // Convert voltage to resistance
      if( i == (number_of_channels_+starting_channel_ -1) )
	du = g_ain_data[i];
      else
	du = g_ain_data[i]-g_ain_data[i+1];
      ROS_INFO("On channel %d delta voltage is %lf, voltage is %lf",i,du, g_ain_data[i]);
      temp_msg_.temperature[i] = volt2temperature(du,c200uA);
    }
  g_labjack_mutex.unlock();

  // Publish the values
  data_pub_.publish(temp_msg_);

}

/*
void TemperatureMeasure::logg()
{
  time_ = g_time - start_time_t; //ros::Time::now() - start_time_;
  data_file_ << time_.toSec() << "\t" << volt2temperature(g_ain_data[3],true) << std::endl;
}*/

void TemperatureMeasure::spin()
{
  while (nh_.ok())
    {
      if (publishing_) //If publishing, publish 
	{
	  publish();
	} 

      
      //ROS_INFO("Time %f temp %lf",time_.toSec(),lintemp);
      ros::spinOnce(); //Needed for callbacks
      loop_rate_.sleep(); //
     
    }
  
}

bool TemperatureMeasure::init_loggfile(char* path)
{

  // Open data file for printing
  time_t t = time(0);
  struct tm* lt = localtime(&t);
  char time_str[256];
  sprintf(time_str, "%s/templog_%04d%02d%02d_%02d%02d%02d.log",path,
          lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
          lt->tm_hour, lt->tm_min, lt->tm_sec);
 
  data_file_.open(time_str);

  return true;
}

double TemperatureMeasure::volt2temperature(double u, bool cur_200ua)
{


  //  double t;
  // Resistance voltage conversion table from 
  // http://www.advindsys.com/ApNotes/YSI400SeriesProbesRvsT.htm

  double resistance[35] = {4273, 4074, 3886, 3708, 3539, 3378, 3226, 
			   3081, 2944, 2814, 2690, 2572, 2460, 2354, 
			   2252, 2156, 2064, 1977, 1894, 1815, 1739,
			   1667, 1599, 1533, 1471, 1412, 1355, 1301,
			   1249, 1200, 1152, 1107, 1064, 1023, 983.8 };
  double temp[35] = {11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,
		     28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45};
  double lintemp=-300;
  int i=0;

  // Call the service get data 

  //labjack_temperature_.request.ain_number = ain_ch;     // Temperature sensor is on channel AIN3
  //labjack_temperature_.request.current_number = curr_n; // Temperature sensor supplied with 200 uA

  //if (!labjack_temperature_client_.call(labjack_temperature_))
  //    ROS_ERROR("Failed to call labjack temperature service");
  //double temp_res = labjack_temperature_.response.temp_res;
  //time_ = labjack_temperature_.response.header.stamp - start_time_;
  
  double temp_res;
  if (cur_200ua == true)
    temp_res = u / cal_200uA_ ;
  else
    temp_res = u / cal_10uA_;      
  
  if( temp_res > 42733 )
    {
      ROS_WARN("Temp too low! Below 11 deg");
      return -300;
    }
  if( temp_res < 1023 )
    {
      ROS_WARN("Temp too high! Above 45 deg");
      return -300;
    }

  while(temp_res < resistance[i])
    i++;
 
  // Picse wise linear approximation:

  lintemp =((temp_res-resistance[i])*(temp[i+1]-temp[i])/(resistance[i+1]-resistance[i]))+temp[i];

  return lintemp;

}

/*
double TemperatureMeasure::get_temperature(int ain_ch, int curr_n)
{
  //double t;
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

  // Call the service get data 

  labjack_temperature_.request.ain_number = ain_ch;     // Temperature sensor is on channel AIN3
  labjack_temperature_.request.current_number = curr_n; // Temperature sensor supplied with 200 uA

  if (!labjack_temperature_client_.call(labjack_temperature_))
    ROS_ERROR("Failed to call labjack temperature service");
  double temp_res = labjack_temperature_.response.temp_res;
  time_ = labjack_temperature_.response.header.stamp - start_time_;
  
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
  temp_[0] =((temp_res-resistance[i])*(temp[i+1]-temp[i])/(resistance[i+1]-resistance[i]))+temp[i];

  data_file_ << time_.toSec() << "\t" << temp_[0] << std::endl;

  ROS_INFO("Time %f temp %lf",time_.toSec(),temp_[0]);

  return temp_[0];
  
}*/

/*double TemperatureMeasure::get_resistance(int ain_ch, int curr_n)
{

  

}
*/
