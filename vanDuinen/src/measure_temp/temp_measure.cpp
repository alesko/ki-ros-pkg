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

/*#include <algorithm> // sort, max_element, random_shuffle, remove_if, lower_bound 
#include <cstdlib>
#include <vector>

#include <fstream>
#include <iostream>*/
#include <sys/time.h>
#include <ros/ros.h>
#include "LJ_temp_measure_class.h"


int main(int argc, char **argv)
{
  // Init ROS
  ros::init(argc, argv, "temp_measure");

  // Create the object
  ros::Time::init();

  // Current channel 0= 10uA, 1 200 uA, 
  // AIN 3 and "range"
  TemperatureMeasure LJ_temp(1,3,1); 
  double temp;
  if(argc > 1)
    {      
      LJ_temp.init_loggfile(argv[1]);
      ROS_INFO("relative path is %s", argv[1]);
    }
  else
    {
      
      LJ_temp.init_loggfile("../data");
      ROS_INFO("relative path is ../data");
    }
  int ain_channel = 3;
  int curr_number = 1;

  LJ_temp.spin();
  /*
  ros::Rate loop_rate(1);

    while(ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();
      temp = LJ_temp.get_temperature( ain_channel, curr_number);
      
    }
  */
}
