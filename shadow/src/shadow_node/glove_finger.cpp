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
#include "shadow_node.h"
#include "shadow_PID.h"
#include <ros/ros.h>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "glove_finger"); 

  ros::NodeHandle nh;
  
  ShadowPID spid; 

  // For parameter server
  std::string full_topic;
  std::string prefix; 
  std::string searched_param;

  double p,i,d;

  // Find the node prefix on the parameter server
  nh.searchParam("gain_prefix", searched_param);
  nh.param(searched_param, prefix, std::string());


  if (argc == 2)
    {
      std::string shadow_dev = argv[1];  
      ShadowNode s(shadow_dev);
      ROS_INFO("ShadowNode created");
      s.ShadowInit();
      spid.initPID(0.03, 0.00, 0.00, 0.0, -0.0); //Get from parameter server instead
      s.spin();

    }
  else
    {
      // Get paramteres from the parameter server
      ShadowNode s;
      ROS_INFO("ShadowNode created");
      s.ShadowInit();
      
      full_topic = prefix + "/gains/p";
      nh.getParam(full_topic, p);
      full_topic = prefix + "/gains/i";
      nh.getParam(full_topic, i);
      full_topic = prefix + "/gains/d";
      nh.getParam(full_topic, d);
      ROS_INFO("Gains\t P:%f\tI:%f\tD:%f ",p,i,d);

      spid.initPID(p, i, d, 0.0, -0.0); //Get from parameter server instead
      spid.initCallback();
      s.spin();

    }
  

  /*spid.initGloveCallback();
  spid.initCallback();
  spid.controlspin();
  */

  return 0;
}
