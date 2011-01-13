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

/* This code is built from Ugo Cupcic <ugo@shadowrobot.com> code for
   Cyberglove in ROS 
*/


#ifndef   	DATAGLOVE_PUBLISHER_H_
#define   	DATAGLOVE_PUBLISHER_H_

#include <ros/ros.h>
#include <vector>

//messages
#include <sensor_msgs/JointState.h>
#include <boost/smart_ptr.hpp>

#include "fifthDT_glove.h" // Fifth Dimension glove

// Mutex
#include <boost/thread/mutex.hpp>

using namespace ros;


namespace dataglove_publisher{

class DataglovePublisher
{
 public:
  /// Constructor
  DataglovePublisher();
  
  /// Destructor
  ~DataglovePublisher();

  //int GloveInit(std::string path_to_glove);
  // Functions
  int  GloveInit();               // Glove must be initialized
  int  initCallback();               // Glove must be initialized
  void publish();
  bool isPublishing();
  void setPublishing(bool value);

  // Internal variables
  Publisher dataglove_pub_; // Due to the callback
  fdGlove*     pGloveA_;

 private:
  // Function
  void add_jointstate(float position, std::string joint_name);

  //ROS variables
  NodeHandle  n_tilde_; //node
  Rate publish_rate_;
  bool publishing_;
  Publisher dataglove_raw_pub;


  // Glove variables
  int          glovetype_;
  std::string  szPort_;
  std::string  szType_;
  ros::Time    last_time_;
  //float*       glovePositions_;
  //std::vector<float> calibration_values_;

  sensor_msgs::JointState jointstate_msg_;
  sensor_msgs::JointState jointstate_raw_msg_;

  short unsigned int raw_old_[18];  
  float scaled_old_[18];  
  short unsigned int raw_vel_[18];    
  float scaled_vel_[18];
 
 

  

}; // end class DataglovePublisher

} // end namespace
#endif 	    /* !DATAGLOVE_PUBLISHER_H_ */
