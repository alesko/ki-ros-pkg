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
//#include "cyberglove/xml_calibration_parser.h"

#include "fifthDT_glove.h" // Fifth Dimension glove

// Mutex
#include <boost/thread/mutex.hpp>

using namespace ros;

//namespace cyberglove_publisher{
namespace dataglove_publisher{

class DataglovePublisher
{
 public:
  /// Constructor
  DataglovePublisher();
  
  /// Destructor
  ~DataglovePublisher();

  int GloveInit(std::string path_to_glove);

  //  Publisher cyberglove_pub;
  Publisher dataglove_pub;
  //void initialize_calibration(std::string path_to_calibration);
  void publish();
  bool isPublishing();
  void setPublishing(bool value);

  fdGlove*     pGloveA;
 private:
  int          glovetype;
  std::string  szPort;
  std::string  szType;
  /////////////////
  //  CALLBACKS  //
  /////////////////

  ros::Time last_time;
  //ros::Duration dt;
  
  //ros node handle
  NodeHandle node, n_tilde;
  Rate publish_rate;
  std::string path_to_glove;
  bool publishing;

  ///the calibration parser
  //xml_calibration_parser::XmlCalibrationParser calibration_parser;

  //Publisher cyberglove_raw_pub;
  Publisher dataglove_raw_pub;

  sensor_msgs::JointState jointstate_msg;
  sensor_msgs::JointState jointstate_raw_msg;

  void add_jointstate(float position, std::string joint_name);

  std::vector<float> calibration_values;

  float* glovePositions;
  
  short unsigned int raw_old[18];  
  float scaled_old[18];
  
  short unsigned int raw_vel[18];    
  float scaled_vel[18];

}; // end class DataglovePublisher

} // end namespace
#endif 	    /* !CYBERGLOVE_PUBLISHER_H_ */
