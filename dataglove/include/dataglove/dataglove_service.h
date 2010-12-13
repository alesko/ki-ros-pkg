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


#ifndef   	DATAGLOVE_SERVICE_H_
#define   	DATAGLOVE_SERVICE_H_

#include <ros/ros.h>
#include <vector>
//#include "cyberglove_publisher.h"
#include "dataglove_publisher.h"
//#include "cyberglove/Start.h"
#include "dataglove/Start.h"
//#include "cyberglove/Calibration.h"
#include "dataglove/Calibration.h"
#include <boost/smart_ptr.hpp>

//messages

using namespace ros;

//namespace cyberglove_service{
namespace dataglove_service{

  //class CybergloveService
class DatagloveService
{
 public:
  /// Constructor
  //CybergloveService(boost::shared_ptr<cyberglove_publisher::CyberglovePublisher> publish);
  //CybergloveService(boost::shared_ptr<dataglove_publisher::CyberglovePublisher> publish);
  DatagloveService(boost::shared_ptr<dataglove_publisher::DataglovePublisher> publish);
    //CybergloveService();
    //bool start(cyberglove::Start::Request &req, cyberglove::Start::Response &res);
    bool start(dataglove::Start::Request &req, dataglove::Start::Response &res);
    //bool calibration(cyberglove::Calibration::Request &req, cyberglove::Calibration::Response &res);
 private:
    
  NodeHandle node;
  //boost::shared_ptr<cyberglove_publisher::CyberglovePublisher> pub;
  boost::shared_ptr<dataglove_publisher::DataglovePublisher> pub;
  ros::ServiceServer service_start;
  //ros::ServiceServer service_calibration;
};

}
#endif
