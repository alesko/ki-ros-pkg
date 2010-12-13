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


#include <ros/ros.h>

#include <string>
#include <sstream>

//#include "cyberglove/serial_glove.h"
//#include "cyberglove/cyberglove_publisher.h"
//#include "cyberglove/cyberglove_service.h"
//#include "cyberglove/dataglove_publisher.h"
//#include "cyberglove/dataglove_service.h"
#include "dataglove/dataglove_publisher.h"
#include "dataglove/dataglove_service.h"

using namespace ros;

//namespace cyberglove_service{
namespace dataglove_service{

  //CybergloveService::CybergloveService(boost::shared_ptr<cyberglove_publisher::CyberglovePublisher> publish)
  //CybergloveService::CybergloveService(boost::shared_ptr<dataglove_publisher::CyberglovePublisher> publish)
DatagloveService::DatagloveService(boost::shared_ptr<dataglove_publisher::DataglovePublisher> publish)                  
 :  node("~"), pub(publish)
{
  //service_start = node.advertiseService("start",&CybergloveService::start,this);
  service_start = node.advertiseService("start",&DatagloveService::start,this);
  //service_calibration = node.advertiseService("calibration", &CybergloveService::calibration, this);
  ROS_INFO("Listening for service");
}

  //bool CybergloveService::start(cyberglove::Start::Request &req, cyberglove::Start::Response &res){
  //bool CybergloveService::start(dataglove::Start::Request &req, dataglove::Start::Response &res){
bool DatagloveService::start(dataglove::Start::Request &req, dataglove::Start::Response &res){
    if(req.start){
        ROS_INFO("Glove is now publishing");
        this->pub->setPublishing(true);    
    }
    else{
        ROS_INFO("Glove has stopped publishing");
        this->pub->setPublishing(false);
    }
    //this->pub->cyberglove_pub.shutdown();
    return true;
}
  /*bool CybergloveService::calibration(cyberglove::Calibration::Request &req, cyberglove::Calibration::Response &res){
    this->pub->setPublishing(false);
    this->pub->initialize_calibration(req.path);
    this->pub->setPublishing(true); 
    return true;
}*/
}
