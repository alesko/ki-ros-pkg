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
#include <time.h>
//#include "cyberglove/cyberglove_publisher.h"
//#include "cyberglove/cyberglove_service.h"
//#include "cyberglove/dataglove_publisher.h"
#include "dataglove/dataglove_publisher.h"
//#include "cyberglove/dataglove_service.h"
#include "dataglove/dataglove_service.h"
//#include "cyberglove/Start.h"
#include "dataglove/Start.h"
#include <boost/smart_ptr.hpp>
//using namespace cyberglove_publisher;
//using namespace cyberglove_service;
using namespace dataglove_publisher;
using namespace dataglove_service;



/////////////////////////////////
//           MAIN              //
/////////////////////////////////


/** 
 *  Start the cyberglove publisher.
 *
 * @param argc 
 * @param argv 
 * 
 * @return -1 if error (e.g. no glove found)
 */

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "cyberglove_publisher");
  ros::init(argc, argv, "dataglove_publisher");
  //NodeHandle n;
  //boost::shared_ptr<DataglovePublisher> cyberglove_pub(new DataglovePublisher());
  boost::shared_ptr<DataglovePublisher> dataglove_pub(new DataglovePublisher());

  //CyberglovePublisher *cyberglove_pub = new CyberglovePublisher();
  //CybergloveService service(cyberglove_pub);  
  //DatagloveService service(cyberglove_pub);  
  DatagloveService service(dataglove_pub);  
 
  while( ros::ok() )    
    {
      if(dataglove_pub->isPublishing()){
        dataglove_pub->publish();
        }
      //else{ros::spinOnce();sleep(100);}
    }

  return 0;
}
