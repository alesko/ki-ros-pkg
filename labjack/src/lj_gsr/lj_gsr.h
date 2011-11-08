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
#ifndef LABJACK_GSR
#define LABJACK_GSR


#include <u6.h>

//#include <labjackusb.h>
#include <vector>

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <fstream>

// messages
#include <labjack/gsr.h>


class LabjackGSR
{
 private: //Only class can operate below

  //ros::NodeHandle node_; //Underscore == class parameters
  boost::mutex labjack_mutex_ ;

  std::string prefix_; // For parameter server

  ros::NodeHandle private_nh_; //("~");
  std::string device;

  uint32 counter_;

  bool publishing_;
  ros::Rate publish_rate_;
  int rate_;

  std::ofstream data_file_; // Data storange, all data
  std::ofstream gsr_file_; // Mean temp, low freq storange
  char* path_;

  ros::Duration time_;
  ros::Duration publish_duration_;
  ros::Time start_time_; 
  ros::Time last_time_publish_; 
  //int number_of_valves_;
  //bool first_update_;
  //ros::Time last_update_;
  //int high_time_[NUM_VALVES];
  ros::Publisher ain_reading_pub_;
  /*ros::Publisher shadow_pub_;
  ros::Publisher valve_state_pub_;


  ros::ServiceServer sensor_reading_srv_;  
  */
  
  
  // Messange
  //labjack::Sensors ain_msg_;
  double ain_[14];
  double gsr_[2];
  //vector<double> temp_vec_;
  std::vector<double> gsr1_vec_,gsr2_vec_;
  labjack::gsr gsr_msg_;
  int buffersize_ ;
  
  //double target_values_[NUM_VALVES];
  //bool getSensorReading(void);
  //bool updateValves();

  // LabJack 
  HANDLE            h_device_;
  u6CalibrationInfo cali_info_;
  u6TdacCalibrationInfo cali_dac_info_;
  int               local_ID_;
  long              error_;
  bool do_state_;
  int tdac_example();
  //int feedback_setup_example();
  

  int packetCounter_ ;
  int totalPackets_ ;
  unit8 SettlingFactor_ ;
  unit8 ScanConfig_ ;
  unit8 GainIndex_ ;
  /*
    const uint8 NumChannels = 5;        //For this example to work proper, SamplesPerPacket needs
    //to be a multiple of NumChannels.
    const uint8 SamplesPerPacket = 25;  //Needs to be 25 to read multiple StreamData responses
    //in one large packet, otherwise can be any value between
    //1-25 for 1 StreamData response per packet.
    */
  uint8 NumChannels_;        //For this example to work proper, SamplesPerPacket needs
  //to be a multiple of NumChannels.
  uint8 SamplesPerPacket_;  //Needs to be 25 to read multiple StreamData responses
  //in one large packet, otherwise can be any value between
  //1-25 for 1 StreamData response per packet.
  int ConfigIO();
  int StreamConfig();
  int StreamStart();
  int StreamData();
  int StreamStop();
  uint16 scanInterval_;
  int resolutionIndex_;
  bool differtialEnable_;
 public:

  LabjackGSR(); //Constructor
  //LabjackTemp(std::string dev); //Constuctor with args
  ~LabjackGSR(); //Destructor
  void init();
  bool spin();
  
  //bool getNodeStateOK();
  //ros::Rate getPublishRate();
  void publish();
  //bool isPublishing();
  //bool getAINdata(double data[14]);
  //int SetDO(uint8 fio, uint8 eio, uint8 cio);
  bool init_loggfile(char *path);
  double volts2temperature(double volts);
  // Contoller commands
  /*bool controllerInit();
  void controllerStarting();
  void controllerUpdate();
  void controllerStopping();*/


};

#endif
