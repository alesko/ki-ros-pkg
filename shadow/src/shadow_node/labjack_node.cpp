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

//#include <vector>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>

//#include <tf/transform_broadcaster.h>
//#include <shadow_base.h>
//#include <shadow_commands.h>
//#include <shadow_io.h>

//#include "shadow_node.h"
#include "labjack_node.h"

/*
// messages
#include <shadow/Sensors.h>
#include <shadow/Valves.h>

// services
#include <shadow/GetStatus.h>
#include <shadow/GetSensors.h>
#include <shadow/SetTargets.h>
#include <shadow/SetValves.h>
#include <shadow/PulseValves.h>

#include <shadow/LJSetTargets.h>
#include <shadow/StartPublishing.h>
*/
using namespace ros;

//const uint8 NumChannels = 5;        //For this example to work proper, SamplesPerPacket needs
                                    //to be a multiple of NumChannels.
//const uint8 SamplesPerPacket = 25;  //Needs to be 25 to read multiple StreamData responses
                                    //in one large packet, otherwise can be any value between
                                    //1-25 for 1 StreamData response per packet.
boost::mutex g_labjack_mutex;

LabjackNode::LabjackNode() : private_nh_("~"), publish_rate_(100) //init variabels - ros grej
{
  NumChannels_ = 5;        //For this example to work proper, SamplesPerPacket needs
                                    //to be a multiple of NumChannels.
  SamplesPerPacket_ = 25;  //Needs to be 25 to read multiple StreamData responses
                                    //in one large packet, otherwise can be any value between
                                    //1-25 for 1 StreamData response per packet.
  packetCounter_ = 0;
  totalPackets_ = 0;
  rate_ = (int)(1.0/(publish_rate_.expectedCycleTime() ).toSec()+0.5);
  ROS_INFO("Rate is %d",rate_);
  //Open first found U6 over USB
  local_ID_ = -1;
  if( (h_device_ = openUSBConnection(local_ID_)) == NULL)
    {
      ROS_ERROR("Unable to open USB connection\n");
      exit(0);
    }
  
  //Get calibration information from U6
  if(getCalibrationInfo(h_device_, &cali_info_) < 0)
    {
      ROS_ERROR("Unable to get calibration\n");
      exit(0);
    }
 
  //Getting calibration information from LJTDAC
  if(getTdacCalibrationInfo(h_device_, &cali_dac_info_, 2) < 0)  
    {
      ROS_ERROR("Unable to get DAC calibration\n");
      exit(0);
    }

  // Configure LabJack's IO
  //ConfigIO();
  if( ConfigIO() != 0 )
    ROS_WARN("LabJack is not properly configured!");
  
  // Stop streming if LJ was stopped wrongly
  StreamStop();

  counter_ = 0;
  //std::string dev;
  //std::string searched_param;
  //double publish_freq;

  //ROS_INFO("Creating a PAM node");
    
  //shadow_ = shadowInitialize();

  // Find the Shadow prefix
  //private_nh_.searchParam("pam_prefix", searched_param);
  //private_nh_.param(searched_param, prefix_, std::string());

  // set path to SPCU
  //std::string full_topic = prefix_ + "/path_to_labjack";  // Necessary??
  /*std::string full_topic = prefix_ + "/path_to_labjack";  // Necessary??
  if (private_nh_.getParam(full_topic, dev))
    {
      ROS_INFO("Path to SPCU is: %s", dev.c_str());
    }
  else
    {
      ROS_ERROR("Unable to determine path to SPCU, full_topic=%s", full_topic.c_str());
      return;
    }
  */
  //strcpy(shadow_->dev.ttyport, dev.c_str());

  //private_nh_.searchParam("/spcu_publish_frequency", searched_param);
  //private_nh_.param(searched_param, prefix_, std::string());


  // set publish frequency from parameter server
  /*
  full_topic = prefix_ + "/spcu_publish_frequency";    

  if (private_nh_.getParam(full_topic, publish_freq))
    {      
      ROS_INFO("Frequency from %s is %f", full_topic.c_str(), publish_freq);
      publish_rate_ = Rate(publish_freq);      
    }
  
  ROS_INFO("Shadow SPCU node is created");
  */

}

LabjackNode::LabjackNode(std::string dev) : private_nh_("~"), publish_rate_(100) //publish_rate_(60)
{
    
  //shadow_ = shadowInitialize();
  //strcpy(shadow_->dev.ttyport, dev.c_str());
 
}

LabjackNode::~LabjackNode(void)  //Destructor destorys object, ~ needed
{
    
  ROS_INFO("Closing LabJack device.");
  closeUSBConnection(h_device_);
  ROS_INFO("LabJack device closed.");

}


void LabjackNode::init()
{
  ROS_INFO("Initializing Labjack");
  int msg_que_len = 5;    
  do_state_ = false;
  //last_update_ = -1.0;
  first_update_ = true;
  ain_reading_pub_ = private_nh_.advertise<shadow::LJSensors>("/labjack/ain_msg",msg_que_len );
  /*if (shadowDeviceConnectPort(&shadow_->dev) < 0) 
    {
      ROS_FATAL("Unable to connect shadow at %s\n", shadow_->dev.ttyport);
      private_nh_.shutdown();
      return;   
    }
  else
  {
      ROS_INFO("Connected to device");
      //publishes sensor readings
      //std::string prefix;
      //std::string searched_param;
      //private_nh_.searchParam("shadow_prefix", searched_param);
      //private_nh_.param(searched_param, prefix_, std::string());
      std::string full_topic = prefix_ + "/sensor_msg";
      ROS_INFO("Starting publisher!");
      labjack_pub_ = private_nh_.advertise<shadow::ShadowSensors>(full_topic,msg_que_len );
      full_topic = prefix_ + "/target_msg";
      target_pub_ = private_nh_.advertise<shadow::ShadowTargets>(full_topic,msg_que_len );
   }
    
  // ***** Parameters *****

  ROS_INFO("Retrieving module state");
  shadowHexStatus(shadow_);  
  shadowPrintParams(&shadow_->par);
     
  //sensor_reading_pub_ = private_nh_.advertise<shadow::Sensors>("sensors_pub", 100);

  ROS_INFO("Starting services!");
  system_status_srv_ = private_nh_.advertiseService("get_status", &ShadowNode::getStatus,this);
  set_valves_srv_ = private_nh_.advertiseService("set_valves", &ShadowNode::setValves,this);
  pulse_valves_srv_ = private_nh_.advertiseService("pulse_valves", &ShadowNode::pulseValves,this);
  sensor_reading_srv_ = private_nh_.advertiseService("get_sensor_readings", &ShadowNode::getSensorReading,this);
  targets_srv_ = private_nh_.advertiseService("set_targets", &ShadowNode::setTargets,this);
  contoller_srv_ = private_nh_.advertiseService("enable_controller", &ShadowNode::setController,this);
  contoller_target_srv_ = private_nh_.advertiseService("enable_controller_target", &ShadowNode::setControllerwTarget,this);
  disable_contoller_srv_ = private_nh_.advertiseService("disable_controller", &ShadowNode::disController,this);*/
  pulse_valves_srv_ = private_nh_.advertiseService("pulse_valves", &LabjackNode::pulseValves,this);
  targets_srv_ = private_nh_.advertiseService("set_targets", &LabjackNode::setTargets,this);
  publishing_srv_ = private_nh_.advertiseService("publishing_service", &LabjackNode::setPublishing,this);

  temperature_srv_ = private_nh_.advertiseService("temperature", &LabjackNode::getTemperatureResistance,this);

  publishing_ = false;

  ROS_INFO("LabJack is ready!");

}

bool LabjackNode::getAINdata(double data[14])
{
  int i;
  for( i=0; i < 14; i++)
    {
      labjack_mutex_.lock();
      data[i] = ain_[i];
      labjack_mutex_.unlock();
    }

  return true;
}

bool LabjackNode::getSensorReading(void)
{
  //long s;
  double dblVoltage;
  for(int i=0; i <14; i++)
    {
      if((error_ = eAIN(h_device_, &cali_info_, i, 15, &dblVoltage, 0, 0, 0, 0, 0, 0)) != 0)
	{
	  ROS_WARN("Unable to aquire data");
	  exit(0);
	}       
      ain_[i] = dblVoltage;
    }
  //ROS_INFO("Read data %f",dblVoltage);
  
 
  //int i;
  //unsigned short  sensor_val[8];

  /*shadow_mutex_.lock();
  // Read the sensor values
  shadowHexReadSensors(&shadow_->dev,sensor_val);
  resp.header.stamp = ros::Time::now();    
  // Send the values
  for(i=0;i < 8;i++)
    resp.Sensors[i] = sensor_val[i];

  shadow_mutex_.unlock();
  */

  return true;

 }


bool LabjackNode::pulseValves(shadow::LJPulseValves::Request& req, shadow::LJPulseValves::Response& resp)
{   
  int i;
  
  for(i=0; i < NUM_VALVES; i++)
    {
      // Do proper round
      high_time_[i] = (int) (req.valve_p[i] * rate_ + 0.5); //(1.0/((publish_rate_.expectedCycleTime() ).toSec() ) ) +0.5); 
      //(publish_rate_ * req.valve_p[i]);
      if( req.valve_p[i] < 0.05)
	high_time_[i] = 0; //(int) (0.05 * (1.0/((publish_rate_.expectedCycleTime() ).toSec() ) )+0.5); //(publish_rate_ * 0.05);
      if( req.valve_p[i] > 1.00)
	high_time_[i] = rate_;//(int) (1.0/((publish_rate_.expectedCycleTime() ).toSec() ) +0.5);
      resp.valve_p[i] = (double)high_time_[i]/(double)(rate_); //(publish_rate_.expectedCycleTime() ).toSec() ) ;
    }
  ROS_INFO("hugh_time_[0]=%d",high_time_[0]);

  return true;
}


bool LabjackNode::setTargets(shadow::LJSetTargets::Request& req, shadow::LJSetTargets::Response& resp)
{
  double upper_limit = 100; // This value is a wild guess!!!! Implement a proper value
  int i;

  // Loop though the vales
  for(i=0; i < NUM_VALVES; i++)
    {
      // If action is not zero or too large
      if( (req.targets[i]> 0) && fabs(req.targets[i])< upper_limit )
	{
	  target_values_[i]= req.targets[i];
	  ROS_INFO("LabJack node: Setting targets:[%d]=%f",i,target_values_[i]);
          resp.targets[i] = target_values_[i];
	}
      else
	{
	  // Too large set value!!!
	  if(req.targets[i] != 0)
	    ROS_WARN("LabJack node: requested target for valve %d: %f is too large", i,req.targets[i]);
	}
    }

  return true;
}

bool LabjackNode::getTemperatureResistance(shadow::LJGetTemperature::Request& req, shadow::LJGetTemperature::Response& resp) 
{

  double dblVoltage;
  int ain = req.ain_number;
  int cur = req.current_number;
  if (cur > 1)
    ROS_ERROR("No current output with this number");
  if (ain > 14)
    ROS_ERROR("No analog input with number %d",ain);
  
  if((error_ = eAIN(h_device_, &cali_info_, ain, 15, &dblVoltage, 0, 0, 0, 0, 0, 0)) != 0)
    {
      ROS_WARN("Unable to aquire data");
      return false;
    }
  
  double res = dblVoltage/cali_info_.ccConstants[20+cur]; 
  
  resp.header.stamp = ros::Time::now();  
  resp.temp_res = res;

  return true;
}


bool LabjackNode::setPublishing(shadow::StartPublishing::Request& req, shadow::StartPublishing::Response& resp) //startpublishing is a srv
{
  if(req.start) //start from srv file
    {
      if( StreamConfig() != 0 )
	ROS_WARN("LabJack is not properly configured !");
      if( StreamStart() != 0)
	ROS_WARN("LabJack data streaming won't start!");

      ROS_INFO("Labjack is now publishing sensor data");
      publishing_ = true;
      resp.state = true;
    }
  else
    {
      StreamStop();
      ROS_INFO("Labjack has stopped publishing");
      publishing_ = false;
      resp.state = false;
     }
  //this->pub->cyberglove_pub.shutdown();
  return true;
}

bool LabjackNode::isPublishing()
{
  if (publishing_)
    {
      return true;
    }
  else
    {
      return false;
    }
}

void LabjackNode::publish()
{

  StreamData();
  int i;

  ain_msg_.header.stamp = ros::Time::now();    
   // Put the values into a message
  for( i=0; i < 14; i++)
    {
      labjack_mutex_.lock();
      ain_msg_.ain[i] = ain_[i];
      labjack_mutex_.unlock();
    }
  ain_reading_pub_.publish(ain_msg_);
  //ain_reading_pub_.publush();

    /*
  // Put the values into a message
  for(i=0; i < NUM_VALVES; i++)
    target_msg_.target[i] = set_target_[i];

   
  //publish the msgs
  shadow_pub_.publish(sensor_msg_);
  target_pub_.publish(target_msg_);
  */
  

}

int LabjackNode::tdac_example() //HANDLE hDevice, u6TdacCalibrationInfo *caliInfo)
{

    int err;
    uint8 options, speedAdjust, sdaPinNum, sclPinNum, address, numBytesToSend, numBytesToReceive, errorcode;
    uint16 binaryVoltage;
    uint8 bytesCommand[5];
    uint8 bytesResponse[64];
    uint8 ackArray[4];
    int i;

    err = 0;

    //Setting up parts I2C command that will remain the same throughout this example
    options = 0;             //I2COptions : 0
    speedAdjust = 0;         //SpeedAdjust : 0 (for max communication speed of about 130 kHz)
    sdaPinNum = 3;           //SDAPinNum : FIO3 connected to pin DIOB
    sclPinNum = 2;           //SCLPinNum : FIO2 connected to pin DIOA


    /* Set DACA to 0 or 5 volts. */
    if( do_state_ == true)
      {
	do_state_ = false;
	getTdacBinVoltCalibrated(&cali_dac_info_, 0, 0.0, &binaryVoltage);
	// Open valve
      }
    else
      {
	do_state_ = true;
	getTdacBinVoltCalibrated(&cali_dac_info_, 0, 5.0, &binaryVoltage);
	//Close valve
      }
    
    //Setting up I2C command
    //Make note that the I2C command can only update 1 DAC channel at a time.
    address = (uint8)(0x24);  //Address : h0x24 is the address for DAC
    numBytesToSend = 3;       //NumI2CByteToSend : 3 bytes to specify DACA and the value
    numBytesToReceive = 0;    //NumI2CBytesToReceive : 0 since we are only setting the value of the DAC
    bytesCommand[0] = (uint8)(0x30);  //LJTDAC command byte : h0x30 (DACA)

    //getTdacBinVoltCalibrated(&cali_dac_info_, 0, 5.0, &binaryVoltage);
    bytesCommand[1] = (uint8)(binaryVoltage/256);          //value (high)
    bytesCommand[2] = (uint8)(binaryVoltage & (0x00FF));   //value (low)

    //Performing I2C low-level call
    err = I2C(h_device_, options, speedAdjust, sdaPinNum, sclPinNum, address, numBytesToSend, numBytesToReceive, bytesCommand, &errorcode, ackArray, bytesResponse);

    /*if(checkI2CErrorcode(errorcode) == -1 || err == -1)
      {
        ROS_WARN("Error in writing I2C");
	return -1;
	}*/
    return 1;

}


//Sends a Feedback low-level command that configures digital directions,

int LabjackNode::SetDO(uint8 fio, uint8 eio, uint8 cio) 
{
  uint8 sendBuff[14], recBuff[10]; //
    int sendChars, recChars;
    int len= 14;
    int r_len= 10;
    uint16 binVoltage16, checksumTotal;
    uint8 state;

    sendBuff[1] = (uint8)(0xF8);  //Command byte
    //sendBuff[2] = 11;             //Number of data words (.5 word for echo, 10.5
                                  //words for IOTypes and data)
    sendBuff[2] = 0x04;             //Number of data words 

    sendBuff[3] = (uint8)(0x00);  //Extended command number
    sendBuff[6] = 0;     //Echo


    // Set digital out
    sendBuff[7]  = 0x1B; //27;  // Changed to 11 = BitStateWrite 
    sendBuff[8]  = 0xFF; // WriteMask determine if the corresponding bit shouldf be updated
    sendBuff[9]  = 0xFF;
    sendBuff[10] = 0xFF;
    sendBuff[11] = fio;
    sendBuff[12] = eio;
    sendBuff[13] = cio;

    /*    // Read the analog input
    sendBuff[14] = 0x02;
    sendBuff[15] = 0x00; // Positive Channel
    sendBuff[16] = 0x00; // Bit 0-3: Resolution Index
                         // Bit 4-7: GainIndex
    sendBuff[17] = 0x00; // Bit 0-2: Settling factor
                         // Bit 7:   Differetial
			 */
    extendedChecksum(sendBuff, len);

    //Sending command to U6
    if( (sendChars = LJUSB_BulkWrite(h_device_, U6_PIPE_EP1_OUT, sendBuff, len)) < len)
    {
        if(sendChars == 0)
            ROS_INFO("Feedback setup error : write failed");
        else
            ROS_INFO("Feedback setup error : did not write all of the buffer");
        return -1;
    }

    //Reading response from U6
    if( (recChars = LJUSB_BulkRead(h_device_, U6_PIPE_EP2_IN, recBuff, r_len)) < r_len)
    {
        if(recChars == 0)
        {
            ROS_INFO("Feedback setup error : read failed");
            return -1;
        }
        else
	  {
            //ROS_INFO("Feedback setup error : did not read all of the buffer");
	  }
    }

    checksumTotal = extendedChecksum16(recBuff, r_len);
    if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5])
    {
      ROS_INFO("Feedback setup error : read buffer has bad checksum16(MSB)");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4])
    {
        ROS_INFO("Feedback setup error : read buffer has bad checksum16(LBS)");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0])
    {
        ROS_INFO("Feedback setup error : read buffer has bad checksum8");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != 2 || recBuff[3] != (uint8)(0x00) )
    {
        ROS_INFO("Feedback setup error : read buffer has wrong command bytes ");
        return -1;
    }

    if( recBuff[6] != 0)
    {
        ROS_INFO("Feedback setup error : received errorcode %d for frame %d in Feedback response. ", recBuff[6], recBuff[7]);
        return -1;
    }

    return 0;
}


bool LabjackNode::updateValves()
{
  //ROS_INFO("updating contoller");
  // Maybe ros::WallTime and/or ros::WallDuration are better to use in this case?!
  ros::Time t = ros::Time::now();
  if (first_update_ == true)
    {
      
      ros::Duration dt = t - last_update_;
      last_update_ = t;

      int local_time = counter_ % rate_ ;//(int) ( 1.0 / (( publish_rate_.expectedCycleTime() ).toSec())) ;
      double gain = 10;
      double desired_value[NUM_VALVES];
      double current_value[NUM_VALVES];
      double effort[NUM_VALVES];
      
      int i;
      for(i=0; i < NUM_VALVES; i++)
	//for(i=0; i < 10; i++)
	{
	  //ROS_INFO("Oh yes!");
	  /*desired_value[i] = target_values_[i];
	  // Check the values' states here
	  if((error_ = eAIN(h_device_, &cali_info_, 0, 15, &current_value[i], 0, 0, 0, 0, 0, 0)) != 0)
	    {
	      ROS_WARN("Unable to aquire data from valve %d",i);
	    }       
	  // Insert PID controller here, for now compute error:
	  
	  effort[i] = gain * (current_value[i] - desired_value[i]);
	  ROS_INFO("Effort[%d]: %f",i,effort[i]);*/

	  /*if( high_time_[i] >= local_time )
	    {
	      // Open valve
	      if( (error_ = eDO(h_device_,i,(long)0.0)) != 0 )
		ROS_WARN("Unable to output data");
	    }
	  else
	    {
	      //Close valve
	      if( (error_ = eDO(h_device_,i,(long)1.0)) != 0 )
		ROS_WARN("Unable to output data");
	    }
	  */
	  if( do_state_ == true)
	    {
	      do_state_ = false;
	      // Open valve
	      if( (error_ = eDO(h_device_,i,(long)0.0)) != 0 )
		ROS_WARN("Unable to output data");
	    }
	  else
	    {
	      do_state_ = true;
	      //Close valve
	      if( (error_ = eDO(h_device_,i,(long)1.0)) != 0 )
		ROS_WARN("Unable to output data");
	    }
	  
	}

    }
  else
    {
      first_update_ = false;
      last_update_ = t;
    }


  return true;
    
}



//Sends a ConfigIO low-level command to turn off timers/counters
int LabjackNode::ConfigIO()
{
    uint8 sendBuff[16], recBuff[16];
    uint16 checksumTotal;
    int sendChars, recChars, i;

    sendBuff[1] = (uint8)(0xF8);  //Command byte
    sendBuff[2] = (uint8)(0x03);  //Number of data words
    sendBuff[3] = (uint8)(0x0B);  //Extended command number

    sendBuff[6] = 1;  //Writemask : Setting writemask for TimerCounterConfig (bit 0)

    sendBuff[7] = 0;  //NumberTimersEnabled : Setting to zero to disable all timers.
    sendBuff[8] = 0;  //CounterEnable: Setting bit 0 and bit 1 to zero to disable both counters
    sendBuff[9] = 0;  //TimerCounterPinOffset

    for(i = 10; i < 16; i++)
        sendBuff[i] = 0;   //Reserved
    extendedChecksum(sendBuff, 16);

    //Sending command to U6
    if( (sendChars = LJUSB_BulkWrite(h_device_, U6_PIPE_EP1_OUT, sendBuff, 16)) < 16)
    {
        if(sendChars == 0)
            ROS_INFO("ConfigIO error : write failed\n");
        else
            ROS_INFO("ConfigIO error : did not write all of the buffer\n");
        return -1;
    }

    //Reading response from U6
    if( (recChars = LJUSB_BulkRead(h_device_, U6_PIPE_EP2_IN, recBuff, 16)) < 16)
    {
        if(recChars == 0)
            ROS_INFO("ConfigIO error : read failed\n");
        else
            ROS_INFO("ConfigIO error : did not read all of the buffer\n");
        return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, 15);
    if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5])
    {
        ROS_INFO("ConfigIO error : read buffer has bad checksum16(MSB)\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4])
    {
        ROS_INFO("ConfigIO error : read buffer has bad checksum16(LBS)\n");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0])
    {
        ROS_INFO("ConfigIO error : read buffer has bad checksum8\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x05) || recBuff[3] != (uint8)(0x0B) )
    {
        ROS_INFO("ConfigIO error : read buffer has wrong command bytes\n");
        return -1;
    }

    if( recBuff[6] != 0)
    {
        ROS_INFO("ConfigIO error : read buffer received errorcode %d\n", recBuff[6]);
        return -1;
    }

    if( recBuff[8] != 0)
    {
        ROS_INFO("ConfigIO error : CounterEnable was not set to 0\n");
        return -1;
    }

   if( recBuff[8] != 0)
    {
        ROS_INFO("ConfigIO error : NumberTimersEnabled was not set to 0\n");
        return -1;
    }

    return 0;
}

//Sends a StreamConfig low-level command to configure the stream.
int LabjackNode::StreamConfig()
{
    int sendBuffSize;
    sendBuffSize = 14+NumChannels_*2;
    uint8 sendBuff[sendBuffSize], recBuff[8];
    int sendChars, recChars;
    uint16 checksumTotal;
    //    uint16 scanInterval;
    int i;

    sendBuff[1] = (uint8)(0xF8);    //command byte
    sendBuff[2] = 4 + NumChannels_;  //number of data words = NumChannels + 4
    sendBuff[3] = (uint8)(0x11);    //extended command number
    sendBuff[6] = NumChannels_;      //NumChannels
    sendBuff[7] = 1;                //ResolutionIndex
    sendBuff[8] = SamplesPerPacket_; //SamplesPerPacket, 1-25
    sendBuff[9] = 0;                //Reserved
    sendBuff[10] = 0;               //SettlingFactor: 0
    sendBuff[11] = 0;               //ScanConfig:
                                    // Bit 3: Internal stream clock frequency = b0: 4 MHz
                                    // Bit 1: Divide Clock by 256 = b0

    scanInterval_ = 4000;
    sendBuff[12] = (uint8)(scanInterval_&(0x00FF));  //scan interval (low byte)
    sendBuff[13] = (uint8)(scanInterval_/256);       //scan interval (high byte)

    for(i = 0; i < NumChannels_; i++)
    {
        sendBuff[14 + i*2] = i;  //ChannelNumber (Positive) = i
        sendBuff[15 + i*2] = 0;  //ChannelOptions: Bit 7: Differential = 0
                                 //                Bit 5-4: GainIndex = 0 (+-10V)
    }

    extendedChecksum(sendBuff, sendBuffSize);

    //Sending command to U6
    sendChars = LJUSB_BulkWrite(h_device_, U6_PIPE_EP1_OUT, sendBuff, sendBuffSize);
    if(sendChars < sendBuffSize)
    {
        if(sendChars == 0)
            ROS_INFO("Error : write failed (StreamConfig).");
        else
            ROS_INFO("Error : did not write all of the buffer (StreamConfig).");
        return -1;
    }

    for(i = 0; i < 8; i++)
        recBuff[i] = 0;

    //Reading response from U6
    recChars = LJUSB_BulkRead(h_device_, U6_PIPE_EP2_IN, recBuff, 8);
    if(recChars < 8)
    {
        if(recChars == 0)
            ROS_WARN("Error: read failed (StreamConfig).");
        else
            ROS_WARN("Error: did not read all of the buffer, %d (StreamConfig).", recChars);

        for(i=0; i<8; i++)
            ROS_INFO("%d ", recBuff[i]);

        return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, 8);
    if( (uint8)((checksumTotal / 256) & 0xff) != recBuff[5])
    {
        ROS_WARN("Error: read buffer has bad checksum16(MSB) (StreamConfig).");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4])
    {
        ROS_WARN("Error: read buffer has bad checksum16(LBS) (StreamConfig).");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0])
    {
        ROS_WARN("Error: read buffer has bad checksum8 (StreamConfig).");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x01) || recBuff[3] != (uint8)(0x11) || recBuff[7] != (uint8)(0x00))
    {
        ROS_WARN("Error: read buffer has wrong command bytes (StreamConfig).");
        return -1;
    }

    if(recBuff[6] != 0)
    {
        ROS_WARN("Errorcode # %d from StreamConfig read.", (unsigned int)recBuff[6]);
        return -1;
    }

    return 0;
}

//Sends a StreamStart low-level command to start streaming.
int LabjackNode::StreamStart()
{
  //totalPackets_ = 0;
   
  uint8 sendBuff[2], recBuff[4];
  int sendChars, recChars;

  sendBuff[0] = (uint8)(0xA8);  //CheckSum8
  sendBuff[1] = (uint8)(0xA8);  //command byte
  
  //Sending command to U6
  sendChars = LJUSB_BulkWrite(h_device_, U6_PIPE_EP1_OUT, sendBuff, 2);
  if(sendChars < 2)
    {
      if(sendChars == 0)
	ROS_ERROR("Error : write failed.");
      else
	ROS_WARN("Error : did not write all of the buffer.");
      return -1;
    }
  
  //Reading response from U6
  recChars = LJUSB_BulkRead(h_device_, U6_PIPE_EP2_IN, recBuff, 4);
  if(recChars < 4)
    {
      if(recChars == 0)
	ROS_ERROR("Error : read failed.");
      else
	ROS_WARN("Error : did not read all of the buffer.");
      return -1;
    }
  
  if( recBuff[1] != (uint8)(0xA9) || recBuff[3] != (uint8)(0x00) )
    {
      ROS_WARN("Error : read buffer has wrong command bytes ");
      return -1;
    }
  
  if(recBuff[2] != 0)
    {
      ROS_WARN("Errorcode # %d from StreamStart read.", (unsigned int)recBuff[2]);
      return -1;
    }
  
  // Reset package counter
  packetCounter_ = 0;
  
  return 0;
}

//Reads the StreamData low-level function response in a loop.
//All voltages from the stream are stored in the voltages 2D array.
int LabjackNode::StreamData()
{
    int recBuffSize;
    recBuffSize = 14 + SamplesPerPacket_*2;
    int recChars, backLog;
    int i, j, k, m, currChannel, scanNumber;//packetCounter
    //int totalPackets;        //The total number of StreamData responses read
    uint16 voltageBytes, checksumTotal;
    long startTime, endTime;
    int autoRecoveryOn;

    //int numDisplay;          //Number of times to display streaming information
    //int numReadsPerDisplay;  //Number of packets to read before displaying streaming information
    int readSizeMultiplier;  //Multiplier for the StreamData receive buffer size
    int responseSize;        //The number of bytes in a StreamData response (differs with SamplesPerPacket)

    //numDisplay = 1;
    //numReadsPerDisplay = 1; //24
    readSizeMultiplier = 25;
    responseSize = 14 + SamplesPerPacket_*2;

    /* Each StreamData response contains (SamplesPerPacket / NumChannels) * readSizeMultiplier
     * samples for each channel.
     * Total number of scans = (SamplesPerPacket / NumChannels) * readSizeMultiplier * numReadsPerDisplay * numDisplay
     */
    //double voltages[(SamplesPerPacket/NumChannels)*readSizeMultiplier*numReadsPerDisplay*numDisplay][NumChannels];
    int stream_data_response_size = (SamplesPerPacket_/NumChannels_)*readSizeMultiplier;
    //int total_number_of_scans = stream_data_response_size*numReadsPerDisplay*numDisplay;
    //int total_number_of_scans = stream_data_response_size; //*numReadsPerDisplay*numDisplay;
    std::vector< std::vector<double> > voltages(stream_data_response_size, std::vector<double> (NumChannels_));

    uint8 recBuff[responseSize*readSizeMultiplier];
    //packetCounter = 0;
    currChannel = 0;
    scanNumber = 0;
    //totalPackets = 0;
    recChars = 0;
    autoRecoveryOn = 0;

    //ROS_INFO("Reading Samples...\n");

    //startTime = getTickCount();

    //    for (i = 0; i < numDisplay; i++)
    //{
    //for(j = 0; j < numReadsPerDisplay; j++)
    //{
    /* For USB StreamData, use Endpoint 3 for reads.  You can read the multiple
     * StreamData responses of 64 bytes only if SamplesPerPacket is 25 to help
     * improve streaming performance.  In this example this multiple is adjusted
     * by the readSizeMultiplier variable.
     */
    
    //Reading stream response from U6
    recChars = LJUSB_BulkRead(h_device_, U6_PIPE_EP3_IN, recBuff, responseSize*readSizeMultiplier);
    if(recChars < responseSize*readSizeMultiplier)
      {
	if(recChars == 0)
	  ROS_WARN("Error: read failed (StreamData).");
	else
	  ROS_WARN("Error: did not read all of the buffer, expected %d bytes but received %d(StreamData).", responseSize*readSizeMultiplier, recChars);
	
	return -1;
      }
    
    //Checking for errors and getting data out of each StreamData response
    for (m = 0; m < readSizeMultiplier; m++)
      {
	//totalPackets++;
	totalPackets_++;
	
	checksumTotal = extendedChecksum16(recBuff + m*recBuffSize, recBuffSize);
	if( (uint8)((checksumTotal >> 8) & 0xff) != recBuff[m*recBuffSize + 5])
	  {
	    ROS_WARN("Error: read buffer has bad checksum16(MSB) (StreamData).");
	    return -1;
	  }
	
	if( (uint8)(checksumTotal & 0xff) != recBuff[m*recBuffSize + 4])
	  {
	    ROS_WARN("Error : read buffer has bad checksum16(LBS) (StreamData).");
	    return -1;
	  }
	
	checksumTotal = extendedChecksum8(recBuff + m*recBuffSize);
	if( checksumTotal != recBuff[m*recBuffSize])
	  {
	    ROS_WARN("Error : read buffer has bad checksum8 (StreamData).");
	    return -1;
	  }
	
	if( recBuff[m*recBuffSize + 1] != (uint8)(0xF9) || recBuff[m*recBuffSize + 2] != 4 + SamplesPerPacket_ || recBuff[m*recBuffSize + 3] != (uint8)(0xC0) )
	  {
	    ROS_WARN("Error : read buffer has wrong command bytes (StreamData).");
	    return -1;
	  }
	
	if(recBuff[m*recBuffSize + 11] == 59)
	  {
	    if(!autoRecoveryOn)
	      {
		ROS_WARN("\nU6 data buffer overflow detected in packet %d.\nNow using auto-recovery and reading buffered samples.", totalPackets_);
		autoRecoveryOn = 1;
	      }
	  }
	else if(recBuff[m*recBuffSize + 11] == 60)
	  {
	    ROS_WARN("Auto-recovery report in packet %d: %d scans were dropped.\nAuto-recovery is now off.", totalPackets_, recBuff[m*recBuffSize + 6] + recBuff[m*recBuffSize + 7]*256);
	    autoRecoveryOn = 0;
	  }
	else if(recBuff[m*recBuffSize + 11] != 0)
	  {
	    ROS_WARN("Errorcode # %d from StreamData read.\n", (unsigned int)recBuff[11]);
	    return -1;
	  }
	
	if(packetCounter_ != (int)recBuff[m*recBuffSize + 10])
	  {
	    ROS_WARN("PacketCounter (%d) does not match with with current packet count (%d)(StreamData).", recBuff[m*recBuffSize + 10], packetCounter_);
	    return -1;
	  }

	backLog = (int)recBuff[m*48 + 12 + SamplesPerPacket_*2];

	for(k = 12; k < (12 + SamplesPerPacket_*2); k += 2)
	  {
	    voltageBytes = (uint16)recBuff[m*recBuffSize + k] + (uint16)recBuff[m*recBuffSize + k+1]*256;

	    //getAinVoltCalibrated(&cali_info_, 1, 0, 0, voltageBytes, &(voltages[scanNumber][currChannel]));
	    getAinVoltCalibrated(&cali_info_, 1, 0, 0, voltageBytes, &(voltages[scanNumber][currChannel]));

	    currChannel++;
	    if(currChannel >= NumChannels_)
	      {
		currChannel = 0;
		scanNumber++;
	      }
	  }

	if(packetCounter_ >= 255)
	  packetCounter_ = 0;
	else
	  packetCounter_++;
      }

    //ROS_INFO("Number of scans: %d", scanNumber);
    //ROS_INFO("Total packets read: %d", totalPackets_);
    //ROS_INFO("Current PacketCounter: %d", ((packetCounter_ == 0) ? 255 : packetCounter_-1));
    //ROS_INFO("Current BackLog: %d", backLog);
    
    for(k = 0; k < NumChannels_; k++)
      {
	labjack_mutex_.lock();
	ain_[k] = voltages[scanNumber - 1][k];
	labjack_mutex_.unlock();
      }
    //  ROS_INFO("  AI%d: %.4f V", k, voltages[scanNumber - 1][k]);
    //}
    
    //endTime = getTickCount();
    //ROS_INFO("\Rate of samples: %.0lf samples per second", (scanNumber*NumChannels_)/((endTime - startTime)/1000.0));
    //ROS_INFO("Rate of scans: %.0lf scans per second", scanNumber/((endTime - startTime)/1000.0));
    
    return 0;
}



//Sends a StreamStop low-level command to stop streaming.
int LabjackNode::StreamStop()
{
    uint8 sendBuff[2], recBuff[4];
    int sendChars, recChars;

    sendBuff[0] = (uint8)(0xB0);  //CheckSum8
    sendBuff[1] = (uint8)(0xB0);  //command byte

    //Sending command to U6
    sendChars = LJUSB_BulkWrite(h_device_, U6_PIPE_EP1_OUT, sendBuff, 2);
    if(sendChars < 2)
    {
        if(sendChars == 0)
            ROS_INFO("Error : write failed (StreamStop).");
        else
            ROS_INFO("Error : did not write all of the buffer (StreamStop).");
        return -1;
    }

    //Reading response from U6
    recChars = LJUSB_BulkRead(h_device_, U6_PIPE_EP2_IN, recBuff, 4);
    if(recChars < 4)
    {
        if(recChars == 0)
            ROS_INFO("Error : read failed (StreamStop).");
        else
            ROS_INFO("Error : did not read all of the buffer (StreamStop).");
        return -1;
    }

    if(recChars < 4)
    {
        if(recChars == 0)
            ROS_INFO("Error : read failed (StreamStop).");
        else
            ROS_INFO("Error : did not read all of the buffer (StreamStop).");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xB1) || recBuff[3] != (uint8)(0x00) )
    {
        ROS_INFO("Error : read buffer has wrong command bytes (StreamStop).");
        return -1;
    }

    if(recBuff[2] != 0)
    {
        ROS_INFO("Errorcode # %d from StreamStop read.", (unsigned int)recBuff[2]);
        return -1;
    }

    /*
    //Reading left over data in stream endpoint.  Only needs to be done with firmwares
    //less than 0.94.
    uint8 recBuffS[64];
    int recCharsS = 64;
    ROS_INFO("Reading left over data from stream endpoint.\n");
    while(recCharsS > 0)
        recCharsS = LJUSB_BulkRead(h_device_, U6_PIPE_EP3_IN, recBuffS, 64);
    */

    return 0;
}

double LabjackNode::volts2temperature(double volts)
{
  // Resistance voltage conversion table from 
  // http://www.advindsys.com/ApNotes/YSI400SeriesProbesRvsT.htm

  double resistance[35] = {4273, 4074, 3886, 3708, 3539, 3378, 3226, 
			   3081, 2944, 2814, 2690, 2572, 2460, 2354, 
			   2252, 2156, 2064, 1977, 1894, 1815, 1739,
			   1667, 1599, 1533, 1471, 1412, 1355, 1301,
			   1249, 1200, 1152, 1107, 1064, 1023, 983.8 };
  double temp[35] = {11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,
		     28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45};

  double t;

  int i=0;
  double meas_res = volts/cali_info_.ccConstants[21]; // 200 milliamp
  
  if( meas_res > 42733 )
    {
      ROS_WARN("Temp too low!");
      return 0;
    }
  if( meas_res < 1023 )
    {
      ROS_WARN("Temp too high !");
      return 0;
    }

  while(meas_res < resistance[i])
    i++;
 
  // Picse wise linear approximation:
  t=((meas_res-resistance[i])*(temp[i+1]-temp[i])/(resistance[i+1]-resistance[i]))+temp[i];
  //ROS_INFO("Res %f, i=%d  %f %f temp %f",meas_res,i,resistance[i],resistance[i+1], t);

  return t;

}

bool LabjackNode::spin()
{
  //unsigned short  sensor_val[8];
  //ros::Rate r(10); // 10 ms or 100 Hz ??


  // Test steraming
  double my_data[14];
  int i;

  while (node_.ok())
    {
      if (publishing_) //If publishing, publish 
	{
	  publish();
	}
      if( do_state_ == true)
	{
	  do_state_ = false;
	  SetDO(0x00,0x00,0x00);
	}
      else
	{
	  do_state_ = true;
	  SetDO(0x01,0x00,0x00);
	}
      //getSensorReading();
      ros::spinOnce(); //Needed for callbacks
      publish_rate_.sleep(); //
      // Increase the "clock" by one tick
      counter_++;
      //ROS_INFO("Counter %d",counter_);
      getAINdata(my_data);
   
  
  
    }
  
  return true;
}






/*
bool ShadowNode::disController(shadow::DisableController::Request& req, shadow::DisableController::Response& resp)
{   
 
  shadow_mutex_.lock();
  shadowHexDisableController(&shadow_->dev, req.Controller_valve);
  shadow_mutex_.unlock();
  ROS_INFO("SHADOW: Disabled controller for valve: %d",
	   (unsigned short)req.Controller_valve);

  return true;
}



bool ShadowNode::setController(shadow::SetController::Request& req, shadow::SetController::Response& resp)
{   
  
  //ROS_INFO("Got: v=%d, s=%d, P=%d, I=%d, D=%d",(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
  //  (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);
  shadow_mutex_.lock();
  //shadowHexSetController(&shadow_->dev,(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
  //		 (char)-1, (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);
  shadowAsciiSetController(&shadow_->dev,(unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
  		 (char)-1, (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);
  shadow_mutex_.unlock();
  ROS_INFO("SHADOW: Setting controller values for valve %d using sensor %d: P=%d, I=%d, D=%d",
	   (unsigned short)req.Controller_valve , (unsigned short)req.Controller_sensor,
	   (char)req.Controller_P, (char)req.Controller_I, (char)req.Controller_D);

  //shadowHexPulseValves(&shadow_->dev,p_time_ms);

  return true;
}

*/
