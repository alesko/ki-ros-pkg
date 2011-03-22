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


#include "u6class.h"

using namespace std;

u6class::u6class()
{

  //Open first found U6 over USB
  local_ID_ = -1;
  if( (h_device_ = openUSBConnection(local_ID_)) == NULL)
    {
      ROS_ERROR("Unable to open USB connection.");
      exit(0);
    }
  
  //Get calibration information from U6
  if(getCalibrationInfo(h_device_, &cali_info_) < 0)
    {
      ROS_ERROR("ERROR: Unable to get calibration.");
      exit(0);
    }

}

u6class::~u6class(void)
{
  
  ROS_INFO("u6class object destroyed!");
  
}


void u6class::read_ain(double ain[14])
{

  //Read the single-ended voltage from AIN3
  //printf("\nCalling eAIN to read voltage from AIN0\n");
  double dblVoltage;
  for(int i=0; i<14; i++)
    {
      if((error_ = eAIN(h_device_, &cali_info_, i, 15, &dblVoltage, 0, 0, 0, 0, 0, 0)) != 0)
	{
	  ROS_WARNING("ERROR: Unable to aquire data \n");
	  exit(0);
	}    
      ain[i] = dblVoltage;
    }

  //data_.push_back(dblVoltage);
  
}

void u6class::write_aout(void)
{

}

void write_dout(void)
{

}

void u6class::read_din(bool din)
{

  //Read the single-ended voltage from AIN3
  //printf("\nCalling eAIN to read voltage from AIN0\n");
  double dblVoltage;
  if((error_ = eAIN(h_device_, &cali_info_, 6, 15, &dblVoltage, 0, 0, 0, 0, 0, 0)) != 0)
    {
      ROS_WARNING("ERROR: Unable to aquire data \n");
      exit(0);
    }       
  //data_.push_back(dblVoltage);
  
}


void u6class::close_LJ_device(void)
{
  ROS_INFO("Closing LabJack device.");
  closeUSBConnection(h_device_);
  /*data_file_ << "];" << std::endl;
  data_file_ << "counts=" << counts_ << ";";
  data_file_.close();
  printf("LabJack device closed!\n");*/
  ROS_INFO("LabJack device closed.");

}
