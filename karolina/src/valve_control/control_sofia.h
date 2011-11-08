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

#ifndef CONTROL_SOFIA_H
#define CONTROL_SOFIA_H

#include <u6.h>  //Use functions directly from here
#include <ros/ros.h>
#include <math.h>
#include "std_msgs/Bool.h" //For publishing valve states
#include "std_msgs/Float32.h" //For publishing pressure values

//#include <dataglove/Start.h>

#define NVALVES 4 //For this application, only 4 valves, 2 valves per muscle, 2 muscles
#define NPSENSORS 14 //Hard-coded as 14 in labjack_node.h
#define P_TOL 0.1 //Pressure tolerance, 0.05 seems good
#define B_TOL 0.15 //Bend tolerance, 0.05 seems good

#define PI 3.14159265

class Mscl_control //control of muscles class
{


private:

  //Variables
  double sensorVector_[NPSENSORS]; //Vector containing sensor data from Labjack

  struct ValvesStruct //status of valves -  either open or closed - 1 struct per muscle
  {
     bool V_tank;	//Valve 'from' tank
     bool V_atm;	//Valve 'to' atmosphere
  };

  struct PamPressureStruct //Mscl_PVal_	//Muscle pressure
  {
     double P_aim;
     double P_cur;
  };

  struct BendStruct //Bend_Val_	//Muscle pressure
  {
     double bend_aim;
     double bend_cur;
  };

  //Mscl_control::
  PamPressureStruct PMscl_[NVALVES/2];//2 muscles 
  //  Mscl_control::
  BendStruct bend_[NVALVES/2];//2 muscles // shoudl corresond to joints!! 

  ValvesStruct valves_[NVALVES/2];//4 valves, each struct contains 2 valves

  ros::Rate loop_rate_;
  // For generating a sine motion
  float step_;
  int count_;
  float ref_;

  //PID:
  double old_error_; //???? - OK?
  double integral_;
  double Dt_; //get the real one lol

  //Sensor 'calibration'
  double p_prop_ ; // = 1.721;
  double p_bot_; // = 0.45;

  // Data from glove
  float glovedata_[18];

  //For rxplot
  ros::NodeHandle controlNH_;

  ros::Publisher vT0_pub_;  //Valve, Tank, Muscle 0
  ros::Publisher vA0_pub_;  //Valve, Atm, Muscle 0
  ros::Publisher vT1_pub_;  //Valve, Tank, Muscle 1
  ros::Publisher vA1_pub_;  //Valve, Atm, Muscle 1

  ros::Publisher P0_aim_pub_;
  ros::Publisher P0_cur_pub_;
  ros::Publisher P1_aim_pub_;
  ros::Publisher P1_cur_pub_;

  ros::Publisher bend_aim_pub_;
  ros::Publisher bend_cur_pub_;
  ros::Publisher bend_sig_pub_;

public:
      
  // Functions
  Mscl_control();  //constructor
  ~Mscl_control(); //destructor
  
  //Functions
  //ros srv to get sensor data?
  bool putValves(HANDLE hDevice, ValvesStruct* new_valves);
  
  ValvesStruct* bangBang(PamPressureStruct* current_pressure, 
			 BendStruct* current_bend,
			 ValvesStruct* new_valves, 
			 bool pressureControl, bool bendControl);

  ValvesStruct* bangBangBend(PamPressureStruct* current_pressure, 
			     BendStruct* current_bend,
			     ValvesStruct* new_valves);
  
  
  bool spin(void);
  bool init(void);
  int publish(void);

  // Callback to subscribe to data glove
  //void GloveSensorMsgCallback(const boost::shared_ptr<const sensor_msgs::JointState> &msg);

 //From labjack_node:

  int SetDO(HANDLE hDevice, uint16 fio, uint16 eio, uint16 cio);
 
  //EXAMPLE U6 STREAMING:

  int ConfigIO_example(HANDLE hDevice);
  int StreamConfig_example(HANDLE hDevice);
  int StreamStart(HANDLE hDevice);
  int StreamData_example(HANDLE hDevice, u6CalibrationInfo *caliInfo);
  int StreamStop(HANDLE hDevice);

 
  HANDLE hDevice_;
  u6CalibrationInfo caliInfo_;

  u6TdacCalibrationInfo cali_dac_info_;

  uint8 NumChannels_;        //For this example to work proper, SamplesPerPacket needs
                                    //to be a multiple of NumChannels.
  uint8 SamplesPerPacket_;  //Needs to be 25 to read multiple StreamData responses
                                    //in one large packet, otherwise can be any value between
                                    //1-25 for 1 StreamData response per packet.

  double stream_volt_[5];
  //double stream_volt0;
  //double stream_volt1;
  //double stream_volt2;
  //double stream_volt3;
  //double stream_volt4;


};



#endif
