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

#include "control_sofia.h"

/*
void Mscl_control::GloveSensorMsgCallback(const boost::shared_ptr<const sensor_msgs::JointState> &msg)
{
  int i;
  ros::Time t = msg->header.stamp;
  
  //glove_mutex_.lock();
  for(i=0; i < 18;i++)
    glovedata_[i] = msg->position[i];
  //glove_mutex_.unlock();

  //ROS_INFO("Glove data: %f %f %f %f %f", glovedata[0], glovedata[3], glovedata[6], glovedata[9], glovedata[12]);

}
*/

Mscl_control::Mscl_control(void):loop_rate_(500) //Constructor
{
  ROS_INFO("Muscle control object created.");
}

Mscl_control::~Mscl_control(void)  //Destructor
{

  //STREAMING EXAMPLE:
  ROS_INFO("LabJack will close");
  Mscl_control::StreamStop(hDevice_);
  closeUSBConnection(hDevice_);
  ROS_INFO("LabJack closed.");

  ROS_INFO("Muscle control object destroyed.");
}


bool Mscl_control::init(void)
{
  step_ = 0.5;
  count_ = 0;
  ref_ = 3;

  ROS_INFO("Initializing Control");

  //Sensor 'calibration'
  p_prop_ = 1.721;
  p_bot_ = 0.45;


  unsigned char msg_len_V = 5;
  unsigned int msg_len_P = 2000; //Needed?

  vT0_pub_ = controlNH_.advertise<std_msgs::Bool>("Valve_T0", msg_len_V);
  vA0_pub_ = controlNH_.advertise<std_msgs::Bool>("Valve_A0", msg_len_V);
  vT1_pub_ = controlNH_.advertise<std_msgs::Bool>("Valve_T1", msg_len_V);
  vA1_pub_ = controlNH_.advertise<std_msgs::Bool>("Valve_A1", msg_len_V);

  P0_aim_pub_ = controlNH_.advertise<std_msgs::Float32>("P0_aim", msg_len_P);
  P0_cur_pub_ = controlNH_.advertise<std_msgs::Float32>("P0_cur", msg_len_P);
  P1_aim_pub_ = controlNH_.advertise<std_msgs::Float32>("P1_aim", msg_len_P);
  P1_cur_pub_ = controlNH_.advertise<std_msgs::Float32>("P1_cur", msg_len_P);

  bend_aim_pub_ = controlNH_.advertise<std_msgs::Float32>("bend_aim", msg_len_P);
  bend_cur_pub_ = controlNH_.advertise<std_msgs::Float32>("bend_cur", msg_len_P);
  bend_sig_pub_ = controlNH_.advertise<std_msgs::Float32>("bend_cal", msg_len_P);


  ROS_INFO("Control initialized");

  //EXAMPLE U6 STREAMING:
  NumChannels_ = 5;        //For this example to work proper, SamplesPerPacket needs
  //to be a multiple of NumChannels.
  SamplesPerPacket_ = 25;  //Needs to be 25 to read multiple StreamData responses
  //in one large packet, otherwise can be any value between
  //1-25 for 1 StreamData response per packet.
  
  
   //HANDLE hDevice;
   //u6CalibrationInfo caliInfo;
  ROS_INFO("b4 hdevice");   
  //Opening first found U6 over USB
  if( (hDevice_ = openUSBConnection(-1)) == NULL )
    ROS_INFO("openusb");
  
  //Getting calibration information from U6
  if( getCalibrationInfo(hDevice_, &caliInfo_) < 0 )
    return false;
  
  //from labjack_node:
  if(getTdacCalibrationInfo(hDevice_, &cali_dac_info_, 2) < 0)  
    return false;
  //^from labjack_node
  
  if( Mscl_control::ConfigIO_example(hDevice_) != 0 )
    return false;
  
  //Stopping any previous streams
  Mscl_control::StreamStop(hDevice_);
  
  if( Mscl_control::StreamConfig_example(hDevice_) != 0 )
    return false;
  
  if( Mscl_control::StreamStart(hDevice_) != 0 )
    return false;
  
  
  //Mscl_control::StreamData_example(hDevice, &caliInfo);
  
  ROS_INFO("LabJack initialized");

  return true;

}

bool Mscl_control::putValves(HANDLE hDevice, Mscl_control::ValvesStruct* new_valves)
{
  //F0 = V_tank0, F1 = V_atm0, F2 = V_tank1, F3 = V_atm1 etc (Muscle 0,1)
  // Muscle 0 = down
  // Muscle 1 = up

  int i = 0;
  unsigned int valve_set = 0;
  unsigned int mult = 1; //Multiplier

  for (i = 0; i<NVALVES/2; i++)
  {
    valve_set = valve_set + (new_valves[i].V_tank)*mult;
    mult = mult << 1; // Shift one step
    valve_set = valve_set + (new_valves[i].V_atm)*mult;
    mult = mult << 1; // Shift one step
  }

  ROS_INFO("---");
  ROS_INFO("valve_set = %d", valve_set);  
  ROS_INFO("Vatm1 = %d, Vtank1 = %d, Vamt0 = %d, Vtank0 = %d ",  new_valves[1].V_atm, new_valves[1].V_tank, new_valves[0].V_atm, new_valves[0].V_tank );
  //ROS_INFO("Vtank0 = %d", new_valves[0].V_tank);
  //ROS_INFO("Vtank1 = %d", new_valves[1].V_tank);
  //ROS_INFO("Vatm1 = %d", new_valves[1].V_atm);
  

  Mscl_control::SetDO(hDevice, valve_set, 0, 0); //int SetDO(uint16 fio, uint16 eio, uint16 cio); => actually 8 bits, first parameter is enough
  //F: 3,2,1,0; 1 == high, 0 == low

  
  return true;
}

Mscl_control::ValvesStruct* Mscl_control::bangBangBend(Mscl_control::PamPressureStruct* current_pressure, 
					      Mscl_control::BendStruct* current_bend,
						       ValvesStruct* new_valves) //"global", then returned 
{  

  double aimValue;
  double curValue;
  float tol;

  int i = 0;

  // Only one bend sensor
  aimValue = current_bend[0].bend_aim;
  curValue = current_bend[0].bend_cur;
  tol = B_TOL;
  //Controller:
  
  //     if((curValue <= aimValue+tol) & (curValue >= aimValue-tol))
  ROS_INFO("curValue: %f, aimValue: %f", curValue, aimValue );
  if((curValue <= aimValue+tol) && (curValue >= aimValue-tol))
       {
	 new_valves[0].V_tank = false; //Close tank valve
	 new_valves[0].V_atm = false; //Close atm valve
	 new_valves[1].V_tank = false; //Close tank valve
	 new_valves[1].V_atm = false; //Close atm valve
       }
  else
    {
      if(curValue < aimValue-tol)
	{
	  // Empty muscle 0
	  new_valves[0].V_tank = false; //Close tank valve
	  new_valves[0].V_atm = true;   //Open atm valve
	  // Fill muscle 1
	  new_valves[1].V_tank = true;  //Close tank valve
	  new_valves[1].V_atm = false;  //Open atm valve
	}
      
      if(curValue > aimValue+tol)
	{
	  // Fill muscle 0
	  new_valves[0].V_tank = true; //Close tank valve
	  new_valves[0].V_atm = false;   //Open atm valve
	  // Empty muscle 1
	  new_valves[1].V_tank = false;  //Close tank valve
	  new_valves[1].V_atm = true;  //Open atm valve
	}
      
    }
  

  std_msgs::Bool msgT0;
  msgT0.data = new_valves[0].V_tank;
  vT0_pub_.publish(msgT0);

  std_msgs::Bool msgA0;
  msgA0.data = new_valves[0].V_atm;
  vA0_pub_.publish(msgA0);

  std_msgs::Bool msgT1;
  msgT1.data = new_valves[1].V_tank;
  vT1_pub_.publish(msgT1);

  std_msgs::Bool msgA1;
  msgA1.data = new_valves[1].V_atm;
  vA1_pub_.publish(msgA1);
  

  return new_valves;
}


Mscl_control::ValvesStruct* Mscl_control::bangBang(Mscl_control::PamPressureStruct* current_pressure, 
					      Mscl_control::BendStruct* current_bend,
					      ValvesStruct* new_valves, //"global", then returned 
					      bool pressureControl, bool bendControl)
{  

  //if ((pressureControl+bendControl) != 1)
  if ((pressureControl ^ bendControl) != true )
  {    
    ROS_INFO("Error: Must choose a type of control.");
    return 0;
  }

  double aimValue;
  double curValue;
  float tol;

  int i = 0;
  for (i = 0; i<NVALVES/2; i++) //2 muscles, 2 valves per muscle
  {	
    if (bendControl == 1)
    {
      aimValue = current_bend[0].bend_aim;
      curValue = current_bend[0].bend_cur;
      tol = B_TOL;
     //Controller:

     //     if((curValue <= aimValue+tol) & (curValue >= aimValue-tol))
      ROS_INFO("curValue: %f, aimValue: %f", curValue, aimValue );
      if((curValue <= aimValue+tol) && (curValue >= aimValue-tol))
       {
	 new_valves[i].V_tank = false; //Close tank valve
	 new_valves[i].V_atm = false; //Close atm valve
       }
      else
	{
	  if(curValue > aimValue+tol)
	   {
	     new_valves[i].V_tank = false; //Close tank valve
	     new_valves[i].V_atm = true; //Open atm valve
	   }
	  
	 if(curValue < aimValue-tol)
	   {
	     new_valves[i].V_tank = true; //Open tank valve
	     new_valves[i].V_atm = false; //Close atm valve
	   }
	 
	}
      
      

    }

    if (pressureControl == 1) //default
    {
      aimValue = current_pressure[i].P_aim;
      curValue = current_pressure[i].P_cur;
      tol = P_TOL;
    
     //Controller:
      
     //     if((curValue <= aimValue+tol) & (curValue >= aimValue-tol))
      ROS_INFO("curValue: %f, aimValue: %f", curValue, aimValue );
      if((curValue <= aimValue+tol) && (curValue >= aimValue-tol))
	{
	 new_valves[i].V_tank = false; //Close tank valve
	 new_valves[i].V_atm = false; //Close atm valve
	}
      else
	{
	 if(curValue > aimValue+tol)
	   {
	     new_valves[i].V_tank = false; //Close tank valve
	     new_valves[i].V_atm = true; //Open atm valve
	   }
	 
	 if(curValue < aimValue-tol)
	   {
	     new_valves[i].V_tank = true; //Open tank valve
	     new_valves[i].V_atm = false; //Close atm valve
	   }
	 
	}
    }


  }	  


  std_msgs::Bool msgT0;
  msgT0.data = new_valves[0].V_tank;
  vT0_pub_.publish(msgT0);

  std_msgs::Bool msgA0;
  msgA0.data = new_valves[0].V_atm;
  vA0_pub_.publish(msgA0);

  std_msgs::Bool msgT1;
  msgT1.data = new_valves[1].V_tank;
  vT1_pub_.publish(msgT1);

  std_msgs::Bool msgA1;
  msgA1.data = new_valves[1].V_atm;
  vA1_pub_.publish(msgA1);
  

  return new_valves;
}
 
bool Mscl_control::spin(void)
{  
  //double P_data[NPSENSORS]; //hard-coded as 14 => labjack_node.h file
  //mask P_data?
  //Mscl_control::Mscl_PVal_ PMscl[NVALVES/2];//2 muscles 
  //Mscl_control::Bend_Val_ bend[NVALVES/2];//2 muscles 
  
  //Mscl_control::Valves_ valves[NVALVES/2];//4 valves, each struct contains 2 valves
  
  //ros::Rate loop_rate_(500); //20 is operating frequency of valves //Data streaming?
  
  //ROS_INFO("Loop commencing");
  //char i = 0;
  //bool lol = true;
  
  //Bangbang
  //   double voltCh0 = 0;
  //double voltCh1 = 0;
  
  long start;
  long end;
  
  // while(ros::ok()) //status?
  // {        
  start = getTickCount(); 
  
  Mscl_control::StreamData_example(hDevice_, &caliInfo_);
  
  //ROS_INFO("Stream_volt2 = %f", stream_volt_[2]);
  
  double freq=5;
  double amp = 1.4;
  double offset = 3.6;
  ref_ = offset+amp/2*(sin((count_*freq*PI)/180)/1); // );
  //ref_ = 3.8;
  
  if(count_ == 50)
    {
      //ref = 2.95;
      //ref_ = 0.45;
    }
  if(count_ == 100) //step
    {
      //ref = 3.3;
      //ref_ = 2.6;
      //count_ = 0;
    }
  
  // lolVoltage0 = lj_.getSensorReading_readCh0(voltCh0);
  // lolVoltage1 = lj_.getSensorReading_readCh1(voltCh1); //Only 1 input read, faster
  
  
  //PMscl[1].P_aim = stream_volt2/2;
  //PMscl[1].P_aim = 1.5;
  //PMscl[1].P_cur = lolVoltage1;
  
  PMscl_[1].P_aim = ref_;
  PMscl_[1].P_cur = stream_volt_[1];
  
  //ROS_INFO("P_data1 = %f", stream_volt_[1]);
  
  PMscl_[0].P_aim = 0;
  PMscl_[0].P_cur =  stream_volt_[0];
  //ROS_INFO("P_data0 = %f", stream_volt_[0]);
  
  bend_[0].bend_aim = ref_;
  bend_[0].bend_cur = 0; //Not attached
  
  bend_[1].bend_aim = ref_;
  
  //bend_[1].bend_cur = 5-stream_volt_[2];//signalFilter(sampleVec, sampleNo);
  bend_[0].bend_cur = stream_volt_[2];//signalFilter(sampleVec, sampleNo);
  
  //ROS_INFO("P_tolerance = +- %f", P_TOL*p_prop_);
  //ROS_INFO("B_tolerance = +- %f", B_TOL);
  
  count_++;
  
  
  if (count_ == 360)
    count_ = 0;
  
  //ROS_INFO("Count = %d", count_);       
  
  
  //Mscl_control::putValves(hDevice_, Mscl_control::bangBang(PMscl, bend, valves_, 1, 0)); //Use bangBang controller to output valves
  // Is this equivalnet?
  ValvesStruct* temp_valve;
  //temp_valve = Mscl_control::bangBang(PMscl_, bend_, valves_, 1, 0); // Pressure control
  //Mscl_control::putValves(hDevice_, temp_valve);

  // Test flexi sensor control
  temp_valve = Mscl_control::bangBangBend(PMscl_, bend_, valves_); // Pressure control
  Mscl_control::putValves(hDevice_, temp_valve);

  // Is hybrid control an later option?

  publish();
  
  
  ros::spinOnce();
  
  loop_rate_.sleep();
  
  end = getTickCount();
  
  //ROS_INFO("whileTick = %i", end - start);
  //}
  
  
  return true;
  
}

int Mscl_control::publish(void)
{
  
  //Publishing pressure values     
  //Muscle 0:
  std_msgs::Float32 msgPaim0;
  msgPaim0.data = (PMscl_[0].P_aim-p_bot_)*p_prop_;
  P0_aim_pub_.publish(msgPaim0);
  
  std_msgs::Float32 msgPcur0;
  msgPcur0.data = (PMscl_[0].P_cur-p_bot_)*p_prop_;
  P0_cur_pub_.publish(msgPcur0);
  
  //Muscle 1:
  std_msgs::Float32 msgPaim1;
  msgPaim1.data = (PMscl_[1].P_aim-p_bot_)*p_prop_;
  P1_aim_pub_.publish(msgPaim1);
  
  std_msgs::Float32 msgPcur1;
  msgPcur1.data = (PMscl_[1].P_cur-p_bot_)*p_prop_;
  P1_cur_pub_.publish(msgPcur1);
  
  
  std_msgs::Float32 msgBendAim;
  msgBendAim.data = bend_[0].bend_aim;
  bend_aim_pub_.publish(msgBendAim);
  
  std_msgs::Float32 msgBendCur;
  msgBendCur.data = bend_[0].bend_cur;
  bend_cur_pub_.publish(msgBendCur);
  
  std_msgs::Float32 msgBendSig;
  double bend_scaling = 12.3;
  msgBendSig.data = (((5-stream_volt_[2])-3)*bend_scaling);
  bend_sig_pub_.publish(msgBendSig);
  
  return 1;

}






 //EXAMPLE U6 STREAMING:

//Sends a ConfigIO low-level command to turn off timers/counters


int Mscl_control::ConfigIO_example(HANDLE hDevice)
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

    for( i = 10; i < 16; i++ )
        sendBuff[i] = 0;   //Reserved
    extendedChecksum(sendBuff, 16);

    //Sending command to U6
    if( (sendChars = LJUSB_Write(hDevice, sendBuff, 16)) < 16 )
    {
        if( sendChars == 0 )
            ROS_INFO("ConfigIO error : write failed");
        else
            ROS_INFO("ConfigIO error : did not write all of the buffer\n");
        return -1;
    }

    //Reading response from U6
    if( (recChars = LJUSB_Read(hDevice, recBuff, 16)) < 16 )
    {
        if( recChars == 0 )
            ROS_INFO("ConfigIO error : read failed\n");
        else
            ROS_INFO("ConfigIO error : did not read all of the buffer\n");
        return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, 15);
    if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5] )
    {
        ROS_INFO("ConfigIO error : read buffer has bad checksum16(MSB)\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4] )
    {
        ROS_INFO("ConfigIO error : read buffer has bad checksum16(LSB)\n");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0] )
    {
        ROS_INFO("ConfigIO error : read buffer has bad checksum8\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x05) || recBuff[3] != (uint8)(0x0B) )
    {
        ROS_INFO("ConfigIO error : read buffer has wrong command bytes\n");
        return -1;
    }

    if( recBuff[6] != 0 )
    {
        ROS_INFO("ConfigIO error : read buffer received errorcode %d\n", recBuff[6]);
        return -1;
    }

    if( recBuff[8] != 0 )
    {
        ROS_INFO("ConfigIO error : NumberTimersEnabled was not set to 0\n");
        return -1;
    }

    if( recBuff[9] != 0 )
    {
        ROS_INFO("ConfigIO error : CounterEnable was not set to 0\n");
        return -1;
    }
    return 0;
}

//Sends a StreamConfig low-level command to configure the stream.
int Mscl_control::StreamConfig_example(HANDLE hDevice)
{
    int sendBuffSize;
    sendBuffSize = 14+NumChannels_*2;
    uint8 sendBuff[sendBuffSize], recBuff[8];
    int sendChars, recChars;
    uint16 checksumTotal;
    uint16 scanInterval;
    int i;

    sendBuff[1] = (uint8)(0xF8);     //Command byte
    sendBuff[2] = 4 + NumChannels_;   //Number of data words = NumChannels + 4
    sendBuff[3] = (uint8)(0x11);     //Extended command number
    sendBuff[6] = NumChannels_;       //NumChannels
    sendBuff[7] = 1;                 //ResolutionIndex
    sendBuff[8] = SamplesPerPacket_;  //SamplesPerPacket
    sendBuff[9] = 0;                 //Reserved
    sendBuff[10] = 0;                //SettlingFactor: 0
    sendBuff[11] = 0;  //ScanConfig:
                       //  Bit 3: Internal stream clock frequency = b0: 4 MHz
                       //  Bit 1: Divide Clock by 256 = b0

    scanInterval = 4000;
    sendBuff[12] = (uint8)(scanInterval&(0x00FF));  //scan interval (low byte)
    sendBuff[13] = (uint8)(scanInterval/256);       //scan interval (high byte)

    for( i = 0; i < NumChannels_; i++ )
    {
        sendBuff[14 + i*2] = i;  //ChannelNumber (Positive) = i
        sendBuff[15 + i*2] = 0;  //ChannelOptions: 
                                 //  Bit 7: Differential = 0
                                 //  Bit 5-4: GainIndex = 0 (+-10V)
    }

    extendedChecksum(sendBuff, sendBuffSize);

    //Sending command to U6
    sendChars = LJUSB_Write(hDevice, sendBuff, sendBuffSize);
    if( sendChars < sendBuffSize )
    {
        if( sendChars == 0 )
            ROS_INFO("Error : write failed (StreamConfig).\n");
        else
            ROS_INFO("Error : did not write all of the buffer (StreamConfig).\n");
        return -1;
    }

    for( i = 0; i < 8; i++ )
        recBuff[i] = 0;

    //Reading response from U6
    recChars = LJUSB_Read(hDevice, recBuff, 8);
    if( recChars < 8 )
    {
        if( recChars == 0 )
            ROS_INFO("Error : read failed (StreamConfig).\n");
        else
            ROS_INFO("Error : did not read all of the buffer, %d (StreamConfig).\n", recChars);

        for( i = 0; i < 8; i++)
            ROS_INFO("%d ", recBuff[i]);

        return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, 8);
    if( (uint8)((checksumTotal / 256) & 0xff) != recBuff[5])
    {
        ROS_INFO("Error : read buffer has bad checksum16(MSB) (StreamConfig).\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4] )
    {
        ROS_INFO("Error : read buffer has bad checksum16(LSB) (StreamConfig).\n");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0] )
    {
        ROS_INFO("Error : read buffer has bad checksum8 (StreamConfig).\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x01) || recBuff[3] != (uint8)(0x11) || recBuff[7] != (uint8)(0x00) )
    {
        ROS_INFO("Error : read buffer has wrong command bytes (StreamConfig).\n");
        return -1;
    }

    if( recBuff[6] != 0 )
    {
        ROS_INFO("Errorcode # %d from StreamConfig read.\n", (unsigned int)recBuff[6]);
        return -1;
    }

    return 0;
}

//Sends a StreamStart low-level command to start streaming.
int Mscl_control::StreamStart(HANDLE hDevice)
{
    uint8 sendBuff[2], recBuff[4];
    int sendChars, recChars;

    sendBuff[0] = (uint8)(0xA8);  //Checksum8
    sendBuff[1] = (uint8)(0xA8);  //Command byte

    //Sending command to U6
    sendChars = LJUSB_Write(hDevice, sendBuff, 2);
    if( sendChars < 2 )
    {
        if( sendChars == 0 )
            ROS_INFO("Error : write failed.\n");
        else
            ROS_INFO("Error : did not write all of the buffer.\n");
        return -1;
    }

    //Reading response from U6
    recChars = LJUSB_Read(hDevice, recBuff, 4);
    if( recChars < 4 )
    {
        if( recChars == 0 )
            ROS_INFO("Error : read failed.\n");
        else
            ROS_INFO("Error : did not read all of the buffer.\n");
        return -1;
    }

    if( normalChecksum8(recBuff, 4) != recBuff[0] )
    {
        ROS_INFO("Error : read buffer has bad checksum8 (StreamStart).\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xA9) || recBuff[3] != (uint8)(0x00) )
    {
        ROS_INFO("Error : read buffer has wrong command bytes \n");
        return -1;
    }

    if( recBuff[2] != 0 )
    {
        ROS_INFO("Errorcode # %d from StreamStart read.\n", (unsigned int)recBuff[2]);
        return -1;
    }

    return 0;
}

//Reads the StreamData low-level function response in a loop.
//All voltages from the stream are stored in the voltages 2D array.
int Mscl_control::StreamData_example(HANDLE hDevice, u6CalibrationInfo *caliInfo)
{
    int recBuffSize;
    recBuffSize = 14 + SamplesPerPacket_*2;
    int recChars; //, backLog;
    int i, j, k, m, packetCounter, currChannel, scanNumber;
    int totalPackets;  //The total number of StreamData responses read
    uint16 voltageBytes; //, checksumTotal;
    long startTime, endTime;
    int autoRecoveryOn;

    int numDisplay;          //Number of times to display streaming information
    int numReadsPerDisplay;  //Number of packets to read before displaying streaming information
    int readSizeMultiplier;  //Multiplier for the StreamData receive buffer size
    int responseSize;        //The number of bytes in a StreamData response (differs with SamplesPerPacket)

    numDisplay = 6;
    numReadsPerDisplay = 24;
    readSizeMultiplier = SamplesPerPacket_/NumChannels_; //5;
    responseSize = 14 + SamplesPerPacket_*2;

    /* Each StreamData response contains (SamplesPerPacket / NumChannels) * readSizeMultiplier
     * samples for each channel.
     * Total number of scans = (SamplesPerPacket / NumChannels) * readSizeMultiplier * numReadsPerDisplay * numDisplay
     */
    double voltages[(SamplesPerPacket_/NumChannels_)*readSizeMultiplier][NumChannels_];//[(SamplesPerPacket/NumChannels)*readSizeMultiplier*numReadsPerDisplay*numDisplay][NumChannels];
    uint8 recBuff[responseSize*readSizeMultiplier];
    packetCounter = 0;
    currChannel = 0;
    scanNumber = 0;
    totalPackets = 0;
    recChars = 0;
    autoRecoveryOn = 0;

    startTime = getTickCount();

    //Reading stream response from U6
    recChars = LJUSB_Stream(hDevice, recBuff, responseSize*readSizeMultiplier);

    if( recChars < responseSize*readSizeMultiplier )
    {
      if(recChars == 0)
	ROS_INFO("Error : read failed (StreamData).\n");
      else
	ROS_INFO("Error : did not read all of the buffer, expected %d bytes but received %d(StreamData).\n", responseSize*readSizeMultiplier, recChars);

      return -1;
    }

    //Checking for errors and getting data out of each StreamData response
    for( m = 0; m < readSizeMultiplier; m++ )
    {
	
      //backLog = (int)recBuff[m*48 + 12 + SamplesPerPacket*2];


	for( k = 12; k < (12 + SamplesPerPacket_*2); k += 2 )
        {
	  voltageBytes = (uint16)recBuff[m*recBuffSize + k] + (uint16)recBuff[m*recBuffSize + k+1]*256;

          getAinVoltCalibrated(caliInfo, 1, 0, 0, voltageBytes, &(voltages[scanNumber][currChannel]));

          currChannel++;
	  if( currChannel >= NumChannels_ )
          {
	    currChannel = 0;
            scanNumber++;
	  }		    

        }

     }                  


    for( m = 0; m < 4; m++ )
      stream_volt_[m] = voltages[scanNumber - 1][m];//AI0
    /*stream_volt0 = voltages[scanNumber - 1][0];//AI0
    stream_volt1 = voltages[scanNumber - 1][1];//AI1
    stream_volt2 = voltages[scanNumber - 1][2];//AI2
    stream_volt3 = voltages[scanNumber - 1][3];//AI3
    stream_volt4 = voltages[scanNumber - 1][4];//AI4
    */
       

    endTime = getTickCount();

    return 0;
}

//Sends a StreamStop low-level command to stop streaming.
int Mscl_control::StreamStop(HANDLE hDevice)
{
    uint8 sendBuff[2], recBuff[4];
    int sendChars, recChars;

    sendBuff[0] = (uint8)(0xB0);  //Checksum8
    sendBuff[1] = (uint8)(0xB0);  //Command byte

    //Sending command to U6
    sendChars = LJUSB_Write(hDevice, sendBuff, 2);
    if( sendChars < 2 )
    {
        if( sendChars == 0 )
            ROS_INFO("Error : write failed (StreamStop).\n");
        else
            ROS_INFO("Error : did not write all of the buffer (StreamStop).\n");
        return -1;
    }

    //Reading response from U6
    recChars = LJUSB_Read(hDevice, recBuff, 4);
    if( recChars < 4 )
    {
        if( recChars == 0 )
            ROS_INFO("Error : read failed (StreamStop).\n");
        else
            ROS_INFO("Error : did not read all of the buffer (StreamStop).\n");
        return -1;
    }

    if( normalChecksum8(recBuff, 4) != recBuff[0] )
    {
        ROS_INFO("Error : read buffer has bad checksum8 (StreamStop).\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xB1) || recBuff[3] != (uint8)(0x00) )
    {
        ROS_INFO("Error : read buffer has wrong command bytes (StreamStop).\n");
        return -1;
    }

    if( recBuff[2] != 0 )
    {
        ROS_INFO("Errorcode # %d from StreamStop read.\n", (unsigned int)recBuff[2]);
        return -1;
    }

    /*
    //Reading left over data in stream endpoint.  Only needs to be done with firmwares
    //less than 0.94.
    uint8 recBuffS[64];
    int recCharsS = 64;
    ROS_INFO("Reading left over data from stream endpoint.\n");
    while( recCharsS > 0 )
        recCharsS = LJUSB_Stream(hDevice, recBuffS, 64);
    */

    return 0;
}



//OUTPUT functions:

int Mscl_control::SetDO(HANDLE hDevice, uint16 fio, uint16 eio, uint16 cio) 
{
    uint8 sendBuff[14], recBuff[10]; //
    int sendChars, recChars;
    int len= 14;
    int r_len= 10;
    //    uint16 binVoltage16, 
    uint16 checksumTotal;
    //uint8 state;

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

    extendedChecksum(sendBuff, len);

    //Sending command to U6
    if( (sendChars = LJUSB_BulkWrite(hDevice, U6_PIPE_EP1_OUT, sendBuff, len)) < len)
    {
        if(sendChars == 0)
            ROS_INFO("Feedback setup error : write failed");
        else
            ROS_INFO("Feedback setup error : did not write all of the buffer");
        return -1;
    }

    //Reading response from U6
    if( (recChars = LJUSB_BulkRead(hDevice, U6_PIPE_EP2_IN, recBuff, r_len)) < r_len)
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
