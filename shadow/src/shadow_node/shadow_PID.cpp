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
#include <ros/ros.h>
#include <message_filters/subscriber.h>

#include <shadow/StartPublishing.h>
#include <shadow/ShadowSensors.h>

#include "shadow_PID.h"

void ShadowPID::GloveSensorMsgCallback(const boost::shared_ptr<const sensor_msgs::JointState> &msg)
{
  int i;
  ros::Time t = msg->header.stamp;
  
  //glove_mutex_.lock();
  for(i=0; i < 18;i++)
    glovedata_[i] = msg->position[i];
  //glove_mutex_.unlock();

  //ROS_INFO("Glove data: %f %f %f %f %f", glovedata[0], glovedata[3], glovedata[6], glovedata[9], glovedata[12]);

}

void ShadowPID::ShadowSensorMsgCallback(const boost::shared_ptr<const shadow::ShadowSensors> &msg)
{
  int i;
  ros::Time t = msg->header.stamp;
  
  //shadow_mutex_.lock();
  for(i=0; i < 8;i++)
    sensordata_[i] = msg->sensor[i];
  //shadow_mutex_.unlock();

  ROS_INFO("Sensor data %d %d %d %d %d %d %d %d", sensordata_[0], sensordata_[1], sensordata_[2], sensordata_[3],
	   sensordata_[4], sensordata_[5], sensordata_[6], sensordata_[7]);

}

ShadowPID::ShadowPID(): //double P, double I, double D, double I1, double I2):
  loop_rate(120.0)
{
  control_loop_rate=120.0;
}

//ShadowPID::ShadowPID(double P, double I, double D, double I1, double I2):
//  loop_rate(50.0)

void ShadowPID::initPID(double P, double I, double D, double I1, double I2)
{
  
  client_pub = nh.serviceClient<shadow::StartPublishing>("/shadow/publishing_service");
  client_valve = nh.serviceClient<shadow::PulseValves>("/shadow/pulse_valves");
  //client_controller = nh.serviceClient<shadow::SetController>("/shadow/enable_controller");
  //client_target = nh.serviceClient<shadow::SetTargets>("/shadow/set_targets");
  //client_glove = nh.serviceClient<cyberglove::Start>("/cyberglove_publisher/start");


  //shadow_sensor_sub = nh.subscribe("/shadow/sensor_msg", 5, ShadowSensorMsgCallback);
  /*ShadowPID pid_object(0.03, 0.00, 0.00, 0.0, -0.0);
    shadow_sensor_sub = nh.subscribe("/shadow/sensor_msg", 5, &ShadowPID::ShadowSensorMsgCallbackNEW, &pid_object);*/

  //message_filters::Subscriber<shadow::ShadowSensors> shadow_sensor_sub(nh, "/shadow/sensor_msg" , 5);
  //shadow_sensor_sub.registerCallback(ShadowSensorMsgCallback);

  setpoint_pub_ = nh.advertise<shadow::ShadowSensors>("/shadow/setpoint_msg",5 ); // Que len


  /*
  client_glove = nh.serviceClient<dataglove::Start>("/dataglove_publisher/start");
  
  // Call the dataglove service to start publishing 
  glove_publishing_state_.request.start = true;
  if (client_glove.call(glove_publishing_state_))
    {
      ROS_INFO("Glove publishing service started" );
      glove_sub = nh.subscribe("/raw/joint_states", 10, GloveSensorMsgCallback);
    }
  else
    ROS_ERROR("Failed to call glove publishing service");
   */


  /* 
  // These parameters are for the onboard PID on the SPCU
  // a bug in the software is reported...
  int mP=12;
  int mI=-8;
  int mD=0;
  control_val.request.Controller_valve = 0;
  control_val.request.Controller_sensor = 0;
  control_val.request.Controller_P = mP;
  control_val.request.Controller_I = mI;
  control_val.request.Controller_D = mD;
  if(client_controller.call(control_val))
    ROS_INFO("Controller 0 started" );
  else
    ROS_ERROR("Failed to start controller 0");

  control_val.request.Controller_valve = 1;
  control_val.request.Controller_sensor = 0;
  control_val.request.Controller_P = -mP;
  control_val.request.Controller_I = -mI;
  control_val.request.Controller_D = -mD;
  if(client_controller.call(control_val))
    ROS_INFO("Controller 1 started" );
  else
    ROS_ERROR("Failed to start controller 1");

  control_val.request.Controller_valve = 2;
  control_val.request.Controller_sensor = 1;
  control_val.request.Controller_P = mP;
  control_val.request.Controller_I = mI;
  control_val.request.Controller_D = mD;
  if(client_controller.call(control_val))
    ROS_INFO("Controller 2 started" );
  else
    ROS_ERROR("Failed to start controller 2");

  control_val.request.Controller_valve = 3;
  control_val.request.Controller_sensor = 1;
  control_val.request.Controller_P = -mP;
  control_val.request.Controller_I = -mI;
  control_val.request.Controller_D = -mD; 
  if(client_controller.call(control_val))
    ROS_INFO("Controller 3 started" );
  else
    ROS_ERROR("Failed to start controller 3");
*/

  pid_controller_[0].initPid(P,I,D,I1,I2);  
  pid_controller_[1].initPid(P,I,D,I1,I2);  
  pid_controller_[2].initPid(P,I,D,I1,I2);  
  pid_controller_[3].initPid(P,I,D,I1,I2);
  //controllerStarting();
  ROS_INFO("Shadow PID is initialized" );
  ROS_INFO("Shadow PID is created" );
}

void ShadowPID::initCallback()
{

  ShadowPID pid_object;//0.03, 0.00, 0.00, 0.0, -0.0);
  shadow_sensor_sub = nh.subscribe("/shadow/sensor_msg", 5, &ShadowPID::ShadowSensorMsgCallback, &pid_object);

  // Call the service to start publishing 
  publishing_state_.request.start = true;
  if (client_pub.call(publishing_state_))
    ROS_INFO("Shadow publishing service started" );
  else
    ROS_ERROR("Failed to call service shadow publishing service");

  controllerStarting();

}

void ShadowPID::initGloveCallback()
{
  ShadowPID pid_object;//0.03, 0.00, 0.00, 0.0, -0.0);
  client_glove = nh.serviceClient<dataglove::Start>("/dataglove_publisher/start");
  
  // Call the dataglove service to start publishing 
  glove_publishing_state_.request.start = true;
  if (client_glove.call(glove_publishing_state_))
    {
      ROS_INFO("Glove publishing service started" );
      glove_sub = nh.subscribe("/raw/joint_states", 10, &ShadowPID::GloveSensorMsgCallback, &pid_object);
    }
  else
    ROS_ERROR("Failed to call glove publishing service");

}

ShadowPID::~ShadowPID()
{
}

void ShadowPID::controllerStarting()
{
  //init_pos_ = joint_state_->position_;
  //time_of_last_cycle_ = robot_->getTime();
  int i;
  for(i = 0; i < 4; i++)
    pid_controller_[i].reset();

}

void ShadowPID::controllerStopping()
{
}

void ShadowPID::controllerUpdate(double position_desi_[4])
{

  ros::Time time = ros::Time::now();
  double effort_hi;
  double effort_low;
  int i;
  int valve_ms[8]={0,0,0,0,0,0,0,0};

  /*for(i = 0; i < 4; i++)
    printf("position_desi_[%d]: %f ",i, position_desi_[i]);
  printf("\n");*/

  if( sensordata_[0] !=0) // First time...
    {
      for(i = 0; i < 2; i++){
	//printf("i: %d: %d %d\n",i, i*2, i*2+1);
	effort_hi = pid_controller_[i*2].updatePid((double) sensordata_[i] - (position_desi_[i]+10), time - last_time);
	effort_low = pid_controller_[i*2+1].updatePid((double) sensordata_[i] - (position_desi_[i]-10), time - last_time);
	//printf("i: %d: %d %d\n",i, i*2, i*2+1);
	//printf("hi: %f:\nlow: %f\n",effort_hi, effort_low);
	
	
	if( effort_hi > 0){
	  if( effort_hi < (1000/control_loop_rate))
	    {
	      //valve_states_.request.valve0_time_ms = (int) effort_hi;
	      valve_ms[i*2] = (int) effort_hi;
	    }
	  else
	    {
	      //valve_states_.request.valve0_time_ms = (int) 1000/control_loop_rate;
	      valve_ms[i*2] = (int) 1000/control_loop_rate;
	    }
	}
	if( effort_low < 0){
	  effort_low = -effort_low;
	  if( effort_low < (1000/control_loop_rate))
	    {
	      //valve_states_.request.valve1_time_ms = (int) effort_low;
	      valve_ms[i*2+1] = (int) effort_low;
	    }
	  else
	    {
	      //valve_states_.request.valve1_time_ms = (int) 1000/control_loop_rate;
	      valve_ms[i*2+1] = (int) 1000/control_loop_rate;
	    }
	}	
	
      }
      last_time = time;
      //ROS_INFO("Valve ms\t%d\t%d\t%d\t%d", valve_ms[0],valve_ms[1],valve_ms[2],valve_ms[3]);
      ROS_INFO("Sensor[0] %d, des: %f Low %f High %lf\t%d:%d", (int) sensordata_[0], position_desi_[0], effort_low, effort_hi,valve_states_.request.valve0_time_ms, valve_states_.request.valve1_time_ms );
      valve_states_.request.valve0_time_ms = valve_ms[0];
      valve_states_.request.valve1_time_ms = valve_ms[1];
      valve_states_.request.valve2_time_ms = valve_ms[2];
      valve_states_.request.valve3_time_ms = valve_ms[3];
      valve_states_.request.valve4_time_ms = valve_ms[4];
      valve_states_.request.valve5_time_ms = valve_ms[5];
      valve_states_.request.valve6_time_ms = valve_ms[6];
      valve_states_.request.valve7_time_ms = valve_ms[7];
      client_valve.call(valve_states_);
      //ROS_INFO("Sensor[0] %d, des: %l.1f Low %lf High %lf\t%d:%d", (int) sensordata[0], position_desi_, effort_low, effort_hi,valve_states_.request.valve0_time_ms, valve_states_.request.valve1_time_ms );
    }
  
  //ros::spinOnce();
  //loop_rate.sleep();
  
  //double desired_pos = init_pos_ + amplitude_ * sin(ros::Time::now().toSec());
  //double current_pos = joint_state_->position_;

  //ros::Duration dt = robot_->getTime() - time_of_last_cycle_;
  //time_of_last_cycle_ = robot_->getTime();
  //joint_state_->commanded_effort_ = pid_controller_.updatePid(current_pos-desired_pos, dt);
}

void ShadowPID::setTarget(double position_desi_[4])
{
 
  for(int i=0; i < 4; i++){
    target_values.request.target[2*i] = position_desi_[i] -25;
    target_values.request.target[2*i+1] = position_desi_[i] +25;
  }
  
  if (!client_target.call(target_values))
    ROS_ERROR("Failed to call service /shadow/set_target");
  
}

// Unnecessary to publish the setpoint
/*void ShadowPID::publishSetPoint()
{
 
  setpoint_msg.header.stamp = ros::Time::now();    
   
  //publish the msgs
  setpoint_pub_.publish(setpoint_msg);

  }*/

void ShadowPID::alterSetPoint()
{
  double t = ros::Time::now().toSec();
  /*set_point[0] = 1100 + 500 * sin(5 * t);
  set_point[1] = 1100 - 500 * sin(5 * t);
  set_point[2] = 0;
  set_point[3] = 0;
  setpoint_msg.sensor[0] = set_point[0];
  setpoint_msg.sensor[1] = set_point[1];*/
  
  // Inverse kinematics
  // replace with a calibration process
  float s_max = 1900;
  float s_min =  500;
  float g_max = 2500;
  float g_min = 1700;
  float dg =g_max-g_min;
  float ds =s_max-s_min;
  float k = ds/dg;
  float d1 = -g_min * k+s_min;
  float d2 = g_min * k+s_max;
  
  set_point[0] = glovedata_[3]*-k+d2;
  set_point[1] = glovedata_[3]*k+d1;
  set_point[2] = 0;
  set_point[3] = 0;
  setpoint_msg.sensor[0] = set_point[0];
  setpoint_msg.sensor[1] = set_point[1];

}

void ShadowPID::controlspin()
{

  while (nh.ok())
    {
      alterSetPoint();
      controllerUpdate(set_point);
      //setTarget(set_point);
      //publishSetPoint();
      ros::spinOnce();
      loop_rate.sleep();
    }  

}

/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "shadow_pressure_PID"); 

  ShadowPID sc(0.03, 0.00, 0.00, 0.0, -0.0); //Get from parameter server instead
  sc.initCallback();
  //sc.track_point = 1000.0;
  //sc.set_point = 1000.0;
  //sc.set_point_dir=80.0;
  sc.controlspin();
  
  
  }*/

