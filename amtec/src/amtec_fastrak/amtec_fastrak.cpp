
//#include <vector>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/thread/mutex.hpp>
//#include <tf/transform_broadcaster.h>

/*#include <amtec_base.h>
#include <amtec_commands.h>
#include <amtec_settings.h>
#include <amtec_io.h>*/


// messages
//#include <matec/FatrakPose.h>
#include <amtec/FastrakPose.h>
#include <fastrak/Pose.h>
#include <fastrak/StartPublishing.h>
#include <amtec/AmtecState.h>

//Fastrak Services
#include <fastrak/StartPublishing.h>

// services
#include <amtec/GetStatus.h>
#include <amtec/Halt.h>
#include <amtec/Home.h>
#include <amtec/Reset.h>
#include <amtec/SetPosition.h>
#include <amtec/SetVelocity.h>
#include <amtec/SetPositionVelocity.h>
#include <amtec/TargetAcceleration.h>
#include <amtec/TargetVelocity.h>
#include <amtec/SweepPan.h>
#include <amtec/SweepTilt.h>

double ya, ro, pi;
//ros::Time time;
ros::Time callbackTime;
boost::mutex _amtec_fastrak_mutex_ ;
bool new_msg;

//void PoseCallback(const amtec::FastrakPose& msg){
void PoseCallback(const fastrak::Pose& msg){
  
  _amtec_fastrak_mutex_.lock();
  ya = msg.ya;
  ro = msg.ro;
  new_msg = true;
  _amtec_fastrak_mutex_.unlock();
  ROS_INFO("Received Pan[%lf]\tTilt:[%lf]", ya, ro);
}




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "amtec_fastrak"); 

  
  ros::NodeHandle nh;  

  //ros::ServiceClient sp_client = nh.serviceClient<amtec::SetPosition>("amtec/set_position");
  ros::ServiceClient spv_client = nh.serviceClient<amtec::SetPositionVelocity>("amtec/set_position_velocity"); //Temp off
  ros::ServiceClient fastrak_client = nh.serviceClient<fastrak::StartPublishing>("/fastrak_publisher/publishing_service");
  ROS_INFO("ServiceClients created");

  // Call the service to start publishing 
  fastrak::StartPublishing publishing_state_;
  publishing_state_.request.state = true;
  ROS_INFO("Request for fastrak to start publishing" );
  if (fastrak_client.call(publishing_state_))
    ROS_INFO("Fastrak started publishing " );
  else
    ROS_ERROR("Failed call fto fastrak  publishing service");
  //amtec::SetPosition set_position_srv;
  amtec::SetPositionVelocity set_position_velocity_srv;

  ros::Subscriber pose_sub = nh.subscribe("/fastrak/pose_msg", 10, PoseCallback);
  ros::Rate loop_rate(1000);
 
  float new_pan_pos = 0;
  float new_tilt_pos = 0;
  float old_pan_pos = 0;
  float old_tilt_pos = 0;
  ros::Duration dt;
  ros::Time old_time = ros::Time::now();
  float tilt_vel,pan_vel;
  int call_flag;


  while (ros::ok())
  {
    _amtec_fastrak_mutex_.lock();
    if( new_msg )
      {
	new_msg = false;
	// Get new posingtion values
	new_pan_pos = ya;//180.0/3.141592654*ya; // convert to rad
	new_tilt_pos = ro;//180.0/3.141592654*ro; // convert to rad
	//dt = ros::Time::now() - old_time;
	dt =  callbackTime - old_time;
	old_time = callbackTime;
	
	pan_vel = abs(new_pan_pos-old_pan_pos) / dt.toSec();
	tilt_vel = abs(new_tilt_pos-old_tilt_pos) / dt.toSec();
	//std::cout << "DT " << dt.toSec() << " Pan vel" << pan_vel << " Tilt vel" << tilt_vel << std::endl ;
    
	set_position_velocity_srv.request.velocity_pan = pan_vel ;
	set_position_velocity_srv.request.velocity_tilt = tilt_vel ;
	
	call_flag = 0;
	if ( sqrt(pow(new_pan_pos-old_pan_pos,2)) > 1){
	  set_position_velocity_srv.request.position_pan = new_pan_pos;
	  old_pan_pos = new_pan_pos;
	  call_flag = 1;
	}
	
	new_tilt_pos = -new_tilt_pos;
	if ( sqrt(pow(new_tilt_pos-old_tilt_pos,2)) > 1){
	  set_position_velocity_srv.request.position_tilt = new_tilt_pos;
	  old_tilt_pos = new_tilt_pos;
	  call_flag = 1;
	}

	//set_position_srv.request.position_tilt = 
	

	if (    call_flag == 1 ){
	  if (spv_client.call(set_position_velocity_srv))//Temp off
	    if (true)
	      {
		//ROS_INFO("A");
	      }
	    else
	      {
		ROS_ERROR("Fail");
		return 1;
	      }
	}
      }
    _amtec_fastrak_mutex_.unlock();
    ros::spinOnce();
    loop_rate.sleep();
  }


  //ros::spin();
 
  //AmtecFastrak af;
  //af.spin();
 
}
