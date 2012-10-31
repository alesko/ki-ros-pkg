#include "ros/ros.h"
#include "std_msgs/String.h"

#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>


#include <sstream>

 main(int argc, char **argv)
{

  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n;
  ros::Publisher keyboard_pub = n.advertise<std_msgs::String>("key", 10);
  ros::Rate loop_rate(10);

  //int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::string mymsg;

    char c = getchar(); //cvWaitKey(1);
    if(c > -1)    
      {

	std::stringstream ss;
	
	//ss << mymsg;
	ss << c;
	msg.data = ss.str();
	
	keyboard_pub.publish(msg);
	}
    
    ros::spinOnce();
    loop_rate.sleep();

  }

  //tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);

  return 0;
}
