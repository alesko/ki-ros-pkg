#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <cmath>

// messages
#include <fastrak/Pose.h>

// Services
#include <fastrak/StartPublishing.h>

//#include <stdio.h>
//#include <unistd.h>
#include <pthread.h>
//#include <gtk/gtk.h>
#include "PiTrackerDef.h"
#include "PiTracker.h"
#include "PingPong.h"
#include "PiTracker_node.h"
//#include "PiTerm.h"
//#include "fastrak_tracker.h"

#define BUFFER_SIZE   1000

#define MAX_ROW   10
#define MAX_LENGTH 300
char GLOBAL_BUF[MAX_ROW][300];
int GLOBAL_ROW;
int GLOBAL_COL;



// This is the thread that reads data from the tracker and stores in the
// ping pong buffer
void* ReadTrackerThread(void* pParam){

  BYTE buf[BUFFER_SIZE];
  LPREAD_WRITE_STRUCT prs=(LPREAD_WRITE_STRUCT)pParam;
  PiTracker* pTrak=(PiTracker*)prs->pParam;
  int len=0;
  int bw;

  // first establish comm and clear out any residual trash data
  do {
    pTrak->WriteTrkData((void*)"\r",1);  // send just a cr, should return an short "Invalid Command" response
    usleep(100000);
    len=pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response
  } while (!len);

  pTrak->WriteTrkData((void*)"C",1);

  while (prs->keepLooping){
    len=pTrak->ReadTrkData(buf,BUFFER_SIZE);  // read tracker data
    if (len>0 && len<BUFFER_SIZE){
      buf[len]=0;  // null terminate
      do {
	bw=prs->pPong->WritePP(buf,len);  // write to buffer
	usleep(1000);
      }while(!bw);
    }
    usleep(2000);  // rest for 2ms
  }
  pTrak->WriteTrkData((void*)"c",1);

  return NULL;
}


bool PiTrackerNode::Write2Buffer(void* pParam){
  //bool PiTrackerNode::Write2Buffer(){

  LPREAD_WRITE_STRUCT lws=(LPREAD_WRITE_STRUCT)pParam;  
  BYTE buf[BUFFER_SIZE];

  int i;
  int len=lws->pPong->ReadPP(buf);
  //int len=writeStruct.pPong->ReadPP(buf);
  int row;
  int flag;
  while (len){
    buf[len]=0;
    for(i=0; i < len; i++){
      flag=0;
      if(0x0a == buf[i] ){
	GLOBAL_ROW++;
	flag = 1;
      }
      if(0x0d == buf[i]){
	GLOBAL_COL=0;
	flag = 1;
      }
      row = GLOBAL_ROW%10;
      if(0 == flag){
	GLOBAL_BUF[row][GLOBAL_COL]=buf[i];
      }
      GLOBAL_COL++;
    }        
    len=lws->pPong->ReadPP(buf);
    //len=writeStruct.pPong->ReadPP(buf);
  }

  ros::spinOnce();
  //publish_rate.sleep();
  return 1;
}


PiTrackerNode::PiTrackerNode(std::string dev)
  : private_nh("~"), publishing(false), publish_rate(100.0)
{

  /*if (pong.InitPingPong(BUFFER_SIZE)<0){
    //fprintf(stderr,"Memory Allocation Error setting up buffers\n");
    ROS_FATAL("Memory Allocation Error setting up buffers");
    //return -1;
  }

  cnxStruct.cnxType=RS232_CNX;
  cnxStruct.trackerType=TRKR_FT;
  //strcpy(cp.port,dev.c_str());
  
  //ROS_INFO("Port: %s",dev.c_str());
  //char myport[50];
  strcpy( ttyport , dev.c_str());
  //ROS_INFO("Port: %s",myport);
  //strcpy( cnxStructPtr->port , dev.c_str());

 

  // The communication with the tracker
  cnxStruct.pTrak=new PiTracker;
  if (!cnxStruct.pTrak){
    ROS_FATAL("Memory Allocation Error creating tracker communications module");
    //printf("Memory Allocation Error creating tracker communications module\n");
    //return -3;
  }
  
  int cnxSuccess;

  if (cnxStruct.cnxType==RS232_CNX){
    cnxSuccess=cnxStruct.pTrak->Rs232Connect(ttyport); //pcs->pTrak->Rs232Connect(port);
    if (cnxSuccess!=0){
       ROS_FATAL("No connection with tracker over RS232");
       free(ttyport);
       //return 1;
     }
    ROS_INFO("Connected to fastrak on%s",ttyport);
  }

  int keepLooping = 1;
  // create structs to be used in Read thread and the write timeout fxn

  //READ_WRITE_STRUCT 
  readStruct.pPong = &pong;
  readStruct.keepLooping=keepLooping;
  readStruct.pthread=&thread_id;
  //readStruct.pParam=cnxStruct.pTrak;
  readStruct.pParam=ttyport;

  //writeStruct.pPong = &pong;
  //writeStruct.pthread=&thread_id;
  //writeStruct.pParam=NULL;
  //writeStruct.keepLooping=0; //keepLooping;
  ROS_INFO("1");

  //READ_WRITE_STRUCT 
  //writeStruct={&pong,keepLooping,&thread_id,NULL};  // will add textview after it's created
  //READ_WRITE_STRUCT publishStruct={&pong,keepLooping,&thread_id,NULL};  // will add textview after it's created

  //readStruct.keepLooping=1;
  //pthread_create(readStruct.pthread,NULL,ReadTrackerThread,&readStruct);
  ROS_INFO("past thread");
  */
  
  double publish_freq=100.0;
  private_nh.param("publish_frequency", publish_freq, 100.0);
  publish_rate = ros::Rate(publish_freq);

  std::string prefix;
  std::string searched_param;
  private_nh.searchParam("fastrak_prefix", searched_param);
  private_nh.param(searched_param, prefix, std::string());
  std::string full_topic = prefix + "/fastrak/pose_msg";
  fastrak_pub = private_nh.advertise<fastrak::Pose>(full_topic, 10);
  
  ROS_INFO("Starting services!");
  publishing_srv = private_nh.advertiseService("publishing_service",&PiTrackerNode::setPublishing, this);//<fastrak::Pose>(full_topic, 10);

  ROS_INFO("Fastrak node is ready!");

}

PiTrackerNode::~PiTrackerNode()
{
  //readStruct.keepLooping=0;
  //usleep(200000); // Wait for thead to exit
  //cnxStruct.pTrak->CloseTrk();  // close down the tracker connection
}

/*
void PiTrackerNode::setup_boresight(int op)
{
  //if (get_logging())
  //  return;

  //One step calibration procedure
  switch(op)
  {
    case INTERACTIVE:
      clear_boresight();
      printf("Place probe at desired origin orientation, then hit enter...");
      getchar();
      set_boresight_to_current();
      printf("Done!\n");
      break;
    case STEP_0:
      clear_boresight();
      break;
    case STEP_1:  //Place probe at origin
      set_boresight_to_current();
      break;
    default:
      #if TraceErrors
      printf("tracker: invalid op given to setup_angles_origin\n");
      #endif
  }
}

void PiTrackerNode::clear_boresight(void)
{
  //if (get_logging())
  //return;
  set_boresight(0.0, 0.0, 0.0);
}

void iTrackerNode::set_boresight(double yaw, double pitch, double roll)
{
  //if (get_logging())
  //return;

  //Wouldn't need to bother with by,bp,br if boresight was working on tracker
  char buf[MAX_LENGTH];

  if (get_angle_units() == ANGLE_UNIT_RAD)
  {
    by = DEG(yaw);
    bp = DEG(pitch);
    br = DEG(roll);
  }
  else
  {
    by = yaw;
    bp = pitch;
    br = roll;
  }

  sprintf(buf, "G%d,%.2lf,%.2lf,%.2lf\r", STATION, by, bp, br);
  port->send_comport(buf);

}

void PiTrackerNode::set_boresight_to_current(void)
{
  //if (get_logging())
  //return;

  //Boresight not working, do it manually
  double yaw, pitch, roll;
  get_pose(NULL, NULL, NULL, &yaw, &pitch, &roll);
  set_boresight(yaw, pitch, roll);
}

int PiTrackerNode::get_boresight(double* yaw, double* pitch, double* roll)
{
  //This would contain what is now in get_system_boresight() if boresight
  //was working on tracker

  if (get_angle_units() == ANGLE_UNIT_RAD)
  {
    *yaw = RAD(by);
    *pitch = RAD(bp);
    *roll = RAD(br);
  }
  else
  {
    *yaw = by;
    *pitch = bp;
    *roll = br;
  }

  return 1;
}


int PiTrackerNode::get_system_boresight(double* yaw, double* pitch, double* roll)
{
  if (get_logging())
    return -1;

  //Read system boresight, only used by constructor
  //This would be get_boresight(), if boresight was working on tracker
  //double y,p,r;
  double values[3];
  char buf[MAX_LENGTH];

  sprintf(buf, "G%d\r", STATION);
  port->send_comport(buf);
  port->recv_comport(buf, MAX_LENGTH);

  //parse_stupid_string(buf, 3, values, 3);
  parse_string(buf, 3, values, 3);
 
  *yaw = values[0];
  *pitch = values[1];
  *roll = values[2];

  return 1;
}
*/

//int parse_string(char* buf, int num_values, double* values, int ignore_first)
int PiTrackerNode::parse_string(char* buf, int num_values, double* values, int ignore_first)
{
  char item[MAX_LENGTH];
  int i,j;
  int start[num_values];
  int end[num_values];

  //Output from tracker is NOT formatted well!!!
  //Spaces separate the numbers if they are positive, but negative numbers
  //can cause two numbers to be side by side
  //This is a sloppy brute force parsing algorithm for these situations

  //Find start and end of all numbers (numbers consist of digits, -, or .)
  //'-' can only be at the start of a number

  for (i=0;i<num_values;i++)
  {
    //Ignore first chars
    if (i==0)
      start[i]=ignore_first;
    else
      start[i]=end[i-1]+1;
    
    while(((buf[start[i]] < '0') || (buf[start[i]] > '9')) && (buf[start[i]] != '-') && (buf[start[i]] != '.'))
      start[i]++;

    end[i]=start[i]+1;
    while(((buf[end[i]] >= '0') && (buf[end[i]] <= '9')) || (buf[end[i]] == '.'))
      end[i]++;
    end[i]--;

    for (j=start[i];j<=end[i];j++)
      item[j-start[i]] = buf[j];

    item[j-start[i]] = '\0';

    if (sscanf(item, "%lf", &(values[i])) != 1)
    {
      //#if TraceErrors
        printf("tracker: error parsing string\n");
        //#endif
        return -1;
    }

  }


  return 1;
}


bool PiTrackerNode::isPublishing()
{
  if (publishing)
    {
      return true;
      }
  else
    {
      ros::spinOnce();
      //publish_rate.sleep();
      return false;
    }
}

//void PiTrackerNode::setPublishing(bool value){
bool PiTrackerNode::setPublishing(fastrak::StartPublishing::Request &req, fastrak::StartPublishing::Response &resp)
{
  if(req.state)
    {
      ROS_INFO("Fastrak will start publishing.");
      publishing = true;
      resp.state = true;
    }
  else
    {
      publishing = false;
      resp.state = false;
      ROS_INFO("Fastrak has stopped publishing.");
    }
  return 1;
}
void PiTrackerNode::setKeepLooping(bool value){
  keepLooping = value;
}


int PiTrackerNode::PublishData(void){

  int ok;
  int row;
  double values[7];
  if( GLOBAL_ROW > 0){
    row = (GLOBAL_ROW-1)%10;
    //printf("row:%d ", row);
    ok = parse_string(GLOBAL_BUF[row], 7, values, 0);
    if(1==ok){
      //printf("%lf:\t\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",values[0],values[1],values[2],values[3],values[4],values[5], values[6]);
      if (0 < values[0] || values[0] < 5 )
      {
	// Should be in inches and degrees
	pose_msg.header.stamp = ros::Time::now();
	pose_msg.id = (int)values[0];
	pose_msg.x  = values[1];
	pose_msg.y  = values[2];
	pose_msg.z  = values[3];
	pose_msg.ya = values[4];
	pose_msg.pi = values[5];
	pose_msg.ro = values[6];
	fastrak_pub.publish(pose_msg);
      }
    }
    
  }
  ros::spinOnce();
  //publish_rate.sleep();
  return 1;
}

bool PiTrackerNode::spin()
{
  while(node_.ok())
    {
      if(publishing)
	{
	  
	  PublishData();
	}
      else
	{
	  ros::spinOnce();
	  publish_rate.sleep();
	}
      publish_rate.sleep();
    }
  return true;
}

int main(int argc,char* argv[]){

  // Init globals
  GLOBAL_ROW=0;
  GLOBAL_COL=0;

  if (argc != 2)
    {
      printf("Usage: PiTracker_node DEVICE\n");
      return 1;
    }
  //const char* dev = argv[1];  
  std::string fastrak_dev = argv[1];  

  PingPong pong;
  if (pong.InitPingPong(BUFFER_SIZE)<0){
    //fprintf(stderr,"Memory Allocation Error setting up buffers\n");
    ROS_FATAL("Memory Allocation Error setting up buffers");
    //return -1;
  }
 
  ros::init(argc, argv, "fastrak_publisher");
  PiTrackerNode PT_node(fastrak_dev);

  int i;
  CNX_PARAMS cp;
  CNX_STRUCT cnxStruct;

  cnxStruct.cnxType=RS232_CNX;
  cnxStruct.trackerType=TRKR_FT;
  strncpy(cp.port,fastrak_dev.c_str(),50);
  //strncpy(cp.port,"/dev/ttyUSB0",50);

  // The communication with the tracker
  cnxStruct.pTrak=new PiTracker;
  if (!cnxStruct.pTrak){
    printf("Memory Allocation Error creating tracker communications module\n");
    return -3;
  }
  
  int cnxSuccess;

  if (cnxStruct.cnxType==RS232_CNX){
    cnxSuccess=cnxStruct.pTrak->Rs232Connect(cp.port); //pcs->pTrak->Rs232Connect(port);
    if (cnxSuccess!=0){
       ROS_FATAL("No connection with tracker over RS232");
       free(cp.port);
       //return 1;
     }
    ROS_INFO("Connected to fastrak on%s",cp.port);
  }

  char buf[BUFFER_SIZE];
  int len;
  do {
    cnxStruct.pTrak->WriteTrkData((void*)"\r",1);  // send just a cr, should return an short "Invalid Command" response
    usleep(100000);
    len=cnxStruct.pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response
  } while (!len);


  
  //sprintf(buf, "G%d,%.2lf,%.2lf,%.2lf\r", 1, by, bp, br);
  sprintf(buf,"P");
  cnxStruct.pTrak->WriteTrkData((void*)buf,1);
  usleep(100000);
  len = cnxStruct.pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response  
  for(i=0; i < len; i++)
    printf("%c",buf[i]);
  printf("\n");
  //  int i;
  //for( i=0; i < 100; i++)
  //cnxStruct.pTrak->WriteTrkData((void*)buf,1);

  //cnxStruct.pTrak->WriteTrkData(buf,1); // Stop continous mode if presenst

  ROS_INFO("Non-interactively setting up origins...");
  ROS_INFO("Place probe at desired origin orientation, then hit enter...");
  // Clear boresight
  int station_no = 1;
  /*sprintf(buf,"b%d\r",station_no);
  cnxStruct.pTrak->WriteTrkData((void*)buf,1);*/

  //t.setup_boresight(STEP_0); //Indicate start - place probe with desired orientation

  getchar();
  sprintf(buf,"B%d\r",station_no);
  cnxStruct.pTrak->WriteTrkData((void*)buf,3);
  //t.setup_boresight(STEP_1); //Could also just use t.set_boresight_to_current();
  sprintf(buf,"P");
  cnxStruct.pTrak->WriteTrkData((void*)buf,1);
  usleep(100000);
  len = cnxStruct.pTrak->ReadTrkData(buf,BUFFER_SIZE);  // keep trying till we get a response  
  for(i=0; i < len; i++)
    printf("%c",buf[i]);
  printf("\n");

  ROS_INFO("Done!\n");
  usleep(100000);
  
  int keepLooping=1;
  pthread_t thread_id;
  //READ_WRITE_STRUCT 
  READ_WRITE_STRUCT readStruct={&pong,keepLooping,&thread_id,cnxStruct.pTrak};
  /*  readStruct.pPong = &pong;
  readStruct.keepLooping=keepLooping;
  readStruct.pthread=&thread_id;
  readStruct.pParam=cnxStruct.pTrak;*/
  //readStruct.pParam=ttyport;

  /*writeStruct.pPong = &pong;
  writeStruct.pthread=&thread_id;
  writeStruct.pParam=NULL;
  writeStruct.keepLooping=0; //keepLooping;*/
  READ_WRITE_STRUCT writeStruct={&pong,keepLooping,&thread_id,NULL};
  ROS_INFO("1");

  //READ_WRITE_STRUCT 
  //writeStruct={&pong,keepLooping,&thread_id,NULL};  // will add textview after it's created
  //READ_WRITE_STRUCT publishStruct={&pong,keepLooping,&thread_id,NULL};  // will add textview after it's created

  //readStruct.keepLooping=1;
  pthread_create(readStruct.pthread,NULL,ReadTrackerThread,&readStruct);
  ROS_INFO("past thread");



  //for(i=0; i<100; i++){
  //  usleep(10000);
    //pthread_create(writeStruct.pthread,NULL,Write2Display,&writeStruct);
    //pthread_create(publishStruct.pthread,NULL,PublishData,&PT_node);
  //  PT_node.Write2Buffer(&writeStruct);
  //  PT_node.PublishData();
  //}
  //PT_node.setKeepLooping(true);  
  //PT_node.ReadTracker(&readStruct);
  while(PT_node.node_.ok())
    {
      if(PT_node.publishing)
	{
	  //PT_node.Write2Buffer(&writeStruct);
	  PT_node.Write2Buffer(&writeStruct);
	  //PT_node.Write2Buffer();
	  PT_node.PublishData();
	}
      else
	{
	  ros::spinOnce();
	  PT_node.publish_rate.sleep();
	}
      PT_node.publish_rate.sleep();
    }
  
  readStruct.keepLooping=0;
  usleep(200000);

  printf("Exit\n");


}

