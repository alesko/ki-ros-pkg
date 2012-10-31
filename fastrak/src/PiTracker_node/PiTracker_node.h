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

#ifndef PiTracker_Node_H_
#define PiTracker_Node_H_

#include <string>
#include "PiTrackerDef.h"
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

//Messages
#include <fastrak/Pose.h>

// Services
#include <fastrak/StartPublishing.h>



//using namespace ros;

class PiTrackerNode{

 private:
  

  pthread_t thread_id;
  PingPong pong;
  CNX_PARAMS cp;
  CNX_STRUCT cnxStruct;
  LPCNX_PARAMS cnxStructPtr;
  
  ros::NodeHandle private_nh;
  fastrak::Pose pose_msg;
  bool keepLooping;
  ros::ServiceServer publishing_srv;

  bool setPublishing(fastrak::StartPublishing::Request &req, fastrak::StartPublishing::Response &resp);
  int parse_string(char* buf, int num_values, double* values, int ignore_first);
  std::string device;
  
 public:
  char ttyport[50];
  //WRITE_STRUCT writeStruct;//={&pong,keepLooping,&thread_id,NULL};  // will add textview after it's created

  bool publishing;
  ros::NodeHandle node_;
  ros::Rate publish_rate;


  PiTrackerNode(std::string dev);
  ~PiTrackerNode();
  int PublishData(void);
  ros::Publisher fastrak_pub;
  //void publish();
  bool isPublishing();
  //void setPublishing(bool value);

  void setKeepLooping(bool value);
  bool spin();
  bool Write2Buffer(void* pParam);
  //bool Write2Buffer();
  //bool ReadTracker(void* pParam);

};
void* ReadTrackerThread(void* pParam);
//void* Write2Display(void* pParam);

//void OnConnect(LPCNX_STRUCT pcs, char* port, LPREAD_WRITE_STRUCT prs, LPREAD_WRITE_STRUCT p);
//int parse_string(char* buf, int num_values, double* values, int ignore_first);



// usb vid/pids for Polehemus trackers
//USB_PARAMS usbTrkParams[NUM_SUPP_TRKS]={
//  {0x0f44,0xff20,0x04,0x88},  // Lib HS
//  {0x0f44,0xff12,0x02,0x82},   // Lib
//  {0x0f44,0xef12,0x02,0x82},  // Patriot
//  {0x0f44,0x0002,0x02,0x82}};  // Fastrak

// polhemus tracker names
//const gchar* trackerNames[NUM_SUPP_TRKS]={
//  "High Speed Liberty","Liberty","Patriot","Fastrak"};

// definitions for the GTK+ callbacks and other worker functions
/*
void OnAbout(GtkWidget*,GtkWindow*);
void OnClear(GtkWidget*,GtkTextView*);
void OnCapture(GtkToggleButton*,LPCAP_STRUCT);
void OnCaptureBrowse(GtkWidget*,LPCAP_STRUCT);
int OpenCaptureFile(LPCAP_STRUCT);
void OnCnxType(GtkToggleButton*,LPCNX_STRUCT);
void OnConnect(GtkWidget*,LPCNX_STRUCT);
void OnDisconnect(GtkWidget*,LPCNX_STRUCT);
int QueryUser4TrackerType(int&);
void TrackerSelect(GtkToggleButton*,int*);
gboolean CommandEntered(GtkEntry*,GdkEventKey*,LPCNX_STRUCT);
int Browse4CaptureFile(LPCAP_STRUCT);
void* ReadTrackerThread(void*);
gboolean Write2Display(gpointer);
*/


#endif
