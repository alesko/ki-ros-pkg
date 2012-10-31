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

#ifndef PiTrackerDef_H_
#define PiTrackerDef_H_

#include "PiTracker.h"
#include "PingPong.h"

enum{TRKR_LIB_HS,TRKR_LIB,TRKR_PAT,TRKR_FT,NUM_SUPP_TRKS};

// structure definitions

typedef struct _CNX_PARAMS {
  int  cnxType;
  int  tracker;
  char port[50];
}*LPCNX_PARAMS,CNX_PARAMS;

typedef struct _CNX_STRUCT {
  int        cnxType;
  int        trackerType;
  PiTracker* pTrak;
}*LPCNX_STRUCT,CNX_STRUCT;

typedef struct _CAP_STRUCT{
  FILE* fCap;
  char* filename;
}*LPCAP_STRUCT,CAP_STRUCT;

/*typedef struct _USB_PARAMS {
  int vid;
  int pid;
  int writeEp;
  int readEp;
}*LPUSB_PARAMS,USB_PARAMS;*/

typedef struct _READ_WRITE_STRUCT {
  PingPong*  pPong;
  int&       keepLooping;
  pthread_t* pthread;
  void*      pParam;
}*LPREAD_WRITE_STRUCT,READ_WRITE_STRUCT;

typedef struct _WRITE_STRUCT {
  PingPong*  pPong;
  //  int&       keepLooping;
  pthread_t* pthread;
  //  void*      pParam;
}*LWRITE_STRUCT,WRITE_STRUCT;

#endif
