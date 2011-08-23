/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexander Skoglund
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
 
#ifndef SERVO_BASE_H
#define SERVO_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>


#define MAX_NAME_LENGTH              256
#define MAX_NUM_CHAR                 256
#define MAX_NUM_LOOPS                100
#define MAX_LOOP                       1000


// Mask commands
  enum MASK_ACTION {HOLD,FILL,EMPTY,DONT_CARE};

// End of commands
//enum PARITY_TYPE   { N, E, O };

typedef struct {
  char                       ttyport[MAX_NAME_LENGTH];
  int                        baud;
  //enum PARITY_TYPE           parity;
  int                        fd;
  int                        databits;
  int                        stopbits;
  int                        hwf;
  int                        swf;
} servo_device_t, *servo_device_p;


typedef struct {
  /*unsigned int  TimeStamp[2];
  unsigned int  Sensors[8];
  unsigned int  Targets[8];
  unsigned int  ValveStates[8];
  unsigned char SetValveStates;
  unsigned char ActualValveStates;
  unsigned char LATAreg;
  unsigned char LATBreg;
  unsigned char ForceStates;
  unsigned int  Controller_Sensor[8];
  unsigned int  Controller_Target[8];
  unsigned int  Controller_P[8];
  unsigned int  Controller_I[8];
  unsigned int  Controller_D[8];*/
  int channel;
  int position;
} servo_params_t, *servo_params_p;


typedef struct {
  servo_device_t dev;
  servo_params_t par;
} servo_t, *servo_p;
  

servo_p servoInitialize();
void servoClear(servo_p servo);
void servoPrintParams(servo_params_p params);

#ifdef __cplusplus
}
#endif

#endif
