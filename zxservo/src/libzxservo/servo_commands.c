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

//#include "shadow_base.h"
#include "servo_io.h"
#include "servo_commands.h"

/**********************************************************/
/* SERVO ROBOTICS SPCU COMMANDS                          */
/**********************************************************/

int servoSetBaudrate(servo_device_p dev)
{
  unsigned char cmd[MAX_NUM_CHAR];
  //int i,j,k;
  unsigned char answer[MAX_NUM_CHAR];

  sprintf(cmd,"!SCSBR%c%c",0x01,0xD);
  /*for(k=0; k<  strlen(cmd); k++){
    printf("%.2x ",cmd[k]);
    }*/
  servoSendReceiveCommand(dev->fd, cmd,strlen(cmd), &answer,0 );
  servoDeviceSetBaudrate(dev, 38400);

  return 1;
}

int servoFirmwareVersion(servo_device_p dev)
{
  unsigned char cmd[MAX_NUM_CHAR];
  //int i,j,k;
  unsigned char answer[MAX_NUM_CHAR];

  sprintf(cmd,"!SCVER?%c",0xD);
  /*for(k=0; k<  strlen(cmd); k++){
    printf("%.2x ",cmd[k]);
    }*/
  servoSendReceiveCommand(dev->fd, cmd,strlen(cmd), &answer,3 );
  //printf("\n%s\n\n",answer);

  return 1;
}

int servoReportPosition(servo_device_p dev, unsigned char ch, int *pos)
{
  printf("servoReportPosition\n");
  unsigned char cmd[MAX_NUM_CHAR];
  //int i,j,k;
  unsigned char answer[MAX_NUM_CHAR];
  //int position;
  
  sprintf(cmd,"!SCRSP%c%c",ch,0xD);
  /*for(k=0; k< 8; k++){
    printf("%.2x ",cmd[k]);
    }*/
  servoSendReceiveCommand(dev->fd, cmd,8, &answer, 3 );
  /*for(k=0; k< 3; k++){
    printf("%.2x ",answer[k]);
    }*/
  *pos =  ((int)answer[1]<<8)+(int)answer[2];


  return 1;
}

int servoSetPosition(servo_device_p dev, unsigned char ch, unsigned char ra,
		     int pos)
{
  //printf("servoSetPosition\n");

  unsigned char cmd[MAX_NUM_CHAR];
  //int i,j,k;
  unsigned char answer[MAX_NUM_CHAR];
  
  unsigned char pwlow,pwhigh;
  pwlow  = (unsigned char) pos;
  pwhigh = (unsigned char) (pos>>8);
  //printf("%.2x %.2x\n",pwlow,pwhigh);

  //pwlow  = 0x00;
  //pwhigh = 0x03;
  sprintf(cmd, "%c%c%c%c%c%c%c%c",0x21,0x53,0x43,ch,ra,pwlow,pwhigh,0xd);
    
  //printf("len %d\n ",strlen(cmd));
  /*for(k=0; k<  strlen(cmd); k++){
    printf("%.2x ",cmd[k]);
    }*/
  servoSendReceiveCommand(dev->fd, cmd,8, &answer,0 );
  //printf("%s\n",answer);
  
  return 1;
}

