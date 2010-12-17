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

//#include "shadow_base.h"
#include "shadow_io.h"
#include "shadow_commands.h"

/**********************************************************/
/* SHADOW ROBOTICS SPCU COMMANDS                          */
/**********************************************************/

void shadowHexPulseValves(shadow_spcu_device_p dev, int time_ms[NUM_VALVES])
{
  unsigned char cmd[MAX_NUM_CHAR];
  int i,j,k;
  cmd[0] = H_PulseValves;
  j=1;
  for(i=0; i < NUM_VALVES; i++){
    if(time_ms[i]>= MIN_PULSE_VALVE)
      {
	cmd[j] = i;
	cmd[j+1] = time_ms[i];
	j=j+2;
      }
    //shadowSendCommand(&shadow->dev, command, 6);  
  }
  cmd[j]=255; // Terminate command
  j++;

  
  // Debugging
  /*printf("Command ");
  for(k=0; k<j; k++){
    printf("%d ",cmd[k]);
  }
  printf("\n");
  printf("fd = %d\n",dev->fd);*/
  //--------------------------------

  //shadowSendCommand(dev->fd, cmd, j); 
  shadowSendCommandEcho(dev->fd, cmd, j); 


}

void shadowHexSetValves(shadow_spcu_device_p dev, int set_bin[NUM_VALVES])
{
  unsigned char cmd[MAX_NUM_CHAR];
  int i,j,k;
  cmd[0] = H_SetValves;
  j=1;
  for(i=0; i < NUM_VALVES; i++){
    if( set_bin[i]==1 || set_bin[i]==0 )
      {
	cmd[j] = i;
	cmd[j+1] = set_bin[i];
	j=j+2;
      }
    //shadowSendCommand(&shadow->dev, command, 6);  
  }
  cmd[j]=255;
  j++;
  /*printf("Command ");
  for(k=0; k<j; k++){
    printf("%d ",cmd[k]);
  }
  printf("\n");*/

  //shadowSendCommand(dev->fd, cmd, j);
  shadowSendCommandEcho(dev->fd, cmd, j);
}

void shadowHexReadSensors(shadow_spcu_device_p dev, unsigned short sensors[NUM_VALVES])
{
  unsigned char cmd[1];
  unsigned char inbuf[MAX_NUM_CHAR];
  int i,j;//,k;
  int len;
  //  unsigned short temp;

  cmd[0] = H_ReadSensors;
  //printf("In shadowHexReadSensors\n");
  
  shadowSendCommand(dev->fd, cmd, 1);
  len = shadowGetAnswer(dev->fd,inbuf,18,1);

  //
  j=0;
  for(i=1;i<len-1;i=i+2){
    sensors[j] = ((inbuf[i]<<8) + inbuf[i+1]) >> 4;
    //printf("sensors[%d]=%x\t",j,sensors[j] );
    j++;
  }
  //printf("\n");

}

// Set the paramters for controller
// valve  = the valve number 0..7
// sensor = the sensor number 0..7
// target = a target number 0..7, (optiaonal, -1 if not in use)
// if target is set, the sensor with the target number operates as target
// P, I, D = control gains
int shadowHexSetController(shadow_spcu_device_p dev, unsigned short valve, 
			   unsigned short sensor, char target, char P, char I, char D)
{
  unsigned char cmd[MAX_NUM_CHAR];
  int i,j,k;
  char tar;
  cmd[0] = H_SetControllers;
  cmd[1] = 0x03 && valve;
  printf("Valvebyte: %x\n",cmd[1]);

  // Check if target should be used
  /*if ( (-1 < target) && (target < 8))
    {
      tar = target;
      printf("Set target: %x\n",tar);
    }
  else
    {
      printf("No target\n");
      tar = 0;
    }

  
  if ( (-1 < target) && (target < 8))
    {
      cmd[2] = (((tar << 1) | 0x01) << 3) | sensor; 
      //printf("shifted target: %x\n",tar);
      //tar = tar | 0x01; 
      //printf("ORed target: %x\n",tar);
      //tar = tar<<3;
      //printf("shfted  target: %x\n",tar);
      //cmd[2] = tar | sensor;
    }
  else*/
    cmd[2] = 0x03 && sensor; // No target

  //printf("cmd[2] %x\n", cmd[2]);

  cmd[3] = (unsigned char) ((short)P + 127);
  cmd[4] = (unsigned char) ((short)I + 127);
  cmd[5] = (unsigned char) ((short)D + 127);
  cmd[6] = 255;
  
  /*  printf("\nCommand ");
  for(k=0; k < 6; k++){
    printf("%x ",cmd[k]);
  }
  printf("\n");*/
  //printf("fd = %d\n",dev->fd);
  //shadowSendCommand(dev->fd, cmd, 6);
  shadowSendCommandEcho(dev->fd, cmd, 7);
  
  return 1;
}

// Set the paramters for controller
// valve  = the valve number 0..7
// sensor = the sensor number 0..7
// target = a target number 0..7, (optional, -1 if not in use)
// if target is set, the sensor with the target number operates as target
// P, I, D = control gains
int shadowAsciiSetController(shadow_spcu_device_p dev, unsigned short valve, 
			   unsigned short sensor, char target, char P, char I, char D)
{
  char cmd[MAX_NUM_CHAR];
  //char cmd;
  int i,j,k;
  char tar;
  //cmd[0] = H_SetControllers;
  //cmd[1] = 0x03 && valve;
  //printf("Valvebyte: %x\n",cmd[1]);

  printf("Target %d\n",target);
  // Check if target should be used
  if ( (-1 < target)  && (target < 8))
    {
      
      tar = target;
      sprintf(cmd,"c %d %d:%d %d %d %d\n", valve, sensor,target, P, I, D);
      printf("%s\n",cmd);
      shadowSendCommandEchoAscii(dev->fd, cmd,strlen(cmd));
      /*for(i=0; i < MAX_NUM_CHAR; i++)
	cmd[i]=NULL;
      usleep(1000);
      sprintf(cmd,"t %d %d\n", sensor, 800);
      printf("%s\n",cmd);
      shadowSendCommandEchoAscii(dev->fd, cmd,strlen(cmd));*/
      //printf("%s",cmd);
      //sprintf(cmd,"c %d %d %d %d %d %d\n", valve, sensor, P, I, D);
      //printf("%s"cmd);
    }
  else
    {
      printf("No target set for controller on valve %d\n",valve);
      sprintf(cmd,"c %d %d %d %d %d\n", valve, sensor, P, I, D);
      //printf("%s",cmd);
      //printf("len %d\n",  strlen(cmd));
      shadowSendCommandEchoAscii(dev->fd, cmd,strlen(cmd));  
      //printf("No target\n");
      //tar = 0;
    }


  return 1;
}


// Enter the a list with the contollers to disable,
// i.e., 1 0 0 1 1 0 1 0 will disable controllers 0, 4 and 6
// and omitt 46 since it is out of range 

void shadowHexDisableController(shadow_spcu_device_p dev, int controller)
{
  unsigned char cmd[MAX_NUM_CHAR];
  int i,k;
  cmd[0] = H_DisableControllers;

  printf("controller number %d ",controller );
  if ( controller >= 0 && controller<8)
    cmd[1] = controller;
  printf("controller number %d ",controller );
  cmd[2]=255; // Terminate command
  
  // Debugging
  /*printf("Command ");
  for(k=0; k<3; k++){
    printf("%.2x ",cmd[k]);
  }
  printf("\n");*/
  //printf("fd = %d\n",dev->fd);
  //--------------------------------

  //shadowSendCommand(dev->fd, cmd, 3);
  shadowSendCommandEcho(dev->fd, cmd, 3);  

}

void shadowHexSetTargets(shadow_spcu_device_p dev, int set_target[NUM_VALVES])
{
  unsigned char cmd[MAX_NUM_CHAR];
  unsigned char inbuf[MAX_NUM_CHAR];
  int i; //,j,k, len;


  cmd[0] = H_SetTargets;
  //j=1;
  for(i=0; i < NUM_VALVES; i++){
    if(set_target[i]>0)
      {
	cmd[1] = i; // Target number
	printf("set_target %x ",i);
	printf("val %d = %x ",set_target[i], set_target[i]);
	//cmd[2] = set_target[i] ;   //Hi bits
	//cmd[2] = ((set_target[i] >> 8) & 0xff)<<4;   //Hi bits
	cmd[2] = ((set_target[i] >> 4) & 0xff);   //Hi bits
	//cmd[2] = ((set_target[i] >> 8) & 0xff);   //Hi bits
	printf("hi bit %#2x ",cmd[2]);
	//cmd[3] = ( (set_target[i] & 0xff) << 4) ;     // low bits
	//cmd[3] = ( (set_target[i] & 0xff) << 0) ;     // low bits
	cmd[3] = ( (set_target[i] & 0x0f) << 4) ;     // low bits
	//cmd[3] = set_target[i] << 4;     // low bits
	printf("low bit %#2x \n",cmd[3]);
	cmd[4] = 255;
	shadowSendCommandEcho(dev->fd, cmd, 5);
      }    
  }

  /*cmd[0] = H_SetTargets;
  j=1;
  for(i=0; i < NUM_VALVES; i++){
    if(set_target[i]>=0)
      {
	cmd[j] = i;	
	//printf("set_target %x ",set_target[i]);
	cmd[j+1] = (set_target[i] >> 8) ;   //Hi bits
	//printf("cmd +1 %x ",cmd[j+1]);
	cmd[j+2] = (set_target[i] & 0xff) ;     // low bits
	//printf("cmd+2 %x ",cmd[j+2]);
	j=j+3;
      }
    //shadowSendCommand(&shadow->dev, command, 6);  
  }
  cmd[j]=255;
  shadowSendCommand(dev->fd, cmd, j+1); */

  // Seems that this command doesn't answer
  //int len =0;
  //len = shadowGetAnswer(dev->fd,inbuf,1,0);

  /*printf("len %d\n",len);
  if(len != 1)
     printf("ERROR in shadowHexSetTargets\n");
  for(i = 0; i < 1; i++){
    printf("inbuf[%d]; %.2x \n",i,inbuf[i]);
    }*/

}

void shadowAsciiSetTargets(shadow_spcu_device_p dev, int set_target[NUM_VALVES])
{
  unsigned char cmd[MAX_NUM_CHAR];
  unsigned char tmp[10];
  int i,j;//,k, len;

  // Rewrite to send all targets at once (to increase speed!)    
  sprintf(cmd,"t ");
  for(i=0; i < NUM_VALVES; i++){
    if(set_target[i]>0)
      {
	sprintf(tmp,"%d %x ", i, (set_target[i]<<4));
	//sprintf(cmd,"t %d %x\n", i, (set_target[i]<<4));
	//printf("%s\n",cmd);
	//shadowSendCommandEchoAscii(dev->fd, cmd,strlen(cmd));
	strcat(cmd,tmp);
	for(j=0; j < 10; j++)
	  tmp[j]=NULL;
      }
  }
  strcat(cmd,"\n");
  printf("%s",cmd);
  shadowSendCommandEchoAscii(dev->fd, cmd,strlen(cmd));

  /*cmd[0] = H_SetTargets;
  j=1;
  for(i=0; i < NUM_VALVES; i++){
    if(set_target[i]>=0)
      {
	cmd[j] = i;	
	//printf("set_target %x ",set_target[i]);
	cmd[j+1] = (set_target[i] >> 8) ;   //Hi bits
	//printf("cmd +1 %x ",cmd[j+1]);
	cmd[j+2] = (set_target[i] & 0xff) ;     // low bits
	//printf("cmd+2 %x ",cmd[j+2]);
	j=j+3;
      }
    //shadowSendCommand(&shadow->dev, command, 6);  
  }
  cmd[j]=255;
  shadowSendCommand(dev->fd, cmd, j+1); */

  // Seems that this command doesn't answer
  //int len =0;
  //len = shadowGetAnswer(dev->fd,inbuf,1,0);

  /*printf("len %d\n",len);
  if(len != 1)
     printf("ERROR in shadowHexSetTargets\n");
  for(i = 0; i < 1; i++){
    printf("inbuf[%d]; %.2x \n",i,inbuf[i]);
    }*/

}

int shadowHexStatus(shadow_spcu_p shadow_unit)
{
  unsigned char cmd[1];
  unsigned char inbuf[MAX_NUM_CHAR];
  int maxtry=10;
  int i,j;//k;
  int len;

  //printf("In shadowHexStatus\n");
  cmd[0] = H_PrintStatus;

  shadowSendCommand(shadow_unit->dev.fd, cmd, 1); 
  len = shadowGetAnswer(shadow_unit->dev.fd,inbuf,91,1); // 1 + 88 + 2 = 91
  printf("len %d\n",len);
  i = 1;
  while( (len != 91) & (i < maxtry))
    {
      printf("ERROR in shadowHexStatus\n");
      shadowSendCommand(shadow_unit->dev.fd, cmd, 1); 
      len = shadowGetAnswer(shadow_unit->dev.fd,inbuf,91,1); // 1 + 88 + 2 = 91
      printf("len %d\n",len);
      i++;
    }
  if(i == maxtry)
    return -1;

  for(i = 0; i < 91; i++)
    printf("Status [%d]: %.2x \n",i,inbuf[i]);

  shadow_unit->par.TimeStamp[0]=(inbuf[1]<<8)+ inbuf[2];
  
  for(i = 0; i < 8; i++){
    j=i*4+3;
    //printf("%d: ",j,j*4,inbuf[j*4], inbuf[j*4]);
    shadow_unit->par.Sensors[i]= (((((unsigned int)inbuf[j]) << 8) & 0xff00) | ((unsigned int)inbuf[j+1]))>>4;
    printf("Target reading[%d]: %.2x %.2x \n",i,inbuf[j+2],inbuf[j+3]);
    shadow_unit->par.Targets[i]= (((((unsigned int)inbuf[j+2]) << 8) & 0xff00) | ((unsigned int)inbuf[j+3]))>>4;
    //shadow_unit->par.Targets[i]= ((((unsigned int)inbuf[j+2]) << 8) + ((unsigned int)inbuf[j+3]));
    //   (((((unsigned int)inbuf[j]) << 8 ) & 0xff00))  | (((unsigned int)inbuf[j+1]) >> 4);
    //shadow_unit->par.Targets[i]=(inbuf[j+2]<<8) + (inbuf[j+3]) >> 4; // Is target equivalent??
    //printf("Sensor[%d] %d\n",i,shadow_unit->par.Sensors[i]);
  }


  for(i = 0; i < 8; i++){
    j=i+35;
    shadow_unit->par.ValveStates[i]=inbuf[j];
    //printf("ValveStates[%d] %d\n",i,shadow_unit->par.ValveStates[i]);
  }
  shadow_unit->par.SetValveStates    = inbuf[43];
  shadow_unit->par.ActualValveStates = inbuf[44];
  shadow_unit->par.LATAreg           = inbuf[45];
  shadow_unit->par.LATBreg           = inbuf[46];
  shadow_unit->par.ForceStates       = inbuf[47];
  //printf("SetValveStates %x\n",shadow_unit->par.SetValveStates);
  for(i = 0; i < 8; i++){
    j=i*5+48;
    shadow_unit->par.Controller_Sensor[i] =inbuf[j];
    shadow_unit->par.Controller_Target[i] =inbuf[j+1];
    shadow_unit->par.Controller_P[i]      =inbuf[j+2];
    shadow_unit->par.Controller_I[i]      =inbuf[j+3];
    shadow_unit->par.Controller_D[i]      =inbuf[j+4];
  }
  shadow_unit->par.TimeStamp[1]=(inbuf[88]<<8)+ inbuf[89];

  return 1;

}

void shadowHexSetByMask(shadow_spcu_device_p dev, enum MASK_ACTION actions[4] )
{
  unsigned char cmd[MAX_NUM_CHAR];
  int i,j,k;
  cmd[0] = H_SetByMask ;
  j=1;
  unsigned char bitmask = 0;

  for(i=0; i < 4;i++){
    if( HOLD == actions[i])
      bitmask = bitmask | 0x00;
    if( FILL == actions[i])
      bitmask = bitmask | 0x01;
    if( EMPTY == actions[i])
      bitmask = bitmask | 0x02;
    if( DONT_CARE == actions[i])
      bitmask = bitmask | 0x03;
    if( i < 3)
      bitmask = bitmask << 2;
  }
  printf("Bitmask %#x\n",bitmask);
  
  // Debugging
  /*printf("Command ");
  for(k=0; k<j; k++){
    printf("%d ",cmd[k]);
  }
  printf("\n");
  printf("fd = %d\n",dev->fd);*/
  //--------------------------------

  //shadowSendCommand(dev->fd, cmd, j);   

}
