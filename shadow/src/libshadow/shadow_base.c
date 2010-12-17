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

#include "shadow_base.h"

void shadowInitializeDevice(shadow_spcu_device_p p)
{
  strncpy(p->ttyport, "/dev/ttyUSB0", MAX_NAME_LENGTH);
  p->baud = 19200;//115200; //19200;
  // p->parity = N;
  p->fd = -1;
  p->databits = 8;
  p->stopbits = 1;
  p->hwf = 0; // Check this?? 1
  p->swf = 0;
}

shadow_spcu_p shadowInitialize()
{

  shadow_spcu_p shadow_spcu;
  shadow_spcu = (shadow_spcu_p)calloc(1, sizeof(shadow_spcu_t));
  shadowInitializeDevice(&(shadow_spcu->dev));
  return shadow_spcu;

}


void shadowClear(shadow_spcu_p shadow)
{
  free(shadow);
}

void shadowPrintParams(shadow_spcu_params_p params)
{
  int i;

  printf("TimeStamp[0] %d\n", params->TimeStamp[0]);  
  printf("Sensors:          ");
  for(i=0;i<8;i++)
    printf("[%d]:%4d   ", i, params->Sensors[i]);
  printf("\n");
  printf("Targets:          ");
  for(i=0;i<8;i++)
    printf("[%d]:%4d   ", i, params->Targets[i]);
  printf("\n");
  printf("Valves state set: ");
  for(i=0;i<8;i++)
    printf("[%d]:%4d   ", i, params->ValveStates[i]);
  printf("\n");
  printf("Set valves state          = %#1x\n", params->SetValveStates);
  printf("Actual valves state       = %#1x\n", params->ActualValveStates);
  printf("LATA register on MCU      = %#1x\n", params->LATAreg);
  printf("LATB register on MCU      = %#1x\n", params->LATBreg);
  printf("Forced States for values  = %#1x\n", params->ForceStates);
  printf("Controller\tSensor\tTarget\tP\tI\tD\n");
  for(i=0;i<8;i++){
    printf("[%d] =      \t%d\t%d\t%d\t%d\t%d\n", i, params->Controller_Sensor[i],
	   params->Controller_Target[i],params->Controller_P[i],
	   params->Controller_I[i], params->Controller_D[i]);
  }
  printf("\n");
  printf("TimeStamp[1] %d\n ", params->TimeStamp[1]);

}
