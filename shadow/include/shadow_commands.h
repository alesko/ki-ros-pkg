/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010,Alexander Skoglund
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

#ifndef SHADOW_COMMANDS
#define SHADOW_COMMANDS

#include "shadow_base.h"

#ifdef __cplusplus
extern "C" {
#endif
  void shadowHexPulseValves(shadow_spcu_device_p dev, int time_ms[NUM_VALVES]);
  void shadowHexSetValves(shadow_spcu_device_p dev, int set_bin[NUM_VALVES]);
  void shadowHexReadSensors(shadow_spcu_device_p dev, unsigned short sensors[NUM_VALVES]);
  int  shadowHexSetController(shadow_spcu_device_p dev, unsigned short valve, 
			   unsigned short sensor, char target, char P, char I, char D);
  void shadowHexDisableController(shadow_spcu_device_p dev, int controller);
  void shadowHexSetTargets(shadow_spcu_device_p dev, int set_target[NUM_VALVES]);
  int  shadowHexStatus(shadow_spcu_p shadow_unit);
  void shadowHexSetByMask(shadow_spcu_device_p dev,enum  MASK_ACTION actions[4] );

  void shadowAsciiSetTargets(shadow_spcu_device_p dev, int set_target[NUM_VALVES]);
  int shadowAsciiSetController(shadow_spcu_device_p dev, unsigned short valve, 
			       unsigned short sensor, char target, char P, char I, char D);

#ifdef __cplusplus
}
#endif

#endif
