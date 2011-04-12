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

#ifndef FINGER_PUSHING_CLASS_H_
#define FINGER_PUSHING_CLASS_H_

// services
#include <shadow/SetController.h>
#include <shadow/GetStatus.h>
#include <shadow/GetSensors.h>
#include <shadow/SetTargets.h>
#include <shadow/DisableController.h>
#include <shadow/PulseValves.h>
#include <shadow/StartPublishing.h>
#include <shadow/SetValves.h>
#include <shadow/ShadowSensors.h>

class FingerPushing
{
private:
  // Data types to communicate with the Shadow node
  shadow::StartPublishing spcu_publishing_state_;
  shadow::PulseValves spcu_valve_pulse_;
  shadow::SetValves spcu_valve_states_;
  shadow::SetTargets spcu_target_values_;
  shadow::SetController spcu_controller_values_;
  shadow::DisableController spcu_controller_disable_;

  // Services for sending commands to the SPCU
  ros::ServiceClient spcu_client_pulse_valves_;
  ros::ServiceClient spcu_client_targets_;
  ros::ServiceClient spcu_client_publisher_;
  ros::ServiceClient spcu_client_valves_;
  ros::ServiceClient spcu_client_controller_;
  ros::ServiceClient spcu_disable_controller_;
  
  // Subscriber to subscribe to sensordata
  ros::Subscriber spcu_shadow_sensor_sub_;

  ros::Rate loop_rate_;

public:
  
  ros::NodeHandle nh_;

  FingerPushing();
  ~FingerPushing(void);
  bool init();
  bool fill_pam();
  bool empty_pam();
  bool wait_button_push(int number, double delay);
  double record_sensor_data(int sensor,int number_of_recordings);
  int record_max_sensor_data_time(int sensor,float duration);
  bool set_controller(int controller, int sensor, int p, int i, int d);
  bool set_target(int controller, int target, int tol);
  bool disable_controller(int controller);
  void interprete_case(unsigned int test_case, int& type, double& scale);
  bool measure_baseline(int sensor,int current_baseline, double tol);

};

#endif
