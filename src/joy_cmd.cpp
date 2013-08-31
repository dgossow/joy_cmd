/*********************************************************************
*
*  Copyright (c) 2013, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/


#include "joy_cmd.h"

JoyCmd::JoyCmd( ros::NodeHandle pnh ) :
  trigger_pressed_(false)
{
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>( "joy", 1, boost::bind(&JoyCmd::joyCb, this, _1) );

  pnh.param<int>( "trigger_button", trigger_button_, 11 );
  pnh.param<std::string>( "on_command", on_command_, "" );
  pnh.param<std::string>( "off_command", off_command_, "" );

  ROS_INFO_STREAM("Activate command with joystick button " << trigger_button_);
}

JoyCmd::~JoyCmd()
{
}

void JoyCmd::joyCb( sensor_msgs::JoyConstPtr joy_msg )
{
  if ( (unsigned)trigger_button_ >= joy_msg->buttons.size() )
  {
    ROS_ERROR_ONCE("Button index for projector trigger is out of bounds!");
    return;
  }

  bool trigger_pressed = joy_msg->buttons.at(trigger_button_);

  if ( trigger_pressed != trigger_pressed_ )
  {
    trigger_pressed_ = trigger_pressed;
    std::string sys_command = trigger_pressed ? on_command_ : off_command_;
    int exit_code = system( sys_command.c_str() );
    if ( exit_code != 0 )
    {
      ROS_ERROR_STREAM( "Call to '" << sys_command << "' exited with code " << exit_code );
    }
  }

}
