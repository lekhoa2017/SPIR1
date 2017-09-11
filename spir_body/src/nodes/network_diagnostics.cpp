/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>
#include <iostream>
#include <stdio.h>


void console_pinger(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  FILE * f = popen( "ping -c 1 10.42.0.1 2>&1", "r" );
    if ( f == 0 ) {
        fprintf( stderr, "Could not execute\n" );        
    }    
    int exit_code = pclose(f); //exit_code = 0 if OK ping, exit_code = 256 if BAD ping
    //printf("\n This is the exit code %d \n", exit_code);
  if (exit_code == 0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "10.42.0.1 Connected");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "10.42.0.1 Disconnected");
}

void payload_pinger(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  FILE * f = popen( "ping -c 1 10.42.0.20 2>&1", "r" );
    if ( f == 0 ) {
        fprintf( stderr, "Could not execute\n" );        
    }    
    int exit_code = pclose(f); //exit_code = 0 if OK ping, exit_code = 256 if BAD ping
    //printf("\n This is the exit code %d \n", exit_code);
  if (exit_code == 0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "10.42.0.20 Connected");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "10.42.0.20 Disconnected");
}

void lowVoltage_pinger(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  FILE * f = popen( "ping -c 1 10.42.0.30 2>&1", "r" );
    if ( f == 0 ) {
        fprintf( stderr, "Could not execute\n" );        
    }    
    int exit_code = pclose(f); //exit_code = 0 if OK ping, exit_code = 256 if BAD ping
    //printf("\n This is the exit code %d \n", exit_code);
  if (exit_code == 0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "10.42.0.30 Connected");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "10.42.0.30 Disconnected");
}

void laserRangeFinder_pinger(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  FILE * f = popen( "ping -c 1 10.42.0.50 2>&1", "r" );
    if ( f == 0 ) {
        fprintf( stderr, "Could not execute\n" );        
    }    
    int exit_code = pclose(f); //exit_code = 0 if OK ping, exit_code = 256 if BAD ping
    //printf("\n This is the exit code %d \n", exit_code);
  if (exit_code == 0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "10.42.0.50 Connected");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "10.42.0.50 Disconnected");
}

void teleopCamera_pinger(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  FILE * f = popen( "ping -c 1 10.42.0.60 2>&1", "r" );
    if ( f == 0 ) {
        fprintf( stderr, "Could not execute\n" );        
    }    
    int exit_code = pclose(f); //exit_code = 0 if OK ping, exit_code = 256 if BAD ping
    //printf("\n This is the exit code %d \n", exit_code);
  if (exit_code == 0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "10.42.0.60 Connected");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "10.42.0.60 Disconnected");
}

void stereoCamera_pinger(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  FILE * f = popen( "ping -c 1 10.42.0.70 2>&1", "r" );
    if ( f == 0 ) {
        fprintf( stderr, "Could not execute\n" );        
    }    
    int exit_code = pclose(f); //exit_code = 0 if OK ping, exit_code = 256 if BAD ping
    //printf("\n This is the exit code %d \n", exit_code);
  if (exit_code == 0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "10.42.0.70 Connected");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "10.42.0.70 Disconnected");
}
void thruster_pinger(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  FILE * f = popen( "ping -c 1 10.42.0.177 2>&1", "r" );
    if ( f == 0 ) {
        fprintf( stderr, "Could not execute\n" );        
    }    
    int exit_code = pclose(f); //exit_code = 0 if OK ping, exit_code = 256 if BAD ping
    //printf("\n This is the exit code %d \n", exit_code);
  if (exit_code == 0)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "10.42.0.177 Connected");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "10.42.0.177 Disconnected");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diagnostic_updater");  
  ros::NodeHandle nh;
  diagnostic_updater::Updater updater;   
  updater.setHardwareID("none"); 
  updater.add("Console", console_pinger);
  updater.add("Payload", payload_pinger); 
  updater.add("Low Voltage", lowVoltage_pinger);
  //updater.add("Laser enclosure", laserRangeFinder_pinger);
  updater.add("Teleop enclosure", teleopCamera_pinger);
  updater.add("Thruster arduino", thruster_pinger);
  //updater.add("Stereo Camera enclosure", stereoCamera_pinger);  
  while (nh.ok())
  {    
    ros::Duration(0.01).sleep();    
    updater.update();
  }
  return 0; 
}
