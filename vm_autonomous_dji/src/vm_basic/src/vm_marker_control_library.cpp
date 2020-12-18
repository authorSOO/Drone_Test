#ifndef __MARKER_CONTROL__
#define __MARKER_CONTROL__

#include "../include/vm_basic_command.h"
#include "vm_basic_command.cpp"


void Vm_Basic_Command::Ar_Marker_Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg){
   Marker_Detect_flag=true;
     double roll,pitch,yaw;
     for(auto p : msg->markers){
     marker_x = -p.pose.pose.position.z;
     marker_y = -p.pose.pose.position.y;
     marker_z = -p.pose.pose.position.x;

      tf::Quaternion q(
      p.pose.pose.orientation.x,
      p.pose.pose.orientation.y,
      p.pose.pose.orientation.z,
      p.pose.pose.orientation.w
      );
      tf::Matrix3x3 m(q);
      
      m.getRPY(roll,pitch,yaw);
      
      
     
   if(pitch<0)pitch = 2.0*M_PI + pitch;
   marker_th=pitch;
   marker_th_deg=RAD_TO_DEG(pitch);
   #if MARKER_CHECK
      ROS_INFO("****MARKER POSE*********");
     // ROS_INFO("roll:%f , pitch :%f ,yaw :%f ",roll,pitch,yaw);
      ROS_INFO("x:%f, y:%f, z:%f, yaw:%lf ",marker_x,marker_y,marker_z,RAD_TO_DEG(pitch));
    //  ROS_INFO("x:%f, y:%f, z:%f, yas:%f",x,y,z,th_deg);
  #endif
     
  }
}


#endif