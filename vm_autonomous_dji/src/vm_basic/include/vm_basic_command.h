#ifndef __COMMAND_H__
#define __COMMAND_H__

#include "vm_pid_control_library.h"
#include "../src/vm_pid_control_library.cpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <stdio.h>
#include <unistd.h>
#include <termio.h>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <utility>

#include <map>

// be class
#define ZERO_INT 0
#define ZERO_FLOAT 0.0
#define TAKEOFF_SPEED  2
#define Z_SPEED 1

#define MARKER_CHECK 1
#define ROTATION_CHECK 0
#define ROTATION_POSE_CHECK 0
#define POSE_CHECK 0
#define POSE_ROTATION_CHECK 1
#define MAP_CHECK 1
#define ALTITUDE_CHECK 0
#define TAKEOFF_CHECK 1
#define ODOM_CHECK 0
#define COMPLEDTE_ODOM_CHECK 1
#define LINEAR_TRANS_CHECK 1
#define GOTO_PARAM_CHECK 1
#define PID_CHECK 0

#define  RAD_TO_DEG(RAD)   (RAD)*(180.0)/(M_PI)
#define  DEG_TO_RAD(DEG)   (DEG)*(M_PI)/(180.0)
#define  RAD_360        (2*M_PI)

#define SAT_RANGE_X      1.5
#define SAT_RANGE_Y      1.5
#define SAT_RANGE_Z      1.5
#define SAT_RANGE_YAW    DEG_TO_RAD(10)

#define SAT_LANDING_HEIGHT    0.5

enum _Info_pose{
   _INFO_X,
   _INFO_Y,
   _INFO_Z,
   _INFO_X_GOAL,
   _INFO_Y_GOAL,
   _INFO_Z_GOAL,
   _INFO_TH_RAD,
   _INFO_TH_GOAL,
   _INFO_TH_DEG,
   _INFO_SAT_RANGE_W,
   _INFO_SAT_RANGE_X,
   _INFO_SAT_RANGE_Y,
   _INFO_SAT_RANGE_Z,
   _INFO_SPEED,
   _INFO_TURN,
   _INFO_X_VEL,
   _INFO_Y_VEL,
   _INFO_Z_VEL,
   _INFO_TH_VEL,
   _INFO_SPEED_Z
};

const std::string error_msg = R"(
   Command is not correct!!!!
)";

const std::string msg = R"(
   Reading from the keyboard and Publishing to Twist!
   ---------------------------
   COMMAND SET  [ unit = m, degree ]
   ---------------------------
   takeoff <z>
   goto  <x> <y> <z> <deg>
   setspeed <speed> <w>
   landing (In circumstances of detecting marker)

   ..........................
   Moving around:
      u    i    o
      j    k    l
      m    ,    .

   For Holonomic mode (strafing), hold down the shift key:
   ---------------------------
      U    I    O
      J    K    L
      M    <    >

   t : up (+z)
   b : down (-z)

   anything else : stop

   q/z : increase/decrease max speeds by 10%
   w/x : increase/decrease only linear speed by 10%
   e/c : increase/decrease only angular speed by 10%

   CTRL-C to quit

   )";

class Vm_Basic_Command
{
private:
   /* data */
	ros::NodeHandle nh;
   ros::Subscriber gps_sub,keyboard_sub,gps_odom_sub,imu_odom_sub,ar_marker_sub;
   
   

public:
   Vm_Basic_Command(/* args */);
   ~Vm_Basic_Command();
   //float Altitude_Dest = 0.0;
   
    #if MAP_CHECK
      // std::map<std::string, std::vector<float>> moveBindings{
      // }
      std::map<int, float> info_pose
      {
      {_INFO_Y, x},
      {_INFO_Y, y},
      {_INFO_Z, z},
      {_INFO_X_GOAL, x_goal},
      {_INFO_Y_GOAL, y_goal},
      {_INFO_Z_GOAL, z_goal},
      {_INFO_TH_RAD, th_rad},
      {_INFO_TH_GOAL, th_goal},
      {_INFO_TH_DEG, th_deg},
      {_INFO_SAT_RANGE_W, SAT_RANGE_YAW},
      {_INFO_SAT_RANGE_X,_INFO_SAT_RANGE_X},
      {_INFO_SAT_RANGE_Y,_INFO_SAT_RANGE_Y},
      {_INFO_SAT_RANGE_Z,_INFO_SAT_RANGE_Z},
      {_INFO_SPEED,speed},
      {_INFO_TURN,turn},
      {_INFO_X_VEL,x_vel},
      {_INFO_Y_VEL,y_vel},
      {_INFO_Z_VEL,z_vel},
      {_INFO_TH_VEL,th_vel},
      {_INFO_SPEED_Z,Z_SPEED}
      };
   #endif
   float x_vel=0,y_vel=0,z_vel =0,th_vel=0;
   float marker_x=0,marker_y=0,marker_z=0,marker_th=0,marker_th_deg=0;

   float x = 0,y=0,z=0,th_rad=0 ,th_deg=0;  
   float x_goal = 0,y_goal=0,z_goal=0, th_goal=0;
   float x_abs = 0, y_abs=0, z_abs=0, th_abs=0; 
   float fabs_x=0,fabs_y=0,fabs_z=0,fabs_th=0;

   float Altitude = 0.0;
   float speed = 0.8;
   float turn = 0.2;
   float marker_speed = 1.0;
   float marker_landing_speed=1.0;

   double pid_dt=0.1, pid_max=1.0, pid_min=0.0, pid_Kp=0.1, pid_Kd=0.01, pid_Ki=0.01 ;
   double pid_z_dt=0.1, pid_z_max=1.0, pid_z_min=0.0, pid_z_Kp=0.1, pid_z_Kd=0.01, pid_z_Ki=0.01 ;
   double pid_th_dt=0.1,pid_th_max=0.5,pid_th_min=0.0,pid_th_Kp=0.1,pid_th_Kd=0.01,pid_th_Ki=0.01;
   bool Mission_flag = false,Odom_Start_flag= false ,Imu_Start_flag=false,Marker_Detect_flag=false;
   geometry_msgs::Twist twist;
   std::string key;
   ros::Publisher cmd_vel_pub;
   std::string command_line(void);
   PID linear_speed_pid;
   PID th_w_pid;
   PID z_speed_pid;
    PID takeoff_pid;    //new change using pointer
   int Take_Off(void);
   int Landing(void);
   int Move_By_Odom(void);
   void Gps_sensor_Callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
   void Keyboard_command_Callback(const std_msgs::String::ConstPtr &msg);
   void Gps_To_Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg);
   void Linear_Flight_Algorithm(std::map<int, float> &pose_info);
   void Imu_To_Odom_Callback(const sensor_msgs::Imu::ConstPtr &msg);
   void Ar_Marker_Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
   
   double Distance(const std::vector<std::pair<float,float>> &point);
   
};

Vm_Basic_Command::Vm_Basic_Command(/* args */)
{
   cmd_vel_pub = nh.advertise< geometry_msgs::Twist>("cmd_vel", 1);
   gps_sub = nh.subscribe("fix", 1, &Vm_Basic_Command::Gps_sensor_Callback,this);
   keyboard_sub = nh.subscribe("keyboard_command",1, &Vm_Basic_Command::Keyboard_command_Callback,this);
   gps_odom_sub = nh.subscribe("vm_odom",1,&Vm_Basic_Command::Gps_To_Odom_Callback,this);
   imu_odom_sub = nh.subscribe("raw_imu",1,&Vm_Basic_Command::Imu_To_Odom_Callback,this);
   ar_marker_sub = nh.subscribe("ar_pose_marker",1,&Vm_Basic_Command::Ar_Marker_Callback,this);
   //linear_speed_pid.pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
   //th_w_pid(pid_th_dt,pid_th_max,pid_th_min,pid_th_Kp,pid_th_Kd,pid_th_Ki);
  
}

Vm_Basic_Command::~Vm_Basic_Command()
{
   ROS_INFO("Basic_command Node close");
}






#endif