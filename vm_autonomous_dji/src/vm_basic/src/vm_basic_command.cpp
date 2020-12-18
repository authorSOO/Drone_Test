#ifndef __COMMAND__
#define __COMMAND__

#include "../include/vm_basic_command.h"
#include "vm_linear_flight_library.cpp"
#include "vm_marker_control_library.cpp"
#include "vm_pid_control_library.cpp"
#include "../include/vm_pid_control_library.h"

void Vm_Basic_Command::Imu_To_Odom_Callback(const sensor_msgs::Imu::ConstPtr &msg){
    tf::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w
   );
   tf::Matrix3x3 m(q);
   double roll,pitch,yaw;
   m.getRPY(roll,pitch,yaw);
   if(yaw<0)yaw = RAD_360 + yaw;
  
   if(!Imu_Start_flag){
      th_abs = yaw;
   }
   else{
      th_rad = yaw-th_abs;
      if(th_rad<0)th_rad=RAD_360+th_rad;
      th_deg=RAD_TO_DEG(th_rad);
      fabs_th=fabs(th_rad-th_goal);
   }  
   Imu_Start_flag=true;
}


int Vm_Basic_Command::Move_By_Odom(void){
   ros::Rate Move_rate(5);
   linear_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
   th_w_pid.PID_set(pid_th_dt,pid_th_max,pid_th_min,pid_th_Kp,pid_th_Kd,pid_th_Ki);
   z_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
   
   while(ros::ok()){
      info_pose[_INFO_X_GOAL]=x_goal;
      info_pose[_INFO_Y_GOAL]=y_goal;
      info_pose[_INFO_Z_GOAL]=z_goal;
      info_pose[_INFO_TH_GOAL]=th_goal;
      info_pose[_INFO_X]=x;
      info_pose[_INFO_Y]=y;
      info_pose[_INFO_Z]=z;
      info_pose[_INFO_TH_RAD]=th_rad;
      info_pose[_INFO_TH_DEG]=th_deg;
      info_pose[_INFO_SAT_RANGE_W]=SAT_RANGE_YAW;
     info_pose[_INFO_SPEED]=speed;
     info_pose[_INFO_SPEED_Z]=Z_SPEED;
     info_pose[_INFO_TURN]=turn;
      info_pose[_INFO_SAT_RANGE_Z] = SAT_RANGE_Z;
      
      Linear_Flight_Algorithm(info_pose);
      // if(x_vel<info_pose[_INFO_X_VEL])x_vel++;
      // else x_vel=info_pose[_INFO_X_VEL];
      // if(y_vel<info_pose[_INFO_X_VEL])y_vel++;
      // else y_vel=info_pose[_INFO_X_VEL];
      twist.linear.x = info_pose[_INFO_X_VEL];
      twist.linear.y = info_pose[_INFO_Y_VEL];
      twist.linear.z = info_pose[_INFO_Z_VEL];
     
      twist.angular.x = ZERO_FLOAT;
      twist.angular.y = ZERO_FLOAT;
      twist.angular.z = info_pose[_INFO_TH_VEL];
     
      fabs_x = fabs(x-x_goal); 
      fabs_y = fabs(y-y_goal);
      fabs_z = fabs(z-z_goal);

      #if ROTATION_POSE_CHECK
         if(fabs(th-th_goal)<0.5)break;
      #endif
      #if POSE_CHECK
          if(fabs(x-x_goal)<1.5 && fabs_y<1.5)break;
      #endif
      #if POSE_ROTATION_CHECK
        
         if(fabs_x<SAT_RANGE_X && fabs_y<SAT_RANGE_Y && fabs_z<SAT_RANGE_Z ){
             ROS_INFO("Meet The Condition");
             //ROS_INFO("fabs_x : %f ",fabs_x);
            break;
         }
         //&& fabs_th<SAT_RANGE_YAW
      #endif

      #if COMPLEDTE_ODOM_CHECK
         ROS_INFO("odom - x: %f y:%f z:%f th_deg:%f",x,y,z,th_deg);
      #endif
      cmd_vel_pub.publish(twist);
      ros::spinOnce();
      Move_rate.sleep();  
   }
   
}



int Vm_Basic_Command::Landing(void){
   //
   linear_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
   th_w_pid.PID_set(pid_th_dt,pid_th_max,pid_th_min,pid_th_Kp,pid_th_Kd,pid_th_Ki);
   z_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
  if(Marker_Detect_flag){
     ros::Rate landing_rate(5);
     while(ros::ok()){
      ROS_INFO("landing");
      info_pose[_INFO_X_GOAL]=ZERO_FLOAT;
      info_pose[_INFO_Y_GOAL]=ZERO_FLOAT;
      info_pose[_INFO_Z_GOAL]=ZERO_FLOAT;
      info_pose[_INFO_TH_GOAL]=ZERO_FLOAT;
      info_pose[_INFO_X]=marker_x;
      info_pose[_INFO_Y]=marker_y;
      info_pose[_INFO_Z]=marker_z;
      info_pose[_INFO_TH_RAD]=marker_th;
      info_pose[_INFO_TH_DEG]=marker_th_deg;
      info_pose[_INFO_SAT_RANGE_W]=SAT_RANGE_YAW;
      info_pose[_INFO_SPEED]=speed;
      info_pose[_INFO_SPEED_Z]=Z_SPEED;
      info_pose[_INFO_TURN]=turn;
      info_pose[_INFO_SAT_RANGE_Z] = SAT_RANGE_Z;
      Linear_Flight_Algorithm(info_pose);

      // if(marker_th<M_PI && marker_th > 0.1) th_vel = -0.5;
      // else if(marker_th>M_PI &&marker_th< 6.0) th_vel = 0.5;
      // else th_vel=ZERO_FLOAT;

      twist.linear.x = info_pose[_INFO_X_VEL];
      twist.linear.y = info_pose[_INFO_Y_VEL];
      twist.linear.z = info_pose[_INFO_Z_VEL];
     
      twist.angular.x = ZERO_FLOAT;
      twist.angular.y = ZERO_FLOAT;
      twist.angular.z = info_pose[_INFO_TH_VEL];

 
      if(z<=SAT_LANDING_HEIGHT)break;
      //ROS_INFO("LANDING");
      cmd_vel_pub.publish(twist);
      ros::spinOnce();
      landing_rate.sleep();  
      }
   }
}


void Vm_Basic_Command::Gps_sensor_Callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
 
   Altitude = msg->altitude;
   #if ALTITUDE_CHECK
      ROS_INFO("Altitude : %f",Altitude);
   #endif
}

void Vm_Basic_Command::Gps_To_Odom_Callback(const nav_msgs::Odometry::ConstPtr &msg){   
  
   if(!Odom_Start_flag){
      y_abs = (-1)*msg->pose.pose.position.x;
      x_abs = msg->pose.pose.position.y;
      z_abs = msg->pose.pose.position.z;
   }
   else{
      y = (-1)*msg->pose.pose.position.x - y_abs ;
      x = msg->pose.pose.position.y - x_abs;
      z = msg->pose.pose.position.z -z_abs;
     
      
//      fabs_th =

      #if ODOM_CHECK
         ROS_INFO("x : %f , y : %f , z : %f",x,y,z);
      #endif
   }

   Odom_Start_flag=1;
}
//filter gps odom & imu & encoding
int Vm_Basic_Command::Take_Off(void){
   z_speed_pid.PID_set(pid_dt,pid_max,pid_min,pid_Kp,pid_Kd,pid_Ki);
   twist.linear.x = ZERO_FLOAT;
   twist.linear.y = ZERO_FLOAT;
   twist.linear.z = TAKEOFF_SPEED;

   twist.angular.x = ZERO_FLOAT;
   twist.angular.y = ZERO_FLOAT;
   twist.angular.z = ZERO_FLOAT;
  // ROS_INFO("%f , %f",Altitude,Altitude_Dest);
   double takeoff_speed=ZERO_FLOAT;
    double diff_pose_z;
   ros::Rate takeoff_rate(5);
   double z_pid;
   while(ros::ok()){
      
      if(z>=z_goal)break;
     
      diff_pose_z = z_goal-z;
      z_pid=z_speed_pid.calculate(ZERO_FLOAT,diff_pose_z);
      if(takeoff_speed<z_pid)takeoff_speed+=0.05;
      else takeoff_speed=z_pid;

      // if(diff_pose_z>pose_info[_INFO_SAT_RANGE_Z])pose_info[_INFO_Z_VEL]=pose_info[_INFO_SPEED_Z];
      // else if(diff_pose_z<(-1*pose_info[_INFO_SAT_RANGE_Z]))pose_info[_INFO_Z_VEL]=(-1*pose_info[_INFO_SPEED_Z]);
      // else pose_info[_INFO_Z_VEL]=ZERO_FLOAT;
      twist.linear.z=takeoff_speed;
      cmd_vel_pub.publish(twist);
      ros::spinOnce();
      takeoff_rate.sleep();  
       #if TAKEOFF_CHECK
         ROS_INFO("takeoff");
         ROS_INFO("v : %f,pid : %f,diff :%f",takeoff_speed,z_pid,diff_pose_z);
         //ROS_INFO("%f %f",Altitude,Altitude_Dest);
      #endif
   }
}

void Vm_Basic_Command::Keyboard_command_Callback(const std_msgs::String::ConstPtr &msg){
     std::istringstream iss(msg->data);
     std::string token;
     std::vector<std::string> param;
     Mission_flag = true;
      while(getline(iss,token,' ')){
         param.emplace_back(token);
      }
     
      if(param[0]=="takeoff"){
         z_goal = stol(param[1]);  //altitude
         Take_Off();
      }
         
      else if(param[0]=="landing"){
         Landing();
      }
      else if(param[0]=="goto"){
         x_goal = stol(param[1]);   //x-position
         y_goal = stol(param[2]);   //y-position
         z_goal = stol(param[3]);   //z-position
         float tmp_th_goal = stol(param[4]);
         th_goal = DEG_TO_RAD(tmp_th_goal);
         #if GOTO_PARAM_CHECK
            ROS_INFO("%f, %f, %f, %f,",x_goal,y_goal,z_goal,tmp_th_goal);
         #endif
         Move_By_Odom();
      }
      else if(param[0]=="set"){
         speed = stol(param[1]);    //linear-spdiff_theed
         turn = stol(param[2]);     //rad-speed
      }
      else{
         std::cout<<error_msg<<std::endl;
      }
      Mission_flag = false;

}

int main(int argc, char *argv[])
{
	//init ROS node
	ros::init(argc,argv,"Vm_Basic_Command");
   Vm_Basic_Command vm_basic_cmd;
   //PID linear_speed_pid;
   
   ros::Rate rate(10);
   //PID
   while (ros::ok())
   { 
      if(!vm_basic_cmd.Mission_flag){
         vm_basic_cmd.twist.linear.x = ZERO_FLOAT;
         vm_basic_cmd.twist.linear.y = ZERO_FLOAT;
         vm_basic_cmd.twist.linear.z = ZERO_FLOAT;

         vm_basic_cmd.twist.angular.x = ZERO_FLOAT;
         vm_basic_cmd.twist.angular.y = ZERO_FLOAT;
         vm_basic_cmd.twist.angular.z = ZERO_FLOAT;    
      // }     
       
         ros::spinOnce();
         rate.sleep();
         vm_basic_cmd.cmd_vel_pub.publish(vm_basic_cmd.twist);
         
         ROS_INFO("Hovering : %f %f %f %f",vm_basic_cmd.x,vm_basic_cmd.y,vm_basic_cmd.z,vm_basic_cmd.th_deg);
     }
   }

	
	return 0;
}


#endif
