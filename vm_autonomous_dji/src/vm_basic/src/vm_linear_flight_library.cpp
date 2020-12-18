#ifndef __LINEAR_FLY_LIB__
#define __LINEAR_FLY_LIB__

#include "../include/vm_basic_command.h"
#include "vm_basic_command.cpp"
#include "vm_pid_control_library.cpp"

double Vm_Basic_Command::Distance(const std::vector<std::pair<float,float>> &point){
   
  return sqrt(pow(point[0].first - point[1].first,2)+pow(point[0].second - point[1].second,2)); 
   
}

void Vm_Basic_Command::Linear_Flight_Algorithm(std::map<int, float> &pose_info){
   
   double transfer_yaw = pose_info[_INFO_TH_RAD];  
//    double transfer_yaw= RAD_360-th_rad;
   double transfer_pose_x = cos(transfer_yaw)*pose_info[_INFO_X] + sin(transfer_yaw)*pose_info[_INFO_Y];
   double transfer_pose_y = -sin(transfer_yaw)*pose_info[_INFO_X] + cos(transfer_yaw)*pose_info[_INFO_Y];
   
   //double transfer_yaw_goal = th_rad;
   double transfer_pose_x_goal = cos(transfer_yaw)*pose_info[_INFO_X_GOAL] + sin(transfer_yaw)*pose_info[_INFO_Y_GOAL] ;
   double transfer_pose_y_goal = -sin(transfer_yaw)*pose_info[_INFO_X_GOAL] + cos(transfer_yaw)*pose_info[_INFO_Y_GOAL] ; 
//     double transfer_pose_x = cos(transfer_yaw)*x - sin(transfer_yaw)*y;
//    double transfer_pose_y = sin(transfer_yaw)*x + cos(transfer_yaw)*y;
   
//    double diff_pose_x = x_goal-transfer_pose_x;
//    double diff_pose_y = y_goal-transfer_pose_y;
   double diff_pose_z = pose_info[_INFO_Z_GOAL]-pose_info[_INFO_Z];
  
   double diff_pose_x = transfer_pose_x_goal - transfer_pose_x;
   double diff_pose_y = transfer_pose_y_goal - transfer_pose_y;


   double tan_degree = atan2(diff_pose_y,diff_pose_x);
   //double diff_tan_yaw = tan_degree-th_rad;
   double z_pid=z_speed_pid.calculate(ZERO_FLOAT,diff_pose_z);

   if(pose_info[_INFO_SPEED_Z]<z_pid)pose_info[_INFO_SPEED_Z]+=0.05;
   else pose_info[_INFO_SPEED_Z]=z_pid;

    if(diff_pose_z>pose_info[_INFO_SAT_RANGE_Z])pose_info[_INFO_Z_VEL]=pose_info[_INFO_SPEED_Z];
    else if(diff_pose_z<(-1*pose_info[_INFO_SAT_RANGE_Z]))pose_info[_INFO_Z_VEL]=(-1*pose_info[_INFO_SPEED_Z]);
    else pose_info[_INFO_Z_VEL]=ZERO_FLOAT;

   #if PID_CHECK
      // PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
      PID pid = PID(0.1,1.0,0,0.1,0.01,0.5);
      double tmp = pid.calculate(0,20);
   #endif
  
  
   std::vector<std::pair<float,float>> point;
   point.emplace_back(std::make_pair(pose_info[_INFO_X],pose_info[_INFO_Y]));
   point.emplace_back(std::make_pair(pose_info[_INFO_X_GOAL],pose_info[_INFO_Y_GOAL]));
   double distance = Distance(point);
 //  linear_speed_pid.dt=0.1;linear_speed_pid.Kd=0.1;linear_speed_pid.Ki=0.1;linear_speed_pid.Kp=0.1; linear_speed_pid.max=1.0; linear_speed_pid.min=0.0;
   

   double speed_pid= linear_speed_pid.calculate(ZERO_FLOAT,distance);
   if(pose_info[_INFO_SPEED]<speed_pid)pose_info[_INFO_SPEED]+=0.05;
   else pose_info[_INFO_SPEED]=speed_pid;
   pose_info[_INFO_Y_VEL] = (pose_info[_INFO_SPEED]) * sin(tan_degree);
   pose_info[_INFO_X_VEL] = (pose_info[_INFO_SPEED]) * cos(tan_degree);
   double fabs_th=fabs(pose_info[_INFO_TH_GOAL]-pose_info[_INFO_TH_RAD]);
   double diff_yaw=pose_info[_INFO_TH_GOAL]-pose_info[_INFO_TH_RAD];
   
   double turn_pid = th_w_pid.calculate(ZERO_FLOAT,fabs(diff_yaw));
   if(pose_info[_INFO_TURN]<turn_pid)pose_info[_INFO_TURN]+=0.05;
   else pose_info[_INFO_TURN]=turn_pid;
   if(fabs_th>M_PI){
      if(RAD_360 - fabs_th < pose_info[_INFO_SAT_RANGE_W]){
         pose_info[_INFO_TH_VEL]=ZERO_FLOAT;
      }
      else{
         if(diff_yaw>0){
            pose_info[_INFO_TH_VEL]=(-1*pose_info[_INFO_TURN]);
         }
         else{
            pose_info[_INFO_TH_VEL]=pose_info[_INFO_TURN];
         }
      }
   }
   else{
      if(fabs_th<pose_info[_INFO_SAT_RANGE_W])th_vel=0;
      else{
         if(diff_yaw>0){
            pose_info[_INFO_TH_VEL]=pose_info[_INFO_TURN];
         }
         else{
            pose_info[_INFO_TH_VEL]=(-1*pose_info[_INFO_TURN]);
         }         
      }
   }
   #if LINEAR_TRANS_CHECK
       
       //th_vel=0;
      //ROS_INFO("trans x: %f , trans y : %f , x :%f , y :%f ,z:%f ,th:%f ",transfer_pose_x,transfer_pose_y,x,y,z,th_deg);
      //ROS_INFO("th_goal:%f , th:%f, th_abs:%f , diff_th:%f",RAD_TO_DEG(th_goal),th_deg,RAD_TO_DEG(th_abs),RAD_TO_DEG(transfer_yaw));
      ROS_INFO("vel  %f %f %f %f",pose_info[_INFO_X_VEL],pose_info[_INFO_Y_VEL],pose_info[_INFO_Z_VEL],pose_info[_INFO_TH_VEL]);
   #endif
}


#if 0
void Vm_Basic_Command::Linear_Flight_Algorithm(void){
   
   
   double transfer_yaw = th_rad ;
//    double transfer_yaw= RAD_360-th_rad;
   double transfer_pose_x = cos(transfer_yaw)*x + sin(transfer_yaw)*y;
   double transfer_pose_y = -sin(transfer_yaw)*x + cos(transfer_yaw)*y;
   
   double transfer_yaw_goal = th_rad;
   double transfer_pose_x_goal = cos(transfer_yaw_goal)*x_goal + sin(transfer_yaw_goal)*y_goal;
   double transfer_pose_y_goal = -sin(transfer_yaw_goal)*x_goal + cos(transfer_yaw_goal)*y_goal; 
//     double transfer_pose_x = cos(transfer_yaw)*x - sin(transfer_yaw)*y;
//    double transfer_pose_y = sin(transfer_yaw)*x + cos(transfer_yaw)*y;
   
//    double diff_pose_x = x_goal-transfer_pose_x;
//    double diff_pose_y = y_goal-transfer_pose_y;
   double diff_pose_z = z_goal-z;
  
   double diff_pose_x = transfer_pose_x_goal - transfer_pose_x;
   double diff_pose_y = transfer_pose_y_goal - transfer_pose_y;


   double tan_degree = atan2(diff_pose_y,diff_pose_x);
   //double diff_tan_yaw = tan_degree-th_rad;

    if(diff_pose_z>SAT_RANGE_Z)z_vel=Z_SPEED;
    else if(diff_pose_z<(-1*SAT_RANGE_Z))z_vel=(-1*Z_SPEED);
    else z_vel=ZERO_FLOAT;

   y_vel = (speed) * sin(tan_degree);
   x_vel = (speed) * cos(tan_degree);
    
   double diff_yaw=th_goal-th_rad;
   if(fabs_th>M_PI){
      if(RAD_360 - fabs_th < SAT_RANGE_YAW){
         th_vel=ZERO_FLOAT;
      }
      else{
         if(diff_yaw>0){
            th_vel=(-1*turn);
         }
         else{
            th_vel=turn;
         }
      }
   }
   else{
      if(fabs_th<SAT_RANGE_YAW)th_vel=0;
      else{
         if(diff_yaw>0){
            th_vel=turn;
         }
         else{
            th_vel=(-1*turn);
         }         
      }
   }
   #if LINEAR_TRANS_CHECK
       
       //th_vel=0;
      ROS_INFO("trans x: %f , trans y : %f , x :%f , y :%f ,z:%f ,th:%f ",transfer_pose_x,transfer_pose_y,x,y,z,th_deg);
      //ROS_INFO("th_goal:%f , th:%f, th_abs:%f , diff_th:%f",RAD_TO_DEG(th_goal),th_deg,RAD_TO_DEG(th_abs),RAD_TO_DEG(transfer_yaw));
      ROS_INFO("vel  %f %f %f %f",x_vel,y_vel,z_vel,th_vel);
   #endif
}
#endif

#endif