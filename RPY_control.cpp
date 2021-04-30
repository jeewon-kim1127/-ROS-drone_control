//move in 10*10 square with RPY PID control using mavros
//coded by jeewon kim https://github.com/jeewon-kim1127

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define PI 3.14

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double real_x; double real_y; double real_z;
double real_p; double real_r; double real_yaw;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    geometry_msgs::PoseStamped pose=*msg;
    real_x = pose.pose.position.x;
    real_y = pose.pose.position.y;
    real_z = pose.pose.position.z;

    tf2::Quaternion q_real;
    tf2::convert(pose.pose.orientation, q_real);
    tf2::Matrix3x3(q_real).getRPY(real_r, real_p, real_yaw);
}

double real_vx; double real_vy; double real_vz;
void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    geometry_msgs::TwistStamped vel=*msg;
    real_vx = vel.twist.linear.x;
    real_vy = vel.twist.linear.y;
    real_vz = vel.twist.linear.z;
}

void controller_timer_function(const ros::TimerEvent& ){
    //ROS_INFO("timer callback triggered");

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

    geometry_msgs::TwistStamped vel;
    vel.header.stamp = ros::Time::now();
    vel.twist.linear.x = 0;
    vel.twist.linear.y = 0;
    vel.twist.linear.z = 0.5;
}

double thrust_saturation(double thrust){
  if (thrust>1){ return 0.98; }
  else if (thrust<0){ return 0.05; }
  else return thrust;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
             ("mavros/local_position/velocity_local", 10, velocity_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/pose", 10);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher cmd_raw_pub = nh.advertise<mavros_msgs::AttitudeTarget>
                ("mavros/setpoint_raw/attitude", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Timer controller_timer = nh.createTimer(ros::Duration(1/30.0), controller_timer_function);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x = 0;
    vel.twist.linear.y = 0;
    vel.twist.linear.z = 0.5;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;

    mavros_msgs::AttitudeTarget raw;
    raw.orientation.w = 0;
    raw.orientation.x = 0;
    raw.orientation.y = 0;
    raw.orientation.z = 0;
    raw.thrust = 0.57;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    double orig_x=real_x;
    double orig_y=real_y;
    double orig_yaw = real_yaw;

    double goal_x = real_x;
    double goal_y = real_y;
    double goal_z = 5;

    double goal_vx = 0;
    double goal_vy = 0;
    double goal_vz = 0.5;

    double goal_r = 0;
    double goal_p = 0;
    double goal_yaw = 0;
    float goal_thrust = 0.57;

    int step = 0;

    while(ros::ok()){

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");

                    orig_x=real_x;
                    orig_y=real_y;

                    goal_x = orig_x;
                    goal_y = orig_y;
                    goal_z = 5;

                    orig_yaw=real_yaw;
                }
                last_request = ros::Time::now();
            }
        }

         //printf("position, goal:(%f,%f,%f) \n .....cur:(%f,%f,%f) \n", goal_x, goal_y, goal_z, real_x, real_y, real_z);
         if( abs(real_x-goal_x) < 0.5 && abs(real_y-goal_y)< 0.5 && abs(real_z-goal_z)<0.5){
              goal_vz=0; goal_z=5;
              printf("step%d \n", step);

                  if ( step %4 == 0 ){
                    goal_x = orig_x+5;
                    goal_y = orig_y;
                  }
                  else if ( step %4 == 1 ){
                    goal_x = orig_x + 5;
                    goal_y = orig_y + 5;
                  }
                  else if( step %4 == 2 ){
                    goal_x = orig_x;
                    goal_y = orig_y + 5;
                  }
                  else if ( step %4 == 3 ){
                    goal_x = orig_x;
                    goal_y = orig_y;
                  }
                  step++;
            }
            
            else{
                goal_vx = 0.8*(goal_x-real_x);
                goal_vy = 0.8*(goal_y-real_y);
                goal_vz = 0.8*(goal_z-real_z);

                goal_p = 0.1*(goal_vx-real_vx);
                goal_r = -0.1*(goal_vy-real_vy);
                goal_yaw = 0;
                
                goal_thrust=thrust_saturation(0.57+0.6*(goal_z-real_z)+0.2*(goal_vz-real_vz)+0.01*(real_z+0.1*(goal_z-real_z)));

                tf2::Quaternion q_goal;
                q_goal.setRPY(goal_r,goal_p,goal_yaw);
                raw.orientation.w = q_goal.w();
                raw.orientation.x = q_goal.x();
                raw.orientation.y = q_goal.y();
                raw.orientation.z = q_goal.z();
                raw.thrust = goal_thrust;

                cmd_raw_pub.publish(raw);
              }
              
        ros::spinOnce();
        rate.sleep();

    }


    return 0;
}
