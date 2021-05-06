//move in 10*10 square with velocity PD control using mavros
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
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    geometry_msgs::PoseStamped pose=*msg;
    real_x = pose.pose.position.x;
    real_y = pose.pose.position.y;
    real_z = pose.pose.position.z;
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

double Eucl_dist(double goal_x, double goal_y, double goal_z, double real_x, double real_y, double real_z){
    return sqrt(pow(goal_x-real_x,2.0)+pow(goal_y-real_y,2.0)+pow(goal_z-real_z,2.0));
}
double total_vel_diff(double goal_vx, double goal_vy, double goal_vz, double real_vx, double real_vy, double real_vz){
  return sqrt(pow(goal_vx-real_vx,2.0)+pow(goal_vy-real_vy,2.0)+pow(goal_vz-real_vz,2.0));
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

    double goal_x = real_x;
    double goal_y = real_y;
    double goal_z = 5;

    double goal_vx = 0;
    double goal_vy = 0;
    double goal_vz = 1;

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
                    printf("original position: %f %f", orig_x, orig_y);
                    goal_x = orig_x;
                    goal_y = orig_y;
                    goal_z = 5;
                }
                last_request = ros::Time::now();
            }
        }

         //printf("position, goal:(%f,%f,%f), cur:(%f,%f,%f) \n", goal_x, goal_y, goal_z, real_x, real_y, real_z);
         //printf("vel, goal:(%f %f %f) real:(%f %f %f)  \n", goal_vx, goal_vy, goal_vz,real_vx, real_vy, real_vz);
         if( abs(real_x-goal_x) < 0.5 && abs(real_y-goal_y)< 0.5 && abs(real_z-goal_z)<0.5 ){
            goal_vz=1; goal_z=5;
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
            goal_vx = 0.2*(goal_vx-real_vx)+0.4*(goal_x-real_x);
            goal_vy = 0.2*(goal_vy-real_vy)+0.4*(goal_y-real_y);
            goal_vz = 0.2*(goal_vz-real_vz)+0.4*(goal_z-real_z);

            vel.twist.linear.x = goal_vx;
            vel.twist.linear.y = goal_vy;
            vel.twist.linear.z = goal_vz;

            cmd_vel_pub.publish(vel);
            }

        ros::spinOnce();
        rate.sleep();

    }
	

    return 0;
}
