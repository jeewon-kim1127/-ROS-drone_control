//move in 10*10 square with position P control using mavros
//coded by jeewon kim https://github.com/jeewon-kim1127


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

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


void controller_timer_function(const ros::TimerEvent& ){
    //ROS_INFO("timer callback triggered");

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;
}

double Eucl_dist(double goal_x, double goal_y, double goal_z, double real_x, double real_y, double real_z){
    return sqrt(pow(goal_x-real_x,2.0)+pow(goal_y-real_y,2.0)+pow(goal_z-real_z,2.0));
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
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

    double goal_x = 0;
    double goal_y = 0;
    double goal_z = 5;
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
                }
                last_request = ros::Time::now();
            }
        }

        if( Eucl_dist(goal_x, goal_y, goal_z, real_x, real_y, real_z) < 0.2 ){
        	//if drone is close with the goal position(which is the vertex of the square), then set next goal position
                if( step%4 == 0 ){
                    goal_x = 10;
                    pose.pose.position.x = goal_x;
                }
                else if( step%4 == 1 ){
                    goal_y = 10;
                    pose.pose.position.y = goal_y;
                }
                else if( step%4 == 2 ){
                    goal_x = 0;
                    pose.pose.position.x = goal_x;
                }
                else if( step%4 == 3 ){
                    goal_y = 0;
                    pose.pose.position.y = goal_y;
                }
                step++;
        }
        
        else{
        	//make drone move by publishing during the drone is on the edged of the square
            ROS_INFO("POSE: %f %f %f", pose.pose.position.x, pose.pose.position.y ,pose.pose.position.z );
            local_pos_pub.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
	}   
	
    return 0;
}

