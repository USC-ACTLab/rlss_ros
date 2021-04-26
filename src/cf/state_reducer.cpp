#include "ros/ros.h"
#include <crazyflie_driver/FullState.h>
#include <rlss_ros/RobotState.h>
#include <geometry_msgs/Quaternion.h>

ros::Publisher state_pub;
crazyflie_driver::FullState reduced_state;

void fullDesiredStateCallback(const rlss_ros::RobotState::ConstPtr& msg) {
    if(msg->dimension != 3U) {
        ROS_WARN_STREAM("cf reducer state dimension is not 3");
        return;
    }

    reduced_state.header.stamp = ros::Time::now();
    reduced_state.header.seq += 1;

    if(msg->vars.size() > 0) {
        reduced_state.pose.position.x = msg->vars[0];
        reduced_state.pose.position.y = msg->vars[1];
        reduced_state.pose.position.z = msg->vars[2];
    } else {
        reduced_state.pose.position.x = 0;
        reduced_state.pose.position.y = 0;
        reduced_state.pose.position.z = 0;
    }

    if(msg->vars.size() > 3) {
        reduced_state.twist.linear.x = msg->vars[3];
        reduced_state.twist.linear.y = msg->vars[4];
        reduced_state.twist.linear.z = msg->vars[5];
    } else {
        reduced_state.twist.linear.x = 0;
        reduced_state.twist.linear.y = 0;
        reduced_state.twist.linear.z = 0;
    }

    if(msg->vars.size() > 6) {
        reduced_state.acc.x = msg->vars[6];
        reduced_state.acc.y = msg->vars[7];
        reduced_state.acc.z = msg->vars[8];
    } else {
        reduced_state.acc.x = 0;
        reduced_state.acc.y = 0;
        reduced_state.acc.z = 0;
    }

    state_pub.publish(reduced_state);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "state_reducer");
    ros::NodeHandle nh;

    reduced_state.header.seq = 0;
    reduced_state.header.frame_id = "/world";
    geometry_msgs::Quaternion quatmsg;
    quatmsg.x = 0;
    quatmsg.y = 0;
    quatmsg.z = 0;
    quatmsg.w = 0;
    reduced_state.pose.orientation = quatmsg;
    reduced_state.twist.angular.x = 0;
    reduced_state.twist.angular.y = 0;
    reduced_state.twist.angular.z = 0;


    state_pub = nh.advertise<crazyflie_driver::FullState>("cmd_full_state", 1);
    ros::Subscriber sub = nh.subscribe("FullDesiredState", 1, fullDesiredStateCallback);

    ros::spin();
    return 0;
}