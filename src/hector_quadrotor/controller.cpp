#include <ros/ros.h>
#include <rlss_ros/RobotState.h>
#include <rlss/internal/Util.hpp>
#include <tf/transform_listener.h>
//#include <tf/Quaternion.h>
//#include <tf/Vector3.h>
#include<geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

constexpr unsigned int DIM = 3U;

using VectorDIM = rlss::internal::VectorDIM<double, DIM>;
using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, DIM>;


StdVectorVectorDIM desired_state;
unsigned int continuity_upto_degree;
bool desired_state_set = false;

void fullDesiredStateCallback(const rlss_ros::RobotState::ConstPtr& msg) {
    if(msg->dimension != DIM) {
        ROS_WARN_STREAM("dim mismatch");
        return;
    }

    for(std::size_t i = 0; i < std::min(static_cast<unsigned int>(msg->vars.size()), DIM * (continuity_upto_degree + 1)); i++) {
        desired_state[i / DIM](i % DIM) = msg->vars[i];
    }
    desired_state_set = true;
}

VectorDIM prev_pos;
ros::Time prev_pos_time = ros::Time(0);
VectorDIM prev_pos_error;
VectorDIM prev_vel_error;
ros::Time prev_vel_time = ros::Time(0);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    int c_u_d;
    nh.getParam("continuity_upto_degree", c_u_d);
    continuity_upto_degree = c_u_d;
    desired_state.resize(c_u_d);

    std::string world_frame;
    nh.getParam("world_frame", world_frame);
    std::string tf_prefix;
    nh.getParam("tf_prefix", tf_prefix);
    world_frame = tf_prefix + "/" + world_frame;
    std::string robot_frame;
    nh.getParam("base_link_frame", robot_frame);

    std::cout << "world_frame: " << world_frame << ", robot_frame: " << robot_frame << std::endl;


    ros::NodeHandle pnh("~");
    double kpp, kpv, kdp, kdv;
    pnh.getParam("kpp", kpp);
    pnh.getParam("kpv", kpv);
    pnh.getParam("kdp", kdp);
    pnh.getParam("kdv", kdv);
    std::cout << "kpp: " << kpp << ", kpv: " << kpv << ", kdp: " << kdp << ", kdv: " << kdv << std::endl;

    ros::Subscriber sub = nh.subscribe("full_desired_state", 1, fullDesiredStateCallback);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
//    ros::Publisher ss = nh.advertise<geometry_msgs::Pose>("current_pose", 1);

    tf::TransformListener listener;

    listener.waitForTransform(world_frame, robot_frame, ros::Time::now(), ros::Duration(3.0));

    ros::Rate rate(100);

    while(ros::ok()) {
        ros::spinOnce();
        if(desired_state_set) {
            tf::Vector3 pos;
            tf::Quaternion ori;

            tf::StampedTransform transform;
            listener.lookupTransform(world_frame, robot_frame, ros::Time(0), transform);
            pos = transform.getOrigin();
            ori = transform.getRotation();

//            geometry_msgs::Pose pose; //debug
//            pose.position.x = pos.x();
//            pose.position.y = pos.y();
//            pose.position.z = pos.z();
//            ss.publish(pose);

            VectorDIM current_pos(pos.x(), pos.y(), pos.z());

//            std::cout << "current_pos: " << current_pos.transpose() << std::endl;
//            std::cout << "desired_pos: " << desired_state[0].transpose() << std::endl;

            VectorDIM pos_error = desired_state[0] - current_pos;
//            std::cout << "pos_error: " << pos_error.transpose() << std::endl;
            VectorDIM input = kpp * pos_error;
//            std::cout << "input: " << input.transpose() << std::endl;

            if(prev_pos_time != ros::Time(0)) {
                input += kdp * (pos_error - prev_pos_error);

                VectorDIM vel = (current_pos - prev_pos) /
                                (ros::Time::now() - prev_pos_time).toSec();
                VectorDIM vel_error = (desired_state[1] - vel);
                input += kpv * vel_error;
                if(prev_vel_time != ros::Time(0)) {
                    input += kdv * (vel_error - prev_vel_error);
                }
                prev_vel_error = vel_error;
                prev_vel_time = ros::Time::now();
            }

            prev_pos_error = pos_error;
            prev_pos = current_pos;
            prev_pos_time = ros::Time::now();

//            std::cout << "input: " << input.transpose() << std::endl;

            tf::Vector3 in_v(input(0), input(1), input(2));
            in_v = tf::quatRotate(ori.inverse(), in_v);

            geometry_msgs::Twist msg;
            msg.linear.x = in_v.getX();
            msg.linear.y = in_v.getY();
            msg.linear.z = in_v.getZ();

            pub.publish(msg);
        }
        rate.sleep();
    }
    return 0;
}