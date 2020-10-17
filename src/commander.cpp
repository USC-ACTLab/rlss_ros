#include "ros/ros.h"
#include <rlss_ros/RobotState.h>
#include <splx/curve/PiecewiseCurve.hpp>
#include <rlss_ros/PiecewiseTrajectory.h>
#include <rlss_ros/Bezier.h>

constexpr unsigned int DIM = DIMENSION;

using PiecewiseCurve = splx::PiecewiseCurve<double, DIM>;
using Bezier = splx::Bezier<double, DIM>;
using VectorDIM = Bezier::VectorDIM;
PiecewiseCurve traj;
ros::Time last_traj_update_time(0);

void trajectoryCallback(const rlss_ros::PiecewiseTrajectory::ConstPtr& msg) {
    PiecewiseCurve curve;

    for(const rlss_ros::Bezier& bez: msg->pieces) {

        unsigned int bezdim = bez.dimension;

        if(bezdim != DIM) {
            ROS_WARN_STREAM("bez dimension does not match DIM");
            return;
        }

        Bezier piece(bez.duration);

        for(std::size_t i = 0; i < bez.cpts.size() / bezdim; i++) {
            VectorDIM cpt;
            for(unsigned int j = 0; j < DIM; j++) {
                cpt(j) = bez.cpts[i*bezdim + j];
            }
            piece.appendControlPoint(cpt);
        }

        curve.addPiece(piece);
    }

    traj = curve;
    last_traj_update_time = msg->generation_time.data;
//    last_traj_update_time = ros::Time::now();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "commander");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("trajectory", 1, trajectoryCallback);
    ros::Publisher pub = nh.advertise<rlss_ros::RobotState>("full_desired_state", 1);

    double dt;
    nh.getParam("commander_dt", dt);

    int c_u_d;
    nh.getParam("continuity_upto_degree", c_u_d);
    unsigned int continuity_upto_degree = c_u_d;

    ros::Rate rate(1/dt);
    while(ros::ok()) {
        ros::spinOnce();
        if(last_traj_update_time != ros::Time(0)) {
            ros::Time current_time = ros::Time::now();
            rlss_ros::RobotState msg;
            msg.dimension = DIM;

            for(unsigned int i = 0; i <= continuity_upto_degree; i++) {
                VectorDIM vec = traj.eval(std::min((current_time - last_traj_update_time).toSec() + dt, traj.maxParameter()), i);
                for(unsigned int d = 0; d < DIM; d++) {
                    msg.vars.push_back(vec(d));
                }
            }
            pub.publish(msg);
        }
        rate.sleep();
    }

    return 0;
}