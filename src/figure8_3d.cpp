#include "ros/ros.h"
#include <splx/curve/Bezier.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <rlss_ros/RobotState.h>

using Bezier = splx::Bezier<double, 3U>;
using PiecewiseCurve = splx::PiecewiseCurve<double, 3U>;
using VectorDIM = Bezier::VectorDIM;

int main(int argc, char** argv) {
    ros::init(argc, argv, "figure8");
    ros::NodeHandle nh;

    double duration;
    nh.getParam("figure8_duration", duration);

    ROS_INFO_STREAM(duration);

    Bezier piece1(duration/2);
    piece1.appendControlPoint(VectorDIM(0,0, 2));
    piece1.appendControlPoint(VectorDIM(1.5,-1.5, 2));
    piece1.appendControlPoint(VectorDIM(1.5,1.5, 2));
    piece1.appendControlPoint(VectorDIM(0,0, 2));

    Bezier piece2(duration/2);
    piece2.appendControlPoint(VectorDIM(0,0, 2));
    piece2.appendControlPoint(VectorDIM(-1.5,-1.5, 2));
    piece2.appendControlPoint(VectorDIM(-1.5,1.5,2 ));
    piece2.appendControlPoint(VectorDIM(0,0,2));


    PiecewiseCurve curve;
    curve.addPiece(piece1);
    curve.addPiece(piece2);

    ros::Publisher pub = nh.advertise<rlss_ros::RobotState>("full_desired_state", 1);

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(1/0.05);


    while(ros::ok()) {
        double t = (ros::Time::now() - start_time).toSec();

        rlss_ros::RobotState msg;
        msg.dimension = 3;
        for(unsigned int i = 0; i < 2; i++) {
            VectorDIM ev = curve.eval(std::fmod(t, curve.maxParameter()), i);
            for(unsigned int d = 0; d < 3U; d++) {
                msg.vars.push_back(ev(d));
            }
        }


        pub.publish(msg);

        rate.sleep();
    }
    return 0;
}