#include "ros/ros.h"
#include <create2_controller/TrajectoryState2D.h>
#include <splx/curve/Bezier.hpp>
#include <splx/curve/PiecewiseCurve.hpp>

using Bezier = splx::Bezier<double, 2U>;
using PiecewiseCurve = splx::PiecewiseCurve<double, 2U>;
using VectorDIM = Bezier::VectorDIM;

int main(int argc, char** argv) {
    ros::init(argc, argv, "figure8");
    ros::NodeHandle nh;

    double duration;
    nh.getParam("figure8_duration", duration);

    ROS_INFO_STREAM(duration);

    Bezier piece1(duration/2);
    piece1.appendControlPoint(VectorDIM(0,0));
    piece1.appendControlPoint(VectorDIM(1.5,-1.5));
    piece1.appendControlPoint(VectorDIM(1.5,1.5));
    piece1.appendControlPoint(VectorDIM(0,0));

    Bezier piece2(duration/2);
    piece2.appendControlPoint(VectorDIM(0,0));
    piece2.appendControlPoint(VectorDIM(-1.5,-1.5));
    piece2.appendControlPoint(VectorDIM(-1.5,1.5));
    piece2.appendControlPoint(VectorDIM(0,0));


    PiecewiseCurve curve;
    curve.addPiece(piece1);
    curve.addPiece(piece2);

    ros::Publisher pub = nh.advertise<create2_controller::TrajectoryState2D>("desired_state", 1);

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(1/0.05);


    while(ros::ok()) {
        double t = (ros::Time::now() - start_time).toSec();

        VectorDIM pos = curve.eval(std::fmod(t, curve.maxParameter()), 0);
        VectorDIM vel = curve.eval(std::fmod(t, curve.maxParameter()), 1);
        VectorDIM acc = curve.eval(std::fmod(t, curve.maxParameter()), 2);

        create2_controller::TrajectoryState2D msg;

        msg.position.x = pos(0);
        msg.position.y = pos(1);
        msg.velocity.x = vel(0);
        msg.velocity.y = vel(1);
        msg.acceleration.x = acc(0);
        msg.acceleration.y = acc(1);

        pub.publish(msg);

        rate.sleep();
    }
    return 0;
}