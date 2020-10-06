#include <ros/ros.h>
#include <splx/curve/PiecewiseCurve.hpp>
#include <splx/curve/Bezier.hpp>
#include <rlss_ros/PiecewiseTrajectory.h>
#include <rlss_ros/Bezier.h>
#include "../third_party/rlss/third_party/json.hpp"
#include <fstream>
#include <ios>
#include <std_srvs/Empty.h>

constexpr unsigned int DIM = DIMENSION;

using Bezier = splx::Bezier<double, DIM>;
using PiecewiseCurve = splx::PiecewiseCurve<double, DIM>;
using VectorDIM = Bezier::VectorDIM;

ros::Publisher pub;
rlss_ros::PiecewiseTrajectory msg;

bool setDesiredTrajectory(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res) {
    pub.publish(msg);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_desired_trajectory_feeder");
    ros::NodeHandle nh;

    std::string robot_description_path;
    nh.getParam("robot_description_path", robot_description_path);

    std::fstream json_fs(robot_description_path, std::ios_base::in);
    nlohmann::json robot_desc = nlohmann::json::parse(json_fs);

    PiecewiseCurve traj;

    for(const auto& piece: robot_desc["original_trajectory"]["pieces"]) {
        if(piece["type"] != "BEZIER") {
            ROS_ERROR_STREAM("piece type should be bezier");
            return 0;
        }

        double duration = piece["duration"];

        Bezier bez(duration);
        for(const auto& cpt: piece["control_points"]) {
            std::vector<double> cptv = cpt;
            if(cptv.size() != DIM) {
                ROS_ERROR_STREAM("cpt size does not match DIM");
                return 0;
            }
            VectorDIM cpt_vec;

            for(unsigned int i = 0; i < DIM; i++) {
                cpt_vec(i)= cptv[i];
            }
            bez.appendControlPoint(cpt_vec);

        }
        traj.addPiece(bez);
    }

    pub = nh.advertise<rlss_ros::PiecewiseTrajectory>("desired_trajectory", 1);


    for(std::size_t i = 0; i < traj.numPieces(); i++) {
        const Bezier& bez = traj[i];

        rlss_ros::Bezier bez_msg;
        bez_msg.dimension = DIM;
        bez_msg.duration = bez.maxParameter();

        for(std::size_t j = 0; j < bez.numControlPoints(); j++) {
            for(unsigned int d = 0; d < DIM; d++) {
                bez_msg.cpts.push_back(bez[j](d));
            }
        }
        msg.pieces.push_back(bez_msg);
    }

    ros::ServiceServer service = nh.advertiseService("set_desired_trajectory", setDesiredTrajectory);

    ros::spin();

    return 0;
}