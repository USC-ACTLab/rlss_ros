#include <rlss_ros/LandAction.h>
#include <rlss_ros/FollowTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include <rlss_ros/SetOnOff.h>
#include <rlss_ros/RobotState.h>
#include <rlss/internal/Util.hpp>
#include <deque>
#include <limits>
#include <rlss_ros/Bezier.h>
#include <rlss_ros/PiecewiseTrajectory.h>
#include <crazyflie_driver/GoTo.h>
#include <crazyflie_driver/Land.h>

using Server = actionlib::SimpleActionServer<rlss_ros::LandAction>;
using Client = actionlib::SimpleActionClient<rlss_ros::FollowTrajectoryAction>;
constexpr unsigned int DIM = DIMENSION;

using Bezier = splx::Bezier<double, DIM>;
using VectorDIM = Bezier::VectorDIM;
using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, DIM>;
using StdDequeVectorDIM = std::deque<VectorDIM, Eigen::aligned_allocator<std::pair<const int, VectorDIM>>>;

ros::ServiceClient ControllerOnOffClient;
ros::ServiceClient CrazyflieLandClient;
ros::ServiceClient CrazyflieGoToClient;
std::unique_ptr<Client> client;
StdVectorVectorDIM state;
unsigned int continuity_upto_degree;
double max_velocity = std::numeric_limits<double>::infinity();
bool with_controller;

void execute(const rlss_ros::LandGoalConstPtr& goal, Server* as) {

    rlss_ros::Bezier bez;
    bez.dimension = DIM;
    bez.duration = std::max(2.0, std::abs(state[0](2) - goal->land_height) / (max_velocity / 5));
    for(unsigned int d = 0; d < DIM; d++) {
        bez.cpts.push_back(state[0](d));
    }
    for(unsigned int d = 0; d < DIM-1; d++) {
        bez.cpts.push_back(state[0](d));
    }
    bez.cpts.push_back(goal->land_height);

    rlss_ros::PiecewiseTrajectory traj;
    traj.pieces.push_back(bez);
    traj.generation_time.data = ros::Time::now();


    rlss_ros::FollowTrajectoryGoal ft_goal;
    ft_goal.trajectory = traj;

    client->sendGoal(ft_goal);

    ros::Rate rate(2);

    rlss_ros::LandFeedback feedback;

    bool preempted = false;

    while(ros::ok()) {
        ros::spinOnce();

        if(as->isPreemptRequested()) {
            client->cancelGoal();
            preempted = true;
            break;
        } else {
            bool result = client->waitForResult(ros::Duration(0, 1));
            if(result) {
                preempted = false;
                break;
            }
            feedback.current_height = state[0](2);
            as->publishFeedback(feedback);
        }
        rate.sleep();
    }


    if(!preempted) {
        rlss_ros::LandResult result;
        if(with_controller) {
            rlss_ros::SetOnOff set_on_off;
            set_on_off.request.on = false;
            ControllerOnOffClient.call(set_on_off);
        } else {
            // crazyflie hack
            crazyflie_driver::GoTo goto_srv;
            goto_srv.request.groupMask = 0;
            goto_srv.request.relative = false;
            goto_srv.request.goal.z = *std::prev(bez.cpts.end());
            goto_srv.request.goal.y = *std::prev(std::prev(bez.cpts.end()));
            goto_srv.request.goal.x = *std::prev(std::prev(std::prev(bez.cpts.end())));
            goto_srv.request.yaw = 0;
            goto_srv.request.duration = ros::Duration(1.5);

            CrazyflieGoToClient.call(goto_srv);
            ros::Duration(2.0).sleep();

            crazyflie_driver::Land land_srv;
            land_srv.request.groupMask = 0;
            land_srv.request.height = 0.06;
            land_srv.request.duration = ros::Duration(2.0);

            CrazyflieLandClient.call(land_srv);
            ros::Duration(2.5).sleep();
        }
        as->setSucceeded(result);
    } else {
        as->setPreempted();
    }
}

void selfStateCallback(const rlss_ros::RobotState::ConstPtr& msg) {
    if(msg->vars.size() != DIM * (continuity_upto_degree + 1)) {
        ROS_FATAL_STREAM("state message vars length is not valid.");
    } else {
        for(unsigned int i = 0; i <= continuity_upto_degree; i++) {
            for(unsigned int j = 0; j < DIM; j++) {
                state[i](j) = msg->vars[i*DIM + j];
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "land_server");
    ros::NodeHandle nh;

    if(DIM != 3U) {
        ROS_ERROR_STREAM("dimension should be 3 for land action");
        return 0;
    }

    int c_upto_d;
    nh.getParam("continuity_upto_degree", c_upto_d);
    continuity_upto_degree = c_upto_d;
    state.resize(continuity_upto_degree + 1);

    nh.getParam("with_controller", with_controller);

    std::vector<int> degrees;
    nh.getParam("maximum_derivative_magnitude_degrees", degrees);
    std::vector<double> max_derivatives;
    nh.getParam("maximum_derivative_magnitude_magnitudes", max_derivatives);
    for(int i = 0; i < degrees.size(); i++) {
        if(degrees[i] == 1) {
            max_velocity = max_derivatives[i];
            break;
        }
    }

    if(with_controller) {
        ros::service::waitForService("controller/SetOnOff");
        ControllerOnOffClient = nh.serviceClient<rlss_ros::SetOnOff>("controller/SetOnOff", true);
        if(!ControllerOnOffClient) {
            ROS_ERROR_STREAM("ControllerOnOffClient not connected");
            return 0;
        }
    }

    if(ros::service::waitForService("go_to", 2)) {
        CrazyflieGoToClient = nh.serviceClient<crazyflie_driver::GoTo>("go_to", true);
        if(!CrazyflieGoToClient) {
            ROS_ERROR_STREAM("CrazyflieGoToClient not connected");
        }
    } else {
        ROS_WARN_STREAM("go_to service is not there");
    }


    if(ros::service::waitForService("land", 2)) {
        CrazyflieLandClient = nh.serviceClient<crazyflie_driver::Land>("land", true);
        if(!CrazyflieLandClient) {
            ROS_ERROR_STREAM("CrazyflieLandClient not connected");
        }
    } else {
        ROS_WARN_STREAM("land service is not there");
    }

    client = std::make_unique<Client>("FollowTrajectory", true);
    client->waitForServer();
    ros::Subscriber statesub = nh.subscribe("FullCurrentState", 1, selfStateCallback);

    Server server(nh, "Land", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();

    return 0;
}