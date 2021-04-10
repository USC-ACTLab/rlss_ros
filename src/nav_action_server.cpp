#include <rlss_ros/LandAction.h>
#include <rlss_ros/FollowTrajectoryAction.h>
#include <rlss_ros/TakeoffAction.h>
#include <rlss_ros/NAVAction.h>
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

using NAVServer = actionlib::SimpleActionServer<rlss_ros::NAVAction>;
using FollowTrajectoryClient = actionlib::SimpleActionClient<rlss_ros::FollowTrajectoryAction>;
using LandClient = actionlib::SimpleActionClient<rlss_ros::LandAction>;
using TakeoffClient = actionlib::SimpleActionClient<rlss_ros::TakeoffAction>;
constexpr unsigned int DIM = DIMENSION;

using Bezier = splx::Bezier<double, DIM>;
using VectorDIM = Bezier::VectorDIM;
using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, DIM>;
using StdDequeVectorDIM = std::deque<VectorDIM, Eigen::aligned_allocator<std::pair<const int, VectorDIM>>>;

std::unique_ptr<FollowTrajectoryClient> ftcli;
std::unique_ptr<LandClient> landcli;
std::unique_ptr<TakeoffClient> tcli;


void execute(const rlss_ros::NAVGoalConstPtr& goal, NAVServer* as) {
    rlss_ros::NAVFeedback feedback;
    feedback.current_state = feedback.TAKING_OFF;

    rlss_ros::TakeoffGoal takeoff_goal;
    takeoff_goal.takeoff_height = goal->takeoff_height;

    tcli->sendGoal(takeoff_goal);

    ros::Rate rate(2);

    bool preempted = false;

    while(ros::ok()) {
        ros::spinOnce();

        if(as->isPreemptRequested()) {
            tcli->cancelGoal();
            preempted = true;
            break;
        } else {
            bool result = tcli->waitForResult(ros::Duration(0, 1));
            if(result) {
                preempted = false;
                break;
            }
            as->publishFeedback(feedback);
        }
        rate.sleep();
    }

    if(preempted) {
        as->setPreempted();
        return;
    }

    rlss_ros::FollowTrajectoryGoal ft_goal;
    ft_goal.trajectory = goal->desired_trajectory;
    ft_goal.trajectory.generation_time.data = ros::Time::now();

    ftcli->sendGoal(ft_goal);
    feedback.current_state = feedback.FOLLOWING;

    while(ros::ok()) {
        ros::spinOnce();

        if(as->isPreemptRequested()) {
            ftcli->cancelGoal();
            preempted = true;
            break;
        } else {
            bool result = ftcli->waitForResult(ros::Duration(0, 1));
            if(result) {
                preempted = false;
                break;
            }
            as->publishFeedback(feedback);
        }
        rate.sleep();
    }

    if(preempted) {
        as->setPreempted();
        return;
    }

    rlss_ros::LandGoal land_goal;
    land_goal.land_height = goal->land_height;

    landcli->sendGoal(land_goal);
    feedback.current_state = feedback.LANDING;

    while(ros::ok()) {
        ros::spinOnce();

        if(as->isPreemptRequested()) {
            ftcli->cancelGoal();
            preempted = true;
            break;
        } else {
            bool result = ftcli->waitForResult(ros::Duration(0, 1));
            if(result) {
                preempted = false;
                break;
            }
            as->publishFeedback(feedback);
        }
        rate.sleep();
    }

    if(preempted) {
        as->setPreempted();
    } else {
        as->setSucceeded();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nav_server");
    ros::NodeHandle nh;

    if(DIM != 3U) {
        ROS_ERROR_STREAM("dimension should be 3 for nav action");
        return 0;
    }


    ftcli = std::make_unique<FollowTrajectoryClient>("FollowTrajectory", true);
    ftcli->waitForServer();

    landcli = std::make_unique<LandClient>("Land", true);
    landcli->waitForServer();

    tcli = std::make_unique<TakeoffClient>("Takeoff", true);
    tcli->waitForServer();

    NAVServer server(nh, "NAV", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();

    return 0;
}