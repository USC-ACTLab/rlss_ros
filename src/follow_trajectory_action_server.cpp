#include <rlss_ros/FollowTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include "ros/ros.h"
#include <rlss_ros/SetOnOff.h>
#include <rlss_ros/DesiredTrajectory.h>
#include <rlss_ros/RobotState.h>
#include <splx/curve/PiecewiseCurve.hpp>
#include <rlss/internal/Util.hpp>
#include <deque>

using Server = actionlib::SimpleActionServer<rlss_ros::FollowTrajectoryAction>;
constexpr unsigned int DIM = DIMENSION;

using PiecewiseCurve = splx::PiecewiseCurve<double, DIM>;
using Bezier = splx::Bezier<double, DIM>;
using VectorDIM = Bezier::VectorDIM;
using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, DIM>;
using StdDequeVectorDIM = std::deque<VectorDIM, Eigen::aligned_allocator<std::pair<const int, VectorDIM>>>;

ros::ServiceClient ReplannerOnOffClient;
ros::ServiceClient CommanderOnOffClient;
ros::ServiceClient ControllerOnOffClient;
ros::ServiceClient ReplannerSetDesiredTrajectoryClient;
ros::Publisher FullDesiredStatePublisher;
StdVectorVectorDIM state;
unsigned int continuity_upto_degree;
bool with_controller;

void execute(const rlss_ros::FollowTrajectoryGoalConstPtr& goal, Server* as) {
    PiecewiseCurve curve;
    for(const rlss_ros::Bezier& bez: goal->trajectory.pieces) {

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
    ros::Time trajectory_generation_time = goal->trajectory.generation_time.data;


    VectorDIM target_position = curve.eval(curve.maxParameter(), 0);

    rlss_ros::SetOnOff set_on_off;
    rlss_ros::DesiredTrajectory desired_trajectory;

    set_on_off.request.on = true;
    desired_trajectory.request.trajectory = goal->trajectory;

    ReplannerSetDesiredTrajectoryClient.call(desired_trajectory);
    ReplannerOnOffClient.call(set_on_off);
    CommanderOnOffClient.call(set_on_off);
    if(with_controller)
        ControllerOnOffClient.call(set_on_off);


    ros::Rate rate(2);

    rlss_ros::FollowTrajectoryFeedback feedback;

    bool preempted = false;
    StdDequeVectorDIM position_queue;
    int max_queue_size = 10;

    while(ros::ok()) {
        ros::spinOnce();

        double total_distance = std::numeric_limits<double>::max();
        if(position_queue.size() == max_queue_size) {
            position_queue.pop_front();
        }
        position_queue.push_back(state[0]);
        if(position_queue.size() == max_queue_size) {
            total_distance = 0;
            for(int i = 1; i < max_queue_size; i++) {
                total_distance += (position_queue.at(i) - position_queue.at(i-1)).norm();
            }
        }


        if(as->isPreemptRequested()) {
            preempted = true;
            break;
        } else {
            if((target_position - state[0]).norm() < 0.05) {
                preempted = false;
                break;
            } else if(state[0](2) < 0.05) {
                feedback.current_state = feedback.CRASHED;
            } else if(position_queue.size() == 10 && total_distance < 0.008) {
                preempted = false;
                break;
                feedback.current_state = feedback.DEADLOCKED;
            } else {
                feedback.current_state = feedback.FOLLOWING;
            }
            feedback.current_time_on_trajectory = (ros::Time::now() - trajectory_generation_time).toSec();
            as->publishFeedback(feedback);
        }
        rate.sleep();
    }

    set_on_off.request.on = false;
    ReplannerOnOffClient.call(set_on_off);
    CommanderOnOffClient.call(set_on_off);

    rlss_ros::RobotState desired_state;
    desired_state.dimension = DIM;
    for(unsigned int i = 0; i < DIM; i++) {
        desired_state.vars.push_back(state[0](i));
    }
    for(unsigned int i = 1; i <= continuity_upto_degree; i++) {
        for(unsigned int d = 0; d < DIM; d++) {
            desired_state.vars.push_back(0);
        }
    }

    FullDesiredStatePublisher.publish(desired_state);

    if(!preempted) {
        rlss_ros::FollowTrajectoryResult result;
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
    ros::init(argc, argv, "follow_trajectory_server");
    ros::NodeHandle nh;

    int c_upto_d;
    nh.getParam("continuity_upto_degree", c_upto_d);
    continuity_upto_degree = c_upto_d;
    state.resize(continuity_upto_degree + 1);

    nh.getParam("with_controller", with_controller);

    ros::service::waitForService("replanner/SetOnOff");
    ReplannerOnOffClient = nh.serviceClient<rlss_ros::SetOnOff>("replanner/SetOnOff", true);
    if(!ReplannerOnOffClient) {
        ROS_ERROR_STREAM("ReplannerOnOffClient not connected");
        return 0;
    }


    ros::service::waitForService("commander/SetOnOff");
    CommanderOnOffClient = nh.serviceClient<rlss_ros::SetOnOff>("commander/SetOnOff", true);
    if(!CommanderOnOffClient) {
        ROS_ERROR_STREAM("CommanderOnOffClient not connected");
        return 0;
    }


    if(with_controller) {
        ros::service::waitForService("controller/SetOnOff");
        ControllerOnOffClient = nh.serviceClient<rlss_ros::SetOnOff>("controller/SetOnOff", true);
        if(!ControllerOnOffClient) {
            ROS_ERROR_STREAM("ControllerOnOffClient not connected");
            return 0;
        }
    }


    ros::service::waitForService("replanner/SetDesiredTrajectory");
    ReplannerSetDesiredTrajectoryClient = nh.serviceClient<rlss_ros::DesiredTrajectory>("replanner/SetDesiredTrajectory", true);
    if(!ReplannerSetDesiredTrajectoryClient) {
        ROS_ERROR_STREAM("ReplannerSetDesiredTrajectoryClient not connected");
        return 0;
    }

    ros::Subscriber statesub = nh.subscribe("FullCurrentState", 1, selfStateCallback);
    FullDesiredStatePublisher = nh.advertise<rlss_ros::RobotState>("FullDesiredState", 1);

    Server server(nh, "FollowTrajectory", boost::bind(&execute, _1, &server), false);
    server.start();
    ros::spin();

    return 0;
}