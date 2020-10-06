#include <ros/ros.h>
#include <rlss_ros/RobotState.h>
#include <create2_controller/TrajectoryState2D.h>
#include <algorithm>

ros::Publisher state_pub;

void fullDesiredStateCallback(const rlss_ros::RobotState::ConstPtr& msg) {
    create2_controller::TrajectoryState2D reduced_state;

    for(std::size_t i = 0; i < std::min(static_cast<int>(msg->vars.size() / msg->dimension), 3); i++) {
        if(i == 0) {
            reduced_state.position.x = msg->vars[i*msg->dimension];
            reduced_state.position.y = msg->vars[i*msg->dimension + 1];
        } else if(i == 1) {
            reduced_state.velocity.x = msg->vars[i*msg->dimension];
            reduced_state.velocity.y = msg->vars[i*msg->dimension + 1];
        } else if(i == 2) {
            reduced_state.acceleration.x = msg->vars[i*msg->dimension];
            reduced_state.acceleration.y = msg->vars[i*msg->dimension + 1];
        }
    }

    state_pub.publish(reduced_state);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "state_reducer");
    ros::NodeHandle nh;

    ros::Subscriber fds_subscriber = nh.subscribe("full_desired_state", 1, fullDesiredStateCallback);
    state_pub = nh.advertise<create2_controller::TrajectoryState2D>("desired_state", 1);

    ros::spin();
    return 0;
}