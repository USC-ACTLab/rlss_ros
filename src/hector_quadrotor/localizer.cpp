#include <ros/ros.h>
#include <rlss_ros/RobotState.h>
#include <rlss_ros/AABBCollisionShape.h>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <rlss/internal/Util.hpp>
#include <tf/transform_listener.h>

constexpr unsigned int DIM = 3U;

using VectorDIM = rlss::internal::VectorDIM<double, DIM>;
using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, DIM>;
using AABBCollisionShape = rlss::AlignedBoxCollisionShape<double, DIM>;
using AlignedBox = rlss::internal::AlignedBox<double, DIM>;

int self_robot_idx;
StdVectorVectorDIM state;
unsigned int continuity_upto_degree;
std::shared_ptr<AABBCollisionShape> shape;

ros::Publisher self_state_publisher;
ros::Publisher collision_shape_publisher;

void publish() {
    rlss_ros::RobotState msg;
    msg.dimension = DIM;

    for(unsigned int i = 0; i <= continuity_upto_degree; i++) {
        for(unsigned int d = 0; d < DIM; d++) {
            msg.vars.push_back(state[i](d));
        }
    }

    self_state_publisher.publish(msg);

    AlignedBox current_cs = shape->boundingBox(state[0]);

    rlss_ros::AABBCollisionShape cs_msg;
    cs_msg.robot_idx = self_robot_idx;
    for(unsigned int i = 0; i < DIM; i++) {
        cs_msg.bbox.min.push_back(current_cs.min()(i));
        cs_msg.bbox.max.push_back(current_cs.max()(i));
    }

    collision_shape_publisher.publish(cs_msg);
}


void fullDesiredStateCallback(const rlss_ros::RobotState::ConstPtr& msg) {
    if(msg->dimension != DIM) {
        ROS_WARN_STREAM("full desired state callback dimension does not match DIM");
        return;
    }

    for(unsigned int i = 1; i <= continuity_upto_degree; i++) {
        for(unsigned int d = 0; d < DIM; d++) {
            state[i](d) = msg->vars[i*DIM + d];
        }
    }

    publish();
}



int main(int argc, char* argv[]) {
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;

    nh.getParam("robot_idx", self_robot_idx);

    int c_u_d;
    nh.getParam("continuity_upto_degree", c_u_d);
    continuity_upto_degree = c_u_d;
    state.resize(continuity_upto_degree, VectorDIM::Zero());


    std::vector<double> colshape_min_vec, colshape_max_vec;
    nh.getParam("collision_shape_at_zero_min", colshape_min_vec);
    nh.getParam("collision_shape_at_zero_max", colshape_max_vec);
    if(colshape_min_vec.size() != colshape_max_vec.size()) {
        ROS_FATAL_STREAM("colshape min-max size mismatch");
        return 0;
    }
    if(colshape_min_vec.size() != DIM) {
        ROS_FATAL_STREAM("colshape vector size not equal to dimension");
        return 0;
    }
    VectorDIM colshape_min, colshape_max;
    for(unsigned int i = 0; i < DIM; i++) {
        colshape_min(i) = colshape_min_vec[i];
        colshape_max(i) = colshape_max_vec[i];
    }
    AlignedBox colshape_at_zero(colshape_min, colshape_max);
    shape = std::make_shared<AABBCollisionShape>(colshape_at_zero);

    self_state_publisher = nh.advertise<rlss_ros::RobotState>("self_state", 1);
    collision_shape_publisher = nh.advertise<rlss_ros::AABBCollisionShape>("/other_robot_collision_shapes", 1);

    ros::Subscriber fdssub = nh.subscribe("full_desired_state", 1, fullDesiredStateCallback);

    std::string world_frame;
    nh.getParam("world_frame", world_frame);
    std::string tf_prefix;
    nh.getParam("tf_prefix", tf_prefix);
    world_frame = tf_prefix + "/" + world_frame;
    std::string robot_frame;
    nh.getParam("base_link_frame", robot_frame);

    std::cout << "world_frame: " << world_frame << ", robot_frame: " << robot_frame << std::endl;

    tf::TransformListener listener;

    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();

        try {
            tf::StampedTransform transform;
            listener.lookupTransform(world_frame, robot_frame, ros::Time(0),
                                     transform);
            state[0](0) = transform.getOrigin().x();
            state[0](1) = transform.getOrigin().y();
            state[0](2) = transform.getOrigin().z();
        } catch(...) {
            ROS_WARN_STREAM("tf err");
        }

        publish();

        rate.sleep();
    }

    return 0;
}