#include "ros/ros.h"
#include <rlss_ros/OccupancyGrid.h>
#include <rlss/OccupancyGrid.hpp>
#include <fstream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
constexpr unsigned int DIM = DIMENSION;

using OccupancyGrid = rlss::OccupancyGrid<double, DIM>;
using OccIndex = OccupancyGrid::Index;
using OccCoordinate = OccupancyGrid::Coordinate;
using StdVectorVectorDIM = OccupancyGrid::StdVectorVectorDIM;
using VectorDIM = OccupancyGrid::VectorDIM;
using Ellipsoid = rlss::Ellipsoid<double, DIM>;
using MatrixDIMDIM = rlss::internal::MatrixRC<double, DIM, DIM>;

int main(int argc, char **argv) {
    ros::init(argc, argv, "static_map_feeder");
    ros::NodeHandle nh;

    std::string obstacles_directory;
    nh.getParam("obstacles_directory", obstacles_directory);

    std::vector<double> step_size;
    nh.getParam("occupancy_grid_step_size", step_size);
    if(step_size.size() != DIM) {
        ROS_FATAL_STREAM("occupancy grid step size dimension" + std::to_string(step_size.size()) + " does not match dimension " + std::to_string(DIM));
        return 0;
    }
    OccCoordinate occ_step_size;
    for(std::size_t i = 0; i < DIM; i++) {
        occ_step_size(i) = step_size[i];
    }

    ROS_INFO_STREAM(occ_step_size.transpose());
    OccupancyGrid occupancy_grid(occ_step_size);
    boost::filesystem::path p(obstacles_directory);
    std::cout << std::endl;
    for(auto& p: fs::directory_iterator(obstacles_directory)) {
        ROS_INFO_STREAM(p.path().string());
        std::fstream obstacle_file(p.path().string(), std::ios_base::in);
        std::string type;
        obstacle_file >> type;
        if(type == "cvxhull") {
            StdVectorVectorDIM obstacle_pts;
            VectorDIM vec;
            while (obstacle_file >> vec(0)) {
                for(unsigned int d = 1; d < DIM; d++)
                    obstacle_file >> vec(d);
                obstacle_pts.push_back(vec);
            }
            occupancy_grid.addObstacle(obstacle_pts);
        } else if(type == "ellipsoid") {
            VectorDIM center;
            MatrixDIMDIM mtr;

            for(unsigned int i = 0; i < DIM; i++) {
                obstacle_file >> center(i);
            }

            for(unsigned int r = 0; r < DIM; r++) {
                for(unsigned int c = 0; c < DIM; c++) {
                    obstacle_file >> mtr(r, c);
                }
            }
            Ellipsoid ell(center, mtr);
            occupancy_grid.addObstacle(ell);
        }
    }

    ros::Publisher pub = nh.advertise<rlss_ros::OccupancyGrid>("occupancy_grid", 1);

    ros::Rate rate(1);
    while(ros::ok()) {
        rlss_ros::OccupancyGrid msg;

        for(unsigned int i = 0; i < DIM; i++) {
            msg.step_size.push_back(occ_step_size(i));
        }

        const auto& index_set = occupancy_grid.getIndexSet();

        for(const auto& idx: index_set) {
            for(unsigned int i = 0; i < DIM; i++) {
                msg.occupied_indexes.push_back(idx(i));
            }
        }

        pub.publish(msg);
        rate.sleep();
    }

    return 0;
}