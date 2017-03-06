#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iomanip>

//ROS
#include <ros/ros.h>

#include <skimap/SkipListMap.hpp>

ros::NodeHandle* nh;


/** MAIN NODE **/
int main(int argc, char** argv) {

    // Initialize ROS
    ros::init(argc, argv, "slam_test");
     nh = new ros::NodeHandle("~");

    // Spin
    while (nh->ok()) {

        ros::spinOnce();
    }

}
