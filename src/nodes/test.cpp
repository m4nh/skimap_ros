/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iomanip>

//ROS
#include <ros/ros.h>

#include <skimap/SkipListMap.hpp>

ros::NodeHandle *nh;

/** MAIN NODE **/
int main(int argc, char **argv)
{

    // Initialize ROS
    ros::init(argc, argv, "slam_test");
    nh = new ros::NodeHandle("~");

    // Spin
    while (nh->ok())
    {

        ros::spinOnce();
    }
}
