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
#include <cstdint>
#include <boost/thread/thread.hpp>
#include <chrono>

//ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/MarkerArray.h>

//OPENCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//Boost
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

//Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/voxels/Voxel6DPose.hpp>
#include <skimap_ros/SkimapIntegrationService.h>

//skimap
typedef skimap::Voxel6DPose VoxelPose;
typedef skimap::SkiMap<VoxelPose, int16_t, float> SKIMAP;
typedef skimap::SkiMap<VoxelPose, int16_t, float>::Voxel3D Voxel3D;
typedef skimap::SkiMap<VoxelPose, int16_t, float>::Tiles2D Tiles2D;
SKIMAP *map;

//Ros
ros::NodeHandle *nh;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
ros::Publisher map_publisher;

//Live parameters
std::string base_frame_name = "world";
std::string target_frame_name = "camera";

/**
 */
struct MapParameters
{
    float map_resolution;
    int min_voxel_weight;
} mapParameters;

/**
 * 
 * @param argc
 * @param argv
 * @return 
 */
int main(int argc, char **argv)
{

    // Initialize ROS
    ros::init(argc, argv, "skimap_map_service");
    nh = new ros::NodeHandle("~");
    tf_listener = new tf::TransformListener();
    tf_broadcaster = new tf::TransformBroadcaster();
    //Frames
    nh->param<std::string>("base_frame_name", base_frame_name, "world");
    nh->param<std::string>("target_frame_name", target_frame_name, "random");
    std::string fitlered_name = target_frame_name + "_fi";
    //Map Publisher
    std::string map_topic = nh->param<std::string>("map_topic", "live_map");
    map_publisher = nh->advertise<visualization_msgs::Marker>(map_topic, 1);

    int hz;
    nh->param<int>("hz", hz, 30);

    //Visualization
    bool auto_publish_markers = false;
    nh->param<bool>("auto_publish_markers", auto_publish_markers, false);

    //SkiMap
    nh->param<float>("map_resolution", mapParameters.map_resolution, 0.05f);
    map = new SKIMAP(mapParameters.map_resolution);

    // Spin & Time
    ros::Rate r(hz);

    // Spin
    while (nh->ok())
    {

        tf::StampedTransform target;
        try
        {
            tf_listener->lookupTransform(base_frame_name, target_frame_name, ros::Time(0), target);
            ROS_INFO("%f", target.getOrigin().x());

            cv::Vec3f trans;
            cv::Vec4f rot;
            trans[0] = target.getOrigin().x();
            trans[1] = target.getOrigin().y();
            trans[2] = target.getOrigin().z();

            rot[0] = target.getRotation().x();
            rot[1] = target.getRotation().y();
            rot[2] = target.getRotation().z();
            rot[3] = target.getRotation().w();

            VoxelPose vp(target_frame_name + "_fi", trans, rot);

            map->integrateVoxel(
                trans[0], trans[1], trans[2],
                &vp);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        std::vector<Voxel3D> voxels;
        map->fetchVoxels(voxels);
        int max_voxel = -1;
        int max_size = -1;
        for (int i = 0; i < voxels.size(); i++)
        {
            Voxel3D &v = voxels[i];
            if (v.data->size() > max_size)
            {
                max_size = v.data->size();
                max_voxel = i;
            }
        }
        if (max_voxel >= 0)
        {
            Voxel3D &v = voxels[max_voxel];
            cv::Vec3f trans;
            cv::Vec4f rot;
            v.data->average(trans, rot);
            ROS_INFO("%f,%f,%f",
                     trans[0],
                     trans[1],
                     trans[2]);

            tf::Vector3 t(trans[0], trans[1], trans[2]);
            tf::Quaternion q(rot[0], rot[1], rot[2], rot[3]);
            tf::Transform filtered;
            filtered.setOrigin(t);
            filtered.setRotation(q);
            tf_broadcaster->sendTransform(
                tf::StampedTransform(filtered, ros::Time::now(),
                                     base_frame_name,
                                     fitlered_name));
        }
        ROS_INFO("Size: %d", int(voxels.size()));

        ros::spinOnce();
        r.sleep();
    }
}
