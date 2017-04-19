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
#include <queue>
#include <boost/thread/thread.hpp>
#include <chrono>

//ROS
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <kdl/frames_io.hpp>
#include <visualization_msgs/MarkerArray.h>

//OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/voxels/VoxelDataOccupancy.hpp>

//skimap
typedef skimap::VoxelDataOccupancy<float> VoxelData;
typedef skimap::SkiMap<VoxelData, int16_t, float> SKIMAP;
typedef skimap::SkiMap<VoxelData, int16_t, float>::Voxel3D Voxel3D;
typedef skimap::SkiMap<VoxelData, int16_t, float>::Tiles2D Tiles2D;
SKIMAP *map;

//Ros
ros::NodeHandle *nh;
tf::TransformListener *tf_listener;
ros::Subscriber cloud_subscriber;
ros::Publisher map_publisher;

//Live Cloud
std::string base_frame_name = "world";
std::string camera_frame_name = "camera";
sensor_msgs::PointCloud2 current_live_cloud;

/**
 */
struct MapServiceParameters
{
    float map_resolution;
    int min_voxel_weight;
    float ground_level;
    float height_color_step;
    float camera_max_z;
    float camera_min_z;
} mapServiceParameters;

/**
 */
struct ColorPoint
{
    cv::Point3f point;
    cv::Vec4b color;
    int w;
};

struct IntegrationPoint
{
    float x, y, z;
    VoxelData voxel_data;
};

/**
 * Creates a "blank" visualization marker with some attributes
 * @param frame_id Base TF Origin for the map points
 * @param time Timestamp for relative message   
 * @param id Unique id for marker identification
 * @param type Type of Marker. 
 * @return 
 */
visualization_msgs::Marker createVisualizationMarker(std::string frame_id, ros::Time time, int id, std::vector<Voxel3D> &voxels, int min_weight_th = 1)
{

    /**
     * Creating Visualization Marker
     */
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = time;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;

    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = mapServiceParameters.map_resolution;
    marker.scale.y = mapServiceParameters.map_resolution;
    marker.scale.z = mapServiceParameters.map_resolution;

    cv::Mat colorSpace(1, voxels.size(), CV_32FC3);
    for (int i = 0; i < voxels.size(); i++)
    {
        colorSpace.at<cv::Vec3f>(i)[0] = 180 - ((voxels[i].z - mapServiceParameters.ground_level) / mapServiceParameters.height_color_step) * 180;
        colorSpace.at<cv::Vec3f>(i)[1] = 1;
        colorSpace.at<cv::Vec3f>(i)[2] = 1;
    }
    cv::cvtColor(colorSpace, colorSpace, CV_HSV2BGR);

    for (int i = 0; i < voxels.size(); i++)
    {

        if (voxels[i].data->w < min_weight_th)
            continue;
        /**
         * Create 3D Point from 3D Voxel
         */
        geometry_msgs::Point point;
        point.x = voxels[i].x;
        point.y = voxels[i].y;
        point.z = voxels[i].z;

        /**
         * Assign Cube Color from Voxel Color
         */
        std_msgs::ColorRGBA color;

        color.r = colorSpace.at<cv::Vec3f>(i)[2];
        color.g = colorSpace.at<cv::Vec3f>(i)[1];
        color.b = colorSpace.at<cv::Vec3f>(i)[0];
        color.a = 1;

        marker.points.push_back(point);
        marker.colors.push_back(color);
    }

    return marker;
}

/**
 * 
 */
void integrateVoxels(std::vector<IntegrationPoint> &integration_points)
{

    map->startBatchIntegration();
    for (int i = 0; i < integration_points.size(); i++)
    {
        IntegrationPoint &ip = integration_points[i];
        map->integrateVoxel(
            float(ip.x),
            float(ip.y),
            float(ip.z),
            &(ip.voxel_data));
    }
    map->commitBatchIntegration();
}

/**
 * Cloud callback
 */
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    tf::StampedTransform base_to_camera;
    try
    {
        tf_listener->lookupTransform(base_frame_name, cloud_msg->header.frame_id, cloud_msg->header.stamp, base_to_camera);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    std::vector<IntegrationPoint> integration_points;
    for (int i = 0; i < temp_cloud->points.size(); i++)
    {

        float x = temp_cloud->points[i].x;
        float y = temp_cloud->points[i].y;
        float z = temp_cloud->points[i].z;

        tf::Vector3 base_to_point(x, y, z);
        base_to_point = base_to_camera * base_to_point;

        IntegrationPoint ip;
        ip.x = base_to_point.x();
        ip.y = base_to_point.y();
        ip.z = base_to_point.z();
        ip.voxel_data.w = 1;
        integration_points.push_back(ip);
    }

    integrateVoxels(integration_points);
}

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

    //Frames
    nh->param<std::string>("base_frame_name", base_frame_name, "world");
    nh->param<std::string>("camera_frame_name", camera_frame_name, "camera");

    //Cloud Publisher
    std::string cloud_topic = nh->param<std::string>("cloud_topic", "live_cloud");
    std::string map_topic = nh->param<std::string>("map_topic", "live_map");
    cloud_subscriber = nh->subscribe(cloud_topic, 1, cloud_callback);
    map_publisher = nh->advertise<visualization_msgs::Marker>(map_topic, 1);

    int hz;
    nh->param<int>("hz", hz, 30);

    //SkiMap
    nh->param<float>("camera_max_z", mapServiceParameters.camera_max_z, 1.5f);
    nh->param<float>("map_resolution", mapServiceParameters.map_resolution, 0.05f);
    nh->param<float>("ground_level", mapServiceParameters.ground_level, 0.15f);
    nh->param<int>("min_voxel_weight", mapServiceParameters.min_voxel_weight, 10);
    nh->param<float>("height_color_step", mapServiceParameters.height_color_step, 0.5f);

    map = new SKIMAP(mapServiceParameters.map_resolution, mapServiceParameters.ground_level);

    // Spin & Time
    ros::Rate r(hz);

    // Spin
    while (nh->ok())
    {
        /**
        * 3D Map Publisher
        */
        std::vector<Voxel3D> voxels;
        map->fetchVoxels(voxels);
        visualization_msgs::Marker map_marker = createVisualizationMarker(base_frame_name, ros::Time::now(), 1, voxels);
        map_publisher.publish(map_marker);

        ros::spinOnce();
        r.sleep();
    }
}
