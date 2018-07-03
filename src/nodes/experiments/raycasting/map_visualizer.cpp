

#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cnpy.h>
#include <math.h>

// EIGEN
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/CXX11/Tensor>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// OPENCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

// Boost
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

// Skimap
#include <skimap/voxels/VoxelDataRGBW.hpp>
#include <skimap/voxels/VoxelDataMultiLabel.hpp>
#include <skimap/voxels/VoxelDataMultiLabelWithCounter.hpp>
#include <skimap/SkiMap.hpp>
#include <skimap/operators/Raycasting.hpp>
#include <skimap_ros/SkimapRosUtils.hpp>

#include <siteco/Geometry.hpp>
#include <siteco/StorageUtils.hpp>
#include <siteco/LadybugRawStream.hpp>
#include <siteco/LadybugFrames.hpp>
#include <siteco/StorageUtils.hpp>
#include <siteco/LadybugCameras.hpp>
#include <siteco/SegmentationDatasets.hpp>

// skimap
#define INVALID_LABEL 255
typedef uint16_t LabelType;
typedef uint16_t WeightType;
typedef double CoordinatesType;
//typedef skimap::VoxelDataMultiLabel<20, LabelType, WeightType> VoxelData;
typedef skimap::VoxelDataMultiLabelWithCounter<20, LabelType, WeightType> VoxelData;
//typedef skimap::VoxelDataRGBW<uint8_t, float> VoxelData;

typedef skimap::SkiMap<VoxelData, int16_t, CoordinatesType> SKIMAP;
typedef skimap::SkiMap<VoxelData, int16_t, CoordinatesType>::Voxel3D Voxel3D;

SKIMAP *map;
ros::NodeHandle *nh;
siteco::SegmentationDataset dataset;

/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{

    // Initialize ROS
    ros::init(argc, argv, "raycasting_test");
    nh = new ros::NodeHandle("~");

    // Topics
    ros::Publisher map_publisher = nh->advertise<visualization_msgs::Marker>("map_visualizer", 1);

    // Dataset
    dataset = siteco::SegmentationDataset("/home/daniele/Desktop/cityscapes.json");

    // Map
    map = new SKIMAP(0.1, 0.0);
    map->loadFromFile("/tmp/map_test.skimap");

    // Query
    std::vector<Voxel3D> voxels;
    map->radiusSearch(0.0, 0.0, 0.0, 100.0, 100.0, 100.0, voxels);
    ROS_INFO_STREAM("Search: " << voxels.size());

    // Visualization marker
    visualization_msgs::Marker marker = skimap::createSimpleMapVisualizationMarker<Voxel3D>("world", ros::Time::now(), "map", 0.2);
    for (int i = 0; i < voxels.size(); i++)
    {
        Voxel3D &voxel = voxels[i];

        geometry_msgs::Point point;
        point.x = voxels[i].x;
        point.y = voxels[i].y;
        point.z = voxels[i].z;

        std_msgs::ColorRGBA color;

        LabelType l = voxels[i].data->heavierLabel();
        Eigen::Vector3i class_color = dataset.getClass(l).color;

        color.r = class_color(0) / 255.;
        color.g = class_color(1) / 255.;
        color.b = class_color(2) / 255.;
        color.a = 1;

        marker.points.push_back(point);
        marker.colors.push_back(color);
    }

    while (ros::ok())
    {
        marker.header.stamp = ros::Time::now();
        map_publisher.publish(marker);
    }
}