#ifndef SKIMAPROSUTILS_HPP
#define SKIMAPROSUTILS_HPP

//ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//OPENCV
#include <opencv2/opencv.hpp>

namespace skimap
{

template <class Voxel3D>
visualization_msgs::Marker createSimpleMapVisualizationMarker(std::string frame_id, ros::Time time, std::string name, float size, int method = visualization_msgs::Marker::ADD)
{
    /**
   * Creating Visualization Marker
   */
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = time;
    marker.action = method;
    marker.id = 0;
    marker.ns = name;

    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    return marker;
}

// template <class HistogramVoxel3D>
// visualization_msgs::Marker createVisualizationMarker(std::string frame_id, ros::Time time, int id, std::vector<HistogramVoxel3D> &voxels, float size = 0.1, int min_weight_th = 1, cv::Scalar col = cv::Scalar(255, 255, 255))
// {

//     /**
//    * Creating Visualization Marker
//    */
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = frame_id;
//     marker.header.stamp = time;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.id = id;

//     marker.type = visualization_msgs::Marker::CUBE_LIST;
//     marker.scale.x = size;
//     marker.scale.y = size;
//     marker.scale.z = size;

//     cv::Mat colorSpace(1, voxels.size(), CV_32FC3);

//     for (int i = 0; i < voxels.size(); i++)
//     {
//         if (voxels[i].data == NULL)
//             continue;
//         if (voxels[i].data->heavierWeight() < min_weight_th)
//             continue;
//         // if (voxels[i].data->w < min_weight_th)
//         //     continue;

//         /**
//      * Create 3D Point from 3D Voxel
//      */
//         geometry_msgs::Point point;
//         point.x = voxels[i].x;
//         point.y = voxels[i].y;
//         point.z = voxels[i].z;

//         /**
//      * Assign Cube Color from Voxel Color
//      */
//         std_msgs::ColorRGBA color;

//         LabelType l = voxels[i].data->heavierLabel();
//         cv::Scalar lc = col;
//         Eigen::Vector3i class_color = dataset.getClass(l).color;

//         lc[0] = class_color(0);
//         lc[1] = class_color(1);
//         lc[2] = class_color(2);

//         // if (l < labels_color_map.size())
//         // {
//         //     lc = labels_color_map[l];
//         // }
//         // else
//         // {
//         //     lc = labels_color_map[l % int(labels_color_map.size())];
//         // }

//         // if (l != 10)
//         // {

//         //   continue;
//         // }
//         //ROS_INFO_STREAM("Voxel " << l << " -> " << lc);

//         color.r = lc[0] / 255.;
//         color.g = lc[1] / 255.;
//         color.b = lc[2] / 255.;
//         color.a = 1;

//         marker.points.push_back(point);
//         marker.colors.push_back(color);
//     }

//     return marker;
// }

} // namespace skimap
#endif /* SKIMAPSERVICECLIENT_HPP */
