/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef UTILITY_H
#define UTILITY_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iomanip>
#include <iostream>

#include <fstream>
#include <map>
#include <vector>
#include <kdl/frames_io.hpp>
#include <tf/tf.h>
#include <eigen3/Eigen/Core>
#include "geometry_msgs/Pose.h"

//OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/core/types_c.h"
#include "opencv2/core/version.hpp"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/convex_hull.h>
#include <nav_msgs/Odometry.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

//tf
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class Utility
{
  public:
    Utility();
    Utility(const Utility &orig);
    virtual ~Utility();

    static void kdl_to_eigen_4x4_d(KDL::Frame &frame, Eigen::Matrix4d &mat);
    static void create_kdl_frame(float x, float y, float z, float roll, float pitch, float yaw, KDL::Frame &out_frame);
    static void create_eigen_4x4_d(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Matrix4d &mat);
    static void eigen_4x4_to_geometrypose_d(Eigen::Matrix4d &mat, geometry_msgs::Pose &pose);
    static void eigen_4x4_d_to_tf(Eigen::Matrix4d &t, tf::Transform &tf, bool reverse);
    static void HSVtoRGB(double &r, double &g, double &b, double h, double s, double v);

  private:
};

#endif /* UTILITY_H */
