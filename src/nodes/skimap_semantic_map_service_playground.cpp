/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// OPENCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

// Boost
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/OctreeMap.hpp>
#include <skimap/InceptionQuadTree.hpp>
#include <skimap/voxels/VoxelDataRGBW.hpp>
#include <skimap/voxels/VoxelDataMultiLabel.hpp>
#include <skimap_ros/SkimapIntegrationService.h>
#include <skimap_ros/SemanticService.h>

std::vector<cv::Scalar> labels_color_map = {
    cv::Scalar(128, 64, 128),
    cv::Scalar(244, 35, 232),
    cv::Scalar(70, 70, 70),
    cv::Scalar(102, 102, 156),
    cv::Scalar(190, 153, 153),
    cv::Scalar(153, 153, 153),
    cv::Scalar(250, 170, 30),
    cv::Scalar(220, 220, 0),
    cv::Scalar(107, 142, 35),
    cv::Scalar(152, 251, 152),
    cv::Scalar(70, 130, 180),
    cv::Scalar(220, 20, 60),
    cv::Scalar(255, 0, 0),
    cv::Scalar(0, 0, 142),
    cv::Scalar(0, 0, 70),
    cv::Scalar(0, 60, 100),
    cv::Scalar(0, 80, 100),
    cv::Scalar(0, 0, 230),
    cv::Scalar(119, 11, 32)};

/**
 */
struct Timings
{
  typedef std::chrono::high_resolution_clock Time;
  typedef std::chrono::milliseconds ms;
  typedef std::chrono::microseconds us;
  typedef std::chrono::duration<float> fsec;

  std::map<std::string, std::chrono::time_point<std::chrono::system_clock>>
      times;

  void startTimer(std::string name)
  {
    times[name] = Time::now(); // IS NOT ROS TIME!
  }

  us elapsedMicroseconds(std::string name)
  {
    fsec elaps = Time::now() - times[name];
    return std::chrono::duration_cast<us>(elaps);
  }

  ms elapsedMilliseconds(std::string name)
  {
    fsec elaps = Time::now() - times[name];
    return std::chrono::duration_cast<ms>(elaps);
  }

  void printTime(std::string name)
  {
    ROS_INFO("Time for %s: %f ms", name.c_str(),
             float(elapsedMicroseconds(name).count()) / 1000.0f);
  }
} timings;

// skimap
#define INVALID_LABEL 255
typedef uint16_t LabelType;
typedef uint16_t WeightType;
typedef skimap::VoxelDataMultiLabel<20, LabelType, WeightType> VoxelDataLabels;

typedef skimap::SkiMap<VoxelDataLabels, int16_t, float> SKIMAP;
typedef skimap::SkiMap<VoxelDataLabels, int16_t, float>::Voxel3D Voxel3D;

// typedef skimap::Octree<VoxelDataLabels, float, 13> SKIMAP;
// typedef skimap::Octree<VoxelDataLabels, float, 13>::Voxel3D Voxel3D;

// typedef skimap::InceptionQuadTree<VoxelDataLabels, uint16_t, float, 16> SKIMAP;
// typedef skimap::InceptionQuadTree<VoxelDataLabels, uint16_t, float, 16>::Voxel3D Voxel3D;

SKIMAP *map;

// Ros
ros::NodeHandle *nh;
tf::TransformListener *tf_listener;
ros::Publisher map_publisher;

// Live parameters
std::string base_frame_name = "world";
std::string camera_frame_name = "camera";

/**
 */
struct map_service_parameters
{
  float map_resolution;
  int min_voxel_weight;
  float ground_level;
  float height_color_step;
  bool height_color_enabled;
  float camera_max_z;
  float camera_min_z;
  std::string height_axis;
} map_service_parameters;

/**
 */
struct MapSynchManager
{
  boost::mutex map_mutex;
} map_synch_manager;

/**
 */
struct ColorPoint
{
  cv::Point3f point;
  LabelType label;
  WeightType w;
};

/**
 */
struct IntegrationPoint
{
  bool valid;
  float x, y, z;
  VoxelDataLabels voxel_data;
  IntegrationPoint()
  {
    valid = false;
  }
};

/**
 * Creates a "blank" visualization marker with some attributes
 * @param frame_id Base TF Origin for the map points
 * @param time Timestamp for relative message
 * @param id Unique id for marker identification
 * @param type Type of Marker.
 * @return
 */
visualization_msgs::Marker
createVisualizationMarker(std::string frame_id, ros::Time time, int id,
                          std::vector<Voxel3D> &voxels, int min_weight_th = 1)
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
  marker.scale.x = map_service_parameters.map_resolution;
  marker.scale.y = map_service_parameters.map_resolution;
  marker.scale.z = map_service_parameters.map_resolution;

  cv::Mat colorSpace(1, voxels.size(), CV_32FC3);

  for (int i = 0; i < voxels.size(); i++)
  {

    if (voxels[i].data->heavierWeight() < min_weight_th)
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

    LabelType l = voxels[i].data->heavierLabel();
    cv::Scalar lc(0, 0, 0);
    if (l < labels_color_map.size())
    {
      lc = labels_color_map[l];
    }

    // if (l != 10)
    // {

    //   continue;
    // }
    //ROS_INFO_STREAM("Voxel " << l << " -> " << lc);

    color.r = lc[0] / 255.;
    color.g = lc[1] / 255.;
    color.b = lc[2] / 255.;
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
  boost::mutex::scoped_lock lock(map_synch_manager.map_mutex);

  map->enableConcurrencyAccess(true);

#pragma omp parallel for
  for (int i = 0; i < integration_points.size(); i++)
  {
    IntegrationPoint &ip = integration_points[i];
    if (!ip.valid)
      continue;
    map->integrateVoxel(float(ip.x), float(ip.y), float(ip.z),
                        &(ip.voxel_data));
  }
}

/**
 * Integration Service callback
 */
bool integration_service_callback(
    skimap_ros::SemanticService::Request &req,
    skimap_ros::SemanticService::Response &res)
{

  timings.startTimer("Prepare");
  tf::Transform base_to_camera = tf::Transform(
      tf::Quaternion(
          req.sensor_pose.orientation.x, req.sensor_pose.orientation.y,
          req.sensor_pose.orientation.z, req.sensor_pose.orientation.w),
      tf::Vector3(req.sensor_pose.position.x, req.sensor_pose.position.y,
                  req.sensor_pose.position.z));

  tf::Quaternion q = base_to_camera.getRotation();
  tf::Vector3 v = base_to_camera.getOrigin();
  std::vector<IntegrationPoint> integration_points(req.points.size());
  std::vector<int> integration_counter(req.points.size());

  ROS_INFO_STREAM("POSE: " << v.x() << "," << v.y() << "," << v.z());
  if (!(req.labels.size() == req.points.size() && req.points.size() == req.weights.size()))
  {
    ROS_ERROR_STREAM("Request invalid! Number of Points doesn't match with number of Labels! (or even Weights!)");
  }

#pragma omp parallel for
  for (int i = 0; i < req.points.size(); i++)
  {

    double x = req.points[i].x;
    double y = req.points[i].y;
    double z = req.points[i].z;
    if (z < map_service_parameters.camera_min_z)
      continue;
    if (req.labels[i] == 255)
      continue;
    tf::Vector3 base_to_point(x, y, z);
    base_to_point = base_to_camera * base_to_point;

    IntegrationPoint ip;
    ip.x = base_to_point.x();
    ip.y = base_to_point.y();
    ip.z = base_to_point.z();

    // if (req.labels[i] != 10)
    // {
    //   continue;
    // }
    // ROS_INFO_STREAM("Integrating label " << req.labels[i] << " -> " << req.weights[i]);

    ip.voxel_data = VoxelDataLabels(req.labels[i], req.weights[i]);

    ip.valid = true;
    integration_points[i] = ip;
    integration_counter[i] = 1.0;
  }

  int sum = std::accumulate(integration_counter.begin(), integration_counter.end(), 0);

  timings.printTime("Prepare");

  timings.startTimer("Integration");

  integrateVoxels(integration_points);
  timings.printTime("Integration");
  res.integrated_points = int(integration_points.size());
  res.status = skimap_ros::SkimapIntegrationService::Response::STATUS_OK;

  ROS_INFO("Request integration of %d / %d points.", sum, int(integration_points.size()));
  return true;
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
  ros::init(argc, argv, "skimap_semantic_map_service_playground");
  nh = new ros::NodeHandle("~");
  tf_listener = new tf::TransformListener();

  // Frames
  nh->param<std::string>("base_frame_name", base_frame_name, "camera");
  nh->param<std::string>("camera_frame_name", camera_frame_name, "camera_rf");

  // Map Publisher
  std::string map_topic = nh->param<std::string>("map_topic", "live_map");
  map_publisher = nh->advertise<visualization_msgs::Marker>(map_topic, 1);

  // Services
  ros::ServiceServer service_integration =
      nh->advertiseService("semantic_integration_service", integration_service_callback);

  int hz;
  nh->param<int>("hz", hz, 30);

  // Visualization
  bool auto_publish_markers = false;
  nh->param<bool>("auto_publish_markers", auto_publish_markers, false);

  // SkiMap
  nh->param<float>("camera_max_z", map_service_parameters.camera_max_z, 1.5f);
  nh->param<float>("map_resolution", map_service_parameters.map_resolution, 0.15f);
  nh->param<float>("ground_level", map_service_parameters.ground_level, 0.00f);
  nh->param<float>("camera_min_z", map_service_parameters.camera_min_z, 0.01f);
  nh->param<int>("min_voxel_weight", map_service_parameters.min_voxel_weight, 1);
  nh->param<float>("height_color_step", map_service_parameters.height_color_step, 0.5f);
  nh->param<bool>("height_color", map_service_parameters.height_color_enabled, false);
  nh->param<std::string>("height_axis", map_service_parameters.height_axis, "z");

  map = new SKIMAP(map_service_parameters.map_resolution, map_service_parameters.ground_level);

  // Spin & Time
  ros::Rate r(hz);

  // Spin
  while (nh->ok())
  {
    /**
    * 3D Map Publisher
    */
    if (auto_publish_markers)
    {

      std::vector<Voxel3D> voxels;
      {
        boost::mutex::scoped_lock lock(map_synch_manager.map_mutex);
        map->fetchVoxels(voxels);
      }
      visualization_msgs::Marker map_marker = createVisualizationMarker(
          base_frame_name, ros::Time::now(),
          1, voxels, map_service_parameters.min_voxel_weight);
      map_publisher.publish(map_marker);
    }

    ros::spinOnce();
    r.sleep();
  }
}
