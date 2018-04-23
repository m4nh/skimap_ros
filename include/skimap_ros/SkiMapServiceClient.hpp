/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SKIMAPSERVICECLIENT_HPP
#define SKIMAPSERVICECLIENT_HPP

#include <fstream>
#include <vector>
#include <queue>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <thread>
#include <mutex>

#include <skimap_ros/SkimapIntegrationService.h>
#include <skimap_ros/SkiMapServiceClient.hpp>

namespace skimap_ros
{

class SkimapServiceClient
{
public:
  /**
 */
  struct ColorPoint
  {

    double x, y, z;
    float r, g, b;
    float w;
    bool valid;

    ColorPoint()
    {
      this->valid = false;
    }
  };

  /**
   *
   */
  SkimapServiceClient(ros::NodeHandle *node_handle, std::string service_name = "/skimap_map_service/integration_service") : service_name(service_name)
  {

    this->node_handle = node_handle;
    this->service_client = this->node_handle->serviceClient<skimap_ros::SkimapIntegrationService>(service_name);
    consuming_thread = new std::thread(&SkimapServiceClient::internalDequeuer, this);
  }

  virtual ~SkimapServiceClient()
  {
    delete consuming_thread;
  }

  /**
   * Sends point to Remote Service for integration
   */
  void integratePoints(std::vector<SkimapServiceClient::ColorPoint> &output_points, geometry_msgs::Pose camera_pose)
  {
    IntegrationEntry entry;
    entry.camera_pose = camera_pose;
    entry.points.insert(entry.points.end(), output_points.begin(), output_points.end());
    std::lock_guard<std::mutex> guard(this->queue_mutex);
    this->integration_queue.push(entry);
  }

  /**
   * Sends point to Remote Service for integration
   */
  void integratePoints(std::vector<SkimapServiceClient::ColorPoint> &output_points, tf::Transform &camera_pose)
  {
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(camera_pose, pose);
    this->integratePoints(output_points, pose);
  }

protected:
  std::string service_name;
  ros::NodeHandle *node_handle;
  ros::ServiceClient service_client;

  //Queue
  std::queue<IntegrationEntry> integration_queue;
  std::mutex queue_mutex;
  std::thread *consuming_thread;

  /**
   *
   */
  void integrateEntry(IntegrationEntry &entry)
  {
    skimap_ros::SkimapIntegrationService srv;
    srv.request.sensor_pose = entry.camera_pose;

    for (int i = 0; i < entry.points.size(); i++)
    {
      SkimapServiceClient::ColorPoint p = entry.points[i];
      if (p.valid)
      {
        geometry_msgs::Point mp;
        mp.x = p.x;
        mp.y = p.y;
        mp.z = p.z;

        std_msgs::ColorRGBA color;
        color.r = p.r;
        color.g = p.g;
        color.b = p.b;
        color.a = 1;
        srv.request.points.push_back(mp);
        srv.request.colors.push_back(color);
        srv.request.weights.push_back(p.w);
      }
    }

    if (service_client.call(srv))
    {
      //Nothing
    }
    else
    {
      ROS_ERROR("SkimapServiceClient: Failed to call service add_two_ints");
    }
  }

  /**
   *
   */
  void internalDequeuer()
  {
    while (this->node_handle->ok())
    {

      if (integration_queue.size() > 0)
      {
        std::unique_lock<std::mutex> guard(this->queue_mutex);
        IntegrationEntry integration_entry = integration_queue.front();
        integration_queue.pop();
        guard.unlock();
        integrateEntry(integration_entry);
      }
    }
  }
};
}

#endif /* SKIMAPSERVICECLIENT_HPP */
