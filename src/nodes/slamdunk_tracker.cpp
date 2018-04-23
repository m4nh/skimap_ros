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

//ROS
#include <ros/ros.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
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

//Slamdunk
#include <slamdunk/slam_dunk.h>
#include <slamdunk_extension/SlamDunkScene.h>
#include <slamdunk_extension/Utility.h>
#include <boost/thread/thread.hpp>

//Skimap
//#include <skimap/voxels/VoxelDataRGBW.hpp>
#include <skimap/SkipListMap.hpp>

#include "skimap/voxels/VoxelDataRGBW.hpp"

//skimap
bool mapping = true;
float map_resolution = 0.05f;
typedef skimap::VoxelDataRGBW<uint16_t, float> VoxelDataColor;
typedef skimap::SkipListMap<VoxelDataColor, int16_t, float> SkipListMapRGBVolume;
typedef skimap::SkipListMap<VoxelDataColor, int16_t, float>::Voxel3D SkipListMapVoxel3D;
SkipListMapRGBVolume *map_rgb;

//defines
typedef pcl::PointXYZRGB PointType;
using namespace std;

//Ros
ros::NodeHandle *nh;
tf::TransformBroadcaster *broadcaster;
ros::Publisher cloud_publisher;
ros::Publisher map_publisher;
std::map<std::string, int> test_map_id;
visualization_msgs::MarkerArray test_array;
std::vector<VoxelDataColor> voxels_to_integrate;
std::vector<Eigen::Vector4d> poses_to_integrate;
std::vector<bool> voxels_to_integrate_mask;

//Live Cloud
std::string base_frame_name = "slam_map";
std::string camera_frame_name = "camera";
sensor_msgs::PointCloud2 current_live_cloud;

//Timings
double current_time, start_time;
double tracker_start_time;

//SlamDunk
boost::shared_ptr<slamdunk::SlamDunk> slam_dunk;
slamdunk::SlamDunkScene slam_dunk_scene(slam_dunk);

//Params
float fx, fy, cx, cy, cols, rows, depth_scale_factor;

//Poses
Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
bool first_pose_ready = false;
bool first_pose_filtered_ready = false;
bool first_pose_keyframe_ready = false;

//Frame & Buffer

struct BufferTime
{
    int secs;
    int nsecs;

    BufferTime()
    {
        secs = 0;
        nsecs = 0;
    }

    BufferTime(int secs, int nsecs)
    {
        this->secs = secs;
        this->nsecs = nsecs;
    }
};

//Camera
Eigen::Matrix3f inverse_kcam;
cv::Mat current_frame_rgb;
cv::Mat current_frame_depth;
bool rgb_frame_ready = false;
bool depth_frame_ready = false;
slamdunk::RGBDFrame current_frame_rgbd;
const int frame_buffer_size = 2;
int frame_buffer_index = 0;
int frame_ready_to_consume = -1;
bool first_frame_ready = false;
slamdunk::RGBDFrame frame_buffer[frame_buffer_size];
BufferTime frame_buffer_time[frame_buffer_size];
std::queue<slamdunk::RGBDFrame> buffer_queue;
int buffer_queue_max_size = 2;
std::vector<std::string> keyframes_to_update;
int max_deintegration_step = 2;
int max_integration_step = 2;
float camera_distance_max = 3.0f;
float camera_distance_min = 0.4f;

//Locks
boost::mutex buffer_lock;
boost::mutex ktu_mutex;
boost::mutex view_mutex;
boost::mutex ray_mutex;

struct Timings
{
    static std::map<std::string, boost::posix_time::ptime> times;

    static void startTimer(std::string name)
    {
        times[name] = boost::posix_time::microsec_clock::local_time();
    }

    static double elapsedMicroseconds(std::string name)
    {
        boost::posix_time::time_duration duration = boost::posix_time::microsec_clock::local_time() - times[name];
        return duration.total_microseconds() / 1000.0;
    }
};
std::map<std::string, boost::posix_time::ptime> Timings::times;

/**
 * RGB + DEPTH callback
 */
void callback(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth)
{

    //RGB
    current_frame_rgbd.m_color_image = cv_bridge::toCvShare(rgb, "bgr8")->image;
    cv::cvtColor(current_frame_rgbd.m_color_image, current_frame_rgbd.m_color_image, CV_BGR2BGRA);

    //DEPTH
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    current_frame_depth = cv_ptr->image;
    current_frame_rgbd.m_depth_image_mm = cv::Mat(current_frame_depth.rows, current_frame_depth.cols, CV_16UC1);
    current_frame_rgbd.m_depth_image = cv::Mat(current_frame_depth.rows, current_frame_depth.cols, CV_32FC1);
    for (int j = 0; j < current_frame_depth.cols; j++)
    {
        for (int i = 0; i < current_frame_depth.rows; i++)
        {
            current_frame_rgbd.m_depth_image.at<float>(i, j) = (float)(current_frame_depth.at<unsigned short>(i, j) / 1000.0f);
            current_frame_rgbd.m_depth_image_mm.at<unsigned short>(i, j) = (unsigned short)(current_frame_depth.at<unsigned short>(i, j));
        }
    }

    boost::mutex::scoped_lock lock(buffer_lock);
    if (buffer_queue.size() < buffer_queue_max_size)
    {
        current_frame_rgbd.m_timestamp = rgb->header.stamp.nsec * 10e-9;
        current_frame_rgbd.secs = depth->header.stamp.sec;
        current_frame_rgbd.nsecs = depth->header.stamp.nsec;
        buffer_queue.push(current_frame_rgbd);
    }
    lock.unlock();

    first_frame_ready = true;
}

/**
 * Create the FeatureTracke by string naming the Feature Extractor/Detector
 */
void create_feature_tracker(slamdunk::FeatureTrackerParams &ft_params, std::string feature_str)
{

    if (feature_str == "SURF" || feature_str == "surf")
    {
        ft_params.feature_extractor = cv::Ptr<const cv::Feature2D>(new cv::SURF(500, 4, 2, false, true));
    }
    else if (feature_str == "SURFEXT" || feature_str == "surfext")
    {
        ft_params.feature_extractor = cv::Ptr<const cv::Feature2D>(new cv::SURF(500, 4, 2, true, false));
    }
    else if (feature_str == "SIFT" || feature_str == "sift")
    {
        ft_params.feature_extractor = cv::Ptr<const cv::Feature2D>(new cv::SIFT());
    }
    else if (feature_str == "ORB" || feature_str == "orb")
    {
        ft_params.feature_extractor = cv::Ptr<const cv::Feature2D>(new cv::ORB());
        ft_params.feature_matcher = slamdunk::FeatureMatcher::Ptr(new slamdunk::RatioMatcherHamming(false, 0.8, ft_params.cores));
    }
    else if (feature_str == "BRISK" || feature_str == "brisk")
    {
        ft_params.feature_extractor = cv::Ptr<const cv::Feature2D>(new cv::BRISK());
        ft_params.feature_matcher = slamdunk::FeatureMatcher::Ptr(new slamdunk::RatioMatcherHamming(false, 0.8, ft_params.cores));
    }
    else
        std::cout << ">> Features ``" << feature_str << "'' not supported." << std::endl;
}

/**
 * Integrating measurements in global Map
 */
void integrate_cloud_in_map(pcl::PointCloud<PointType>::Ptr &cloud, Eigen::Matrix4d &camera_pose, bool deintegration = false)
{

    //Prepares Voxels and Poses buffers just first time to avoid unuseful realloc
    if (voxels_to_integrate.empty())
    {
        voxels_to_integrate.resize(cloud->points.size());
        poses_to_integrate.resize(cloud->points.size());
        voxels_to_integrate_mask.resize(cloud->points.size());
    }

#pragma omp for nowait
    for (int i = 0; i < cloud->points.size(); i++)
    {
        poses_to_integrate[i] << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1;
        voxels_to_integrate_mask[i] = cloud->points[i].z >= camera_distance_min && cloud->points[i].z <= camera_distance_max;
        poses_to_integrate[i] = camera_pose * poses_to_integrate[i];
        voxels_to_integrate[i].r = cloud->points[i].r;
        voxels_to_integrate[i].g = cloud->points[i].g;
        voxels_to_integrate[i].b = cloud->points[i].b;
        voxels_to_integrate[i].w = deintegration ? -1 : 1;
    }

    for (int i = 0; i < voxels_to_integrate.size(); i++)
    {
        if (voxels_to_integrate_mask[i])
            map_rgb->integrateVoxel(float(poses_to_integrate[i][0]), float(poses_to_integrate[i][1]), float(poses_to_integrate[i][2]), &(voxels_to_integrate[i]));
    }
}

/**
 * Consumes a Cloud. First produces a Live Cloud projection for visualization. If Mapping is enabled integrate measurements in global Map
 * @param key
 * @param cloud 
 */
void consume_cloud(std::string key, pcl::PointCloud<PointType>::Ptr &cloud, bool publish_cloud, bool deintegrate)
{
    Eigen::Matrix4d cam_in_map_pose;
    slam_dunk_scene.getGenericPoseInMapFrame(slam_dunk_scene.poses[key].matrix(), cam_in_map_pose);

    if (!deintegrate)
    {
        if (publish_cloud)
        {
            pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
            pcl::transformPointCloud(*cloud, *cloud_trans, cam_in_map_pose);

            pcl::PCLPointCloud2 cloud2;
            pcl::toPCLPointCloud2(*cloud_trans, cloud2);
            pcl_conversions::fromPCL(cloud2, current_live_cloud);

            current_live_cloud.header.stamp = ros::Time::now(); //(slam_dunk_scene.rgbd_frames[key].secs, slam_dunk_scene.rgbd_frames[key].nsecs);
            current_live_cloud.header.frame_id = base_frame_name;
        }
        if (mapping)
        {
            integrate_cloud_in_map(cloud, cam_in_map_pose);
        }
    }
    else
    {

        if (mapping)
        {
            integrate_cloud_in_map(cloud, cam_in_map_pose);
        }
    }
}

/**
 * SlamDunk Loop. Consumes available RGB-D Frame trying to track camera position w.r.t. previous frames
 */
void slam_dunk_loop()
{

    while (ros::ok())
    {
        if (current_time > tracker_start_time && first_frame_ready)
        {

            boost::mutex::scoped_lock lock(buffer_lock);
            if (buffer_queue.size() <= 0)
                continue;
            //Consume from buffer

            slamdunk::RGBDFrame frame = slamdunk::cloneRGBDFrame(buffer_queue.front());
            buffer_queue.pop();
            lock.unlock();

            //Check if Frame is void //TODO: this must be never true!
            if (frame.m_color_image.empty() || frame.m_depth_image.empty())
                continue;

            //Slamdunk track!
            const int tracked = (*slam_dunk)(frame, current_pose);

            //Update Environment
            if (tracked == slamdunk::SlamDunk::KEYFRAME_DETECTED)
            {

                std::string key = slam_dunk_scene.getKeyframeName(frame.m_timestamp);

                ROS_INFO("New keyframe: %s", key.c_str());

                pcl::PointCloud<PointType>::Ptr cloud = slam_dunk_scene.createCloudFromFrame(frame, fx, fy, cx, cy, 4);
                if (cloud->points.size() <= 0)
                    return;
                slam_dunk_scene.addCloud(key, cloud);
                slam_dunk_scene.addPose(key, current_pose);
                slam_dunk_scene.addRGBDFrame(key, frame);
                slam_dunk_scene.updatePoseHistory(key, current_pose, true);

                if (slam_dunk_scene.poses.size() == 1)
                {
                    slam_dunk_scene.computeSegmentation(key, 30 * M_PI / 180.0, 0.4f);
                }
            }

            if (tracked != slamdunk::SlamDunk::TRACKING_FAILED)
            {
                if (tracked == slamdunk::SlamDunk::KEYFRAME_DETECTED)
                {
                    first_pose_keyframe_ready = true;
                }

                //camera pose
                Eigen::Matrix4d T_MAP_CAM;
                slam_dunk_scene.getGenericPoseInMapFrame(current_pose.matrix(), T_MAP_CAM);

                //Camera pose publisher
                tf::Transform tf_MAP_CAM;
                Utility::eigen_4x4_d_to_tf(T_MAP_CAM, tf_MAP_CAM, false);
                broadcaster->sendTransform(tf::StampedTransform(tf_MAP_CAM, ros::Time(frame.secs, frame.nsecs), base_frame_name, camera_frame_name));
            }
        }
    }
}

/**
 * Creates a Visualization Marker representing a Voxel Map of the environment
 * @param voxels
 * @param frame_id
 * @param time
 * @param size
 * @param id
 * @return 
 */
visualization_msgs::Marker createVisualizationMarkerFromVoxels(std::vector<SkipListMapVoxel3D> &voxels, std::string frame_id, ros::Time time, float size, int id = 0)
{

    /**
     * Creating Visualization Marker
     */
    visualization_msgs::Marker voxels_marker;
    voxels_marker.header.frame_id = frame_id;
    voxels_marker.header.stamp = time;
    voxels_marker.action = visualization_msgs::Marker::ADD;
    voxels_marker.type = visualization_msgs::Marker::CUBE_LIST;
    voxels_marker.id = id;
    voxels_marker.scale.x = size;
    voxels_marker.scale.y = size;
    voxels_marker.scale.z = size;

    for (int i = 0; i < voxels.size(); i++)
    {

        if (voxels[i].data->w < 1)
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
        color.r = float(voxels[i].data->r) / 255.0;
        color.g = float(voxels[i].data->g) / 255.0;
        color.b = float(voxels[i].data->b) / 255.0;
        color.a = 1;

        voxels_marker.points.push_back(point);
        voxels_marker.colors.push_back(color);
    }
    return voxels_marker;
}

/**
 * Publish Live Tracked Cloud and Map if any
 */
void publish_cloud_markers()
{

    cloud_publisher.publish(current_live_cloud);

    if (mapping)
    {
        std::vector<SkipListMapVoxel3D> voxels;
        map_rgb->fetchVoxels(voxels);
        visualization_msgs::Marker map_marker = createVisualizationMarkerFromVoxels(voxels, base_frame_name, ros::Time::now(), map_resolution);
        map_publisher.publish(map_marker);
    }
}

/**
 * Callback called when New Entry is available for integration in Map
 * @param new_entries
 */
void scene_new_entries_callback(std::vector<slamdunk::ConsumingEntry> &new_entries)
{
    std::string key;
    for (int i = 0; i < new_entries.size(); i++)
    {
        key = new_entries[i].key;
        if (!new_entries[i].replacement)
        {
            consume_cloud(key, slam_dunk_scene.clouds[key], true, false);
        }
    }
}

/**
 * Callback called when an Old Entry is updated and ready to be Deintegrated and Integrated again
 * @param optimized_entries
 */
void scene_optimized_entries_callback(std::vector<slamdunk::ConsumingEntry> &optimized_entries)
{
    std::string key;
    for (int i = 0; i < optimized_entries.size(); i++)
    {
        key = optimized_entries[i].key;
        if (optimized_entries[i].replacement)
        {
            consume_cloud(key, slam_dunk_scene.clouds[key], false, true);
            slam_dunk_scene.addPose(optimized_entries[i].key, optimized_entries[i].new_pose);
            consume_cloud(key, slam_dunk_scene.clouds[key], false, false);
        }
    }
}

/**
 * Checks for Optimized frames. Forces Scene to produce new entries and optimizations
 */
void check_optimized_frames()
{

    while (ros::ok())
    {
        slam_dunk_scene.spinIntegrationUpdate();
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
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
    ros::init(argc, argv, "skamdunk_tracker");
    nh = new ros::NodeHandle("~");
    broadcaster = new tf::TransformBroadcaster();

    //Cloud Publisher
    std::string map_cloud_publisher_topic = nh->param<std::string>("map_cloud_publisher_topic", "live_cloud");
    cloud_publisher = nh->advertise<sensor_msgs::PointCloud2>(map_cloud_publisher_topic, 1);

    int hz;
    nh->param<int>("hz", hz, 30);

    int cores;
    nh->param<int>("cores", cores, 4);

    bool viz;
    nh->param<bool>("viz", viz, true);
    nh->param<double>("tracker_start_time", tracker_start_time, 5);

    //Camera params
    nh->param<float>("fx", fx, 542.461710f);
    nh->param<float>("fy", fy, 543.536535f);
    nh->param<float>("cx", cx, 311.081384f);
    nh->param<float>("cy", cy, 236.535761f);
    nh->param<float>("cols", cols, 640);
    nh->param<float>("rows", rows, 480);
    nh->param<float>("depth_scale_factor", depth_scale_factor, 0.0002);
    nh->param<int>("max_deintegration_step", max_deintegration_step, 1);
    nh->param<int>("max_integration_step", max_integration_step, 5);

    inverse_kcam = Eigen::Matrix3f::Identity();
    inverse_kcam(0, 0) = 1.f / fx;
    inverse_kcam(1, 1) = 1.f / fy;
    inverse_kcam(0, 2) = cx * (-1.f / fx);
    inverse_kcam(1, 2) = cy * (-1.f / fy);

    //General parameters
    slamdunk::SlamDunkParams sd_params;
    sd_params.cores = cores;

    nh->param<int>("rings", (int &)sd_params.rba_rings, 3);
    nh->param<float>("kf_overlapping", sd_params.kf_overlapping, 0.8f);
    nh->param<bool>("loop_inference", sd_params.try_loop_inference, false);
    nh->param<bool>("doicp", sd_params.doicp, false);
    nh->param<float>("icp_distance", sd_params.icp_distance_th, 0.2f);
    nh->param<float>("icp_normal", sd_params.icp_normal_th, 30.0f);
    nh->param<bool>("verbose", sd_params.verbose, false);

    //Feature tracker parameters
    slamdunk::FeatureTrackerParams ft_params(cores);
    std::string feature_str;
    nh->param<std::string>("features", feature_str, "surfext");
    create_feature_tracker(ft_params, feature_str);
    nh->param<double>("winl", ft_params.active_win_length, 6.0f);
    nh->param<bool>("feat_redux", ft_params.frustum_feature_reduction, false);
    ft_params.verbose = sd_params.verbose;
    sd_params.tracker.reset(new slamdunk::FeatureTracker(inverse_kcam, cols, rows, ft_params));

    //SlamDunk
    slam_dunk.reset(new slamdunk::SlamDunk(inverse_kcam, sd_params));
    slam_dunk_scene.setSlamDunkHandle(slam_dunk);
    slam_dunk_scene.max_spin_integration_steps = max_integration_step;
    slam_dunk_scene.max_spin_deintegration_steps = max_deintegration_step;
    slam_dunk_scene.addNewEntryCallaback(scene_new_entries_callback);
    slam_dunk_scene.addOptimezedEntryCallabck(scene_optimized_entries_callback);

    //SKimap
    mapping = nh->param<bool>("mapping", false);
    map_resolution = nh->param<float>("map_resolution", 0.05f);
    map_rgb = new SkipListMapRGBVolume(-32000, 32000, map_resolution, map_resolution, map_resolution);
    map_publisher = nh->advertise<visualization_msgs::Marker>("slam_map", 1);
    camera_distance_max = nh->param<float>("camera_distance_max", 3.0f);
    camera_distance_min = nh->param<float>("camera_distance_min", 0.4f);

    //Topics
    std::string camera_rgb_topic, camera_depth_topic;
    nh->param<std::string>("camera_rgb_topic", camera_rgb_topic, "/camera/rgb/image_raw");
    nh->param<std::string>("camera_depth_topic", camera_depth_topic, "/camera/depth/image_raw");
    nh->param<std::string>("base_frame_name", base_frame_name, "slam_map");
    nh->param<std::string>("camera_frame_name", camera_frame_name, "camera");

    //Image/Depth synchronized callbacks
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxSync;
    message_filters::Subscriber<sensor_msgs::Image> m_rgb_sub(*nh, camera_rgb_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> m_depth_sub(*nh, camera_depth_topic, 1);
    message_filters::Synchronizer<ApproxSync> sync(ApproxSync(100), m_rgb_sub, m_depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Spin & Time
    ros::Rate r(hz);
    start_time = ros::Time::now().toSec();
    current_time = ros::Time::now().toSec() - start_time;

    //Threads
    boost::thread slam_dunk_thread(slam_dunk_loop);
    boost::thread opt_thread(check_optimized_frames);

    //Named
    if (viz)
    {
        cv::namedWindow("ray", CV_WINDOW_FREERATIO);
        cv::namedWindow("rgb", CV_WINDOW_FREERATIO);
    }

    double start_time = ros::Time::now().toSec();

    // Spin
    while (nh->ok())
    {

        current_time = ros::Time::now().toSec() - start_time;

        publish_cloud_markers();

        //Imshow
        if (first_frame_ready && viz)
        {
            boost::mutex::scoped_lock vis_lock(ray_mutex);
            cv::imshow("rgb", current_frame_rgbd.m_color_image);
            cv::imshow("depth", current_frame_rgbd.m_depth_image);
        }

        //Wait key
        char c = cv::waitKey(10);
        if (c == 113)
            ros::shutdown();

        ros::spinOnce();
        r.sleep();
    }

    slam_dunk_thread.join();
    opt_thread.join();
}
