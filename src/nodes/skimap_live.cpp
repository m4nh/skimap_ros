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
#include <skimap/voxels/VoxelDataRGBW.hpp>

//skimap
typedef skimap::VoxelDataRGBW<uint16_t, float> VoxelDataColor;
typedef skimap::SkiMap<VoxelDataColor, int16_t, float> SKIMAP;
typedef skimap::SkiMap<VoxelDataColor, int16_t, float>::Voxel3D Voxel3D;
typedef skimap::SkiMap<VoxelDataColor, int16_t, float>::Tiles2D Tiles2D;
SKIMAP *map;

//Ros
ros::NodeHandle *nh;
tf::TransformListener *tf_listener;
ros::Publisher cloud_publisher;
ros::Publisher map_publisher;
ros::Publisher map_2d_publisher;

//Live Cloud
std::string base_frame_name = "slam_map";
std::string camera_frame_name = "camera";
sensor_msgs::PointCloud2 current_live_cloud;

/**
 */
struct MapParameters
{
    float ground_level;
    float agent_height;
    float map_resolution;
    int min_voxel_weight;
    bool enable_chisel;
    bool height_color;
    int chisel_step;
} mapParameters;

/**
 */
struct CameraParameters
{
    double fx, fy, cx, cy;
    int cols, rows;
    double min_distance;
    double max_distance;
    int point_cloud_downscale;
} camera;

/**
 */
struct ColorPoint
{
    cv::Point3f point;
    cv::Vec4b color;
    int w;
};

/**
 */
struct SensorMeasurement
{
    ros::Time stamp;
    std::vector<ColorPoint> points;
    std::vector<ColorPoint> chisel_points;

    void addChiselPoints(CameraParameters &camera, float resolution)
    {
        chisel_points.clear();

#pragma omp parallel
        {
            std::vector<ColorPoint> new_points;

#pragma omp for nowait
            for (int i = 0; i < points.size(); i++)
            {
                cv::Point3f dir = points[i].point - cv::Point3f(0, 0, 0);
                dir = dir * (1 / cv::norm(dir));
                for (float dz = camera.min_distance; dz < points[i].point.z; dz += resolution)
                {
                    cv::Point3f dp = dir * dz;
                    ColorPoint colorPoint;
                    colorPoint.point = dp;
                    colorPoint.w = -mapParameters.chisel_step;
                    new_points.push_back(colorPoint);
                }
            }

#pragma omp critical
            points.insert(points.end(), new_points.begin(), new_points.end());
        }

        //        points.insert(points.end(), chisel_points.begin(), chisel_points.end());
    }
};

std::queue<SensorMeasurement> measurement_queue;
int measurement_queue_max_size = 2;

/**
 */
struct Timings
{
    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::milliseconds ms;
    typedef std::chrono::microseconds us;
    typedef std::chrono::duration<float> fsec;

    std::map<std::string, std::chrono::time_point<std::chrono::system_clock>> times;

    void startTimer(std::string name)
    {
        times[name] = Time::now(); //IS NOT ROS TIME!
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
        ROS_INFO("Time for %s: %f ms", name.c_str(), float(elapsedMicroseconds(name).count()) / 1000.0f);
    }
} timings;

/**
 * Extracts point cloud from RGB-D Frame
 * @param rgb RGB image
 * @param depth Depth image
 * @param camera Camera parameters
 * @param sample_jumps downsample factor 
 * @param output_points OUTPUT vector containing points
 */
void extractPointCloud(cv::Mat rgb, cv::Mat depth, CameraParameters camera, int sample_jumps, std::vector<ColorPoint> &output_points)
{
    sample_jumps = sample_jumps > 1 ? sample_jumps : 1;

    output_points.clear();

    for (float y = 0; y < depth.rows; y += sample_jumps)
    {
        for (float x = 0; x < depth.cols; x += sample_jumps)
        {
            float d = depth.at<float>(y, x);
            ColorPoint cp;
            cp.point.x = (d / camera.fx) * (x - camera.cx);
            cp.point.y = (d / camera.fy) * (y - camera.cy);
            cp.point.z = d;
            if (cp.point.z != cp.point.z)
                continue;
            if (cp.point.z >= camera.min_distance && cp.point.z <= camera.max_distance)
            {
                cp.color = rgb.at<cv::Vec4b>(y, x);
                cp.w = 1;
                output_points.push_back(cp);
            }
        }
    }
}

/**
 * Visualizatoin types for Markers
 */
enum VisualizationType
{
    POINT_CLOUD,
    VOXEL_MAP,
    VOXEL_GRID,
};

/**
 * Creates a "blank" visualization marker with some attributes
 * @param frame_id Base TF Origin for the map points
 * @param time Timestamp for relative message   
 * @param id Unique id for marker identification
 * @param type Type of Marker. 
 * @return 
 */
visualization_msgs::Marker createVisualizationMarker(std::string frame_id, ros::Time time, int id, VisualizationType type)
{

    /**
     * Creating Visualization Marker
     */
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = time;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;

    if (type == VisualizationType::POINT_CLOUD)
    {
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
    }
    else if (type == VisualizationType::VOXEL_MAP)
    {
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.scale.x = mapParameters.map_resolution;
        marker.scale.y = mapParameters.map_resolution;
        marker.scale.z = mapParameters.map_resolution;
    }
    else if (type == VisualizationType::VOXEL_GRID)
    {
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.scale.x = mapParameters.map_resolution;
        marker.scale.y = mapParameters.map_resolution;
        marker.scale.z = mapParameters.map_resolution;
    }
    return marker;
}

/**
 * Creates a Visualization Marker representing a Voxel Map of the environment
 * @param voxels_marker Marker to fill
 * @param voxels 3D Voxel list
 * @param min_weight_th Minimum weight for a voxel to be displayed
 */
void fillVisualizationMarkerWithVoxels(visualization_msgs::Marker &voxels_marker, std::vector<Voxel3D> &voxels, int min_weight_th)
{

    cv::Mat colorSpace(1, voxels.size(), CV_32FC3);
    if (mapParameters.height_color)
    {
        for (int i = 0; i < voxels.size(); i++)
        {
            colorSpace.at<cv::Vec3f>(i)[0] = 180 - (voxels[i].z / 2) * 180;
            colorSpace.at<cv::Vec3f>(i)[1] = 1;
            colorSpace.at<cv::Vec3f>(i)[2] = 1;
        }
        cv::cvtColor(colorSpace, colorSpace, CV_HSV2BGR);
    }

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
        if (mapParameters.height_color)
        {
            color.r = colorSpace.at<cv::Vec3f>(i)[2];
            color.g = colorSpace.at<cv::Vec3f>(i)[1];
            color.b = colorSpace.at<cv::Vec3f>(i)[0];
        }
        else
        {
            color.r = float(voxels[i].data->r) / 255.0;
            color.g = float(voxels[i].data->g) / 255.0;
            color.b = float(voxels[i].data->b) / 255.0;
        }
        color.a = 1;

        voxels_marker.points.push_back(point);
        voxels_marker.colors.push_back(color);
    }
}

/**
 * Fills Visualization Marker with 2D Tiles coming from a 2D Query in SkiMap. 
 * Represent in a black/white chessboard the occupied/free space respectively
 * 
 * @param voxels_marker Marker to fill
 * @param tiles Tiles list
 */
void fillVisualizationMarkerWithTiles(visualization_msgs::Marker &voxels_marker, std::vector<Tiles2D> &tiles)
{
    for (int i = 0; i < tiles.size(); i++)
    {

        /**
         * Create 3D Point from 3D Voxel
         */
        geometry_msgs::Point point;
        point.x = tiles[i].x;
        point.y = tiles[i].y;
        point.z = tiles[i].z;

        /**
         * Assign Cube Color from Voxel Color
         */
        std_msgs::ColorRGBA color;
        if (tiles[i].data != NULL)
        {
            color.r = color.g = color.b = tiles[i].data->w >= mapParameters.min_voxel_weight ? 0.0 : 1.0;
            color.a = 1;
        }
        else
        {
            color.r = color.g = color.b = 1.0;
            color.a = 1;
        }

        voxels_marker.points.push_back(point);
        voxels_marker.colors.push_back(color);
    }
}

/**
 * Fills a Visualization Marker with points coming from a SensorMeasuremetn object. It's used
 * to show the Live Cloud
 * 
 * @param voxels_marker
 * @param measurement
 * @param min_weight_th
 */
void fillVisualizationMarkerWithSensorMeasurement(visualization_msgs::Marker &voxels_marker, SensorMeasurement measurement)
{
    for (int i = 0; i < measurement.points.size(); i++)
    {

        /**
         * Create 3D Point from 3D Voxel
         */
        geometry_msgs::Point point;
        point.x = measurement.points[i].point.x;
        point.y = measurement.points[i].point.y;
        point.z = measurement.points[i].point.z;

        /**
         * Assign Cube Color from Voxel Color
         */
        std_msgs::ColorRGBA color;
        color.r = measurement.points[i].color[2] / 255.0;
        color.g = measurement.points[i].color[1] / 255.0;
        color.b = measurement.points[i].color[0] / 255.0;
        color.a = 1;

        voxels_marker.points.push_back(point);
        voxels_marker.colors.push_back(color);
    }
}

/**
 */
struct IntegrationParameters
{
    std::vector<VoxelDataColor> voxels_to_integrate;
    std::vector<tf::Vector3> poses_to_integrate;
    std::vector<bool> tiles_mask;
    int integration_counter;

    IntegrationParameters()
    {
        integration_counter = 0;
    }
} integrationParameters;

/**
 * Integrates measurements in global Map. Integration is made with OpenMP if possibile so
 * real integration function is surrounded by "startBatchIntegration" and "commitBatchIntegration". By removing
 * these two lines the integration will be launched in single-thread mode
 * @param measurement
 * @param map
 * @param base_to_camera
 */
void integrateMeasurement(SensorMeasurement measurement, SKIMAP *&map, tf::Transform base_to_camera)
{

    if (mapParameters.enable_chisel)
    {
        if (integrationParameters.integration_counter > 0)
            if (integrationParameters.integration_counter % mapParameters.chisel_step == 0)
                measurement.addChiselPoints(camera, mapParameters.map_resolution);
    }

    if (integrationParameters.poses_to_integrate.size() <= measurement.points.size())
    {
        integrationParameters.voxels_to_integrate.resize(measurement.points.size());
        integrationParameters.poses_to_integrate.resize(measurement.points.size());
        integrationParameters.tiles_mask.resize(measurement.points.size());
    }

    for (int i = 0; i < measurement.points.size(); i++)
    {
        integrationParameters.voxels_to_integrate[i].r = measurement.points[i].color[2];
        integrationParameters.voxels_to_integrate[i].g = measurement.points[i].color[1];
        integrationParameters.voxels_to_integrate[i].b = measurement.points[i].color[0];
        integrationParameters.voxels_to_integrate[i].w = measurement.points[i].w;

        float x = measurement.points[i].point.x;
        float y = measurement.points[i].point.y;
        float z = measurement.points[i].point.z;

        tf::Vector3 base_to_point(x, y, z);
        base_to_point = base_to_camera * base_to_point;

        integrationParameters.poses_to_integrate[i] = base_to_point;
        integrationParameters.tiles_mask[i] = base_to_point.z() <= mapParameters.ground_level;
    }

    map->startBatchIntegration();
    for (int i = 0; i < measurement.points.size(); i++)
    {
        tf::Vector3 p = integrationParameters.poses_to_integrate[i];
        map->integrateVoxel(
            float(p.x()),
            float(p.y()),
            float(p.z()),
            &(integrationParameters.voxels_to_integrate[i]));
    }
    map->commitBatchIntegration();

    integrationParameters.integration_counter++;
}

/**
 * RGB + DEPTH callback
 */
void callback(const sensor_msgs::ImageConstPtr &rgb_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{

    tf::StampedTransform base_to_camera;
    try
    {
        tf_listener->lookupTransform(base_frame_name, camera_frame_name, rgb_msg->header.stamp, base_to_camera);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    //RGB
    cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
    cv::cvtColor(rgb, rgb, CV_BGR2BGRA);

    //DEPTH
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //Millimeters to meter conversion
    cv::Mat depth = cv::Mat(rgb.rows, rgb.cols, CV_32FC1);
    const uint16_t *depth_mm_p = cv_ptr->image.ptr<uint16_t>(0);
    float *depth_p = depth.ptr<float>(0);
    int size = rgb.rows * rgb.cols;
    for (int j = 0; j < size; j++)
    {
        depth_p[j] = float(depth_mm_p[j]) / 1000.0f;
    }

    /**
     * Extracts sensor measurement
     */
    SensorMeasurement measurement;
    measurement.stamp = rgb_msg->header.stamp;
    extractPointCloud(rgb, depth, camera, camera.point_cloud_downscale, measurement.points);

    /**
     * Map Integration
     */
    timings.startTimer("Integration");
    integrateMeasurement(measurement, map, base_to_camera);
    timings.printTime("Integration");

    /**
     * 3D Map Publisher
     */
    std::vector<Voxel3D> voxels;
    map->fetchVoxels(voxels);
    visualization_msgs::Marker map_marker = createVisualizationMarker(base_frame_name, rgb_msg->header.stamp, 1, VisualizationType::VOXEL_MAP);
    fillVisualizationMarkerWithVoxels(map_marker, voxels, mapParameters.min_voxel_weight);
    map_publisher.publish(map_marker);

    /**
     * 2D Grid Publisher
     */
    std::vector<Tiles2D> tiles;
    map->fetchTiles(tiles, mapParameters.agent_height);
    visualization_msgs::Marker map_2d_marker = createVisualizationMarker(base_frame_name, rgb_msg->header.stamp, 1, VisualizationType::VOXEL_GRID);
    fillVisualizationMarkerWithTiles(map_2d_marker, tiles);
    map_2d_publisher.publish(map_2d_marker);

    /**
     * Cloud publisher
     */
    visualization_msgs::Marker cloud_marker = createVisualizationMarker(camera_frame_name, rgb_msg->header.stamp, 1, VisualizationType::POINT_CLOUD);
    fillVisualizationMarkerWithSensorMeasurement(cloud_marker, measurement);
    cloud_publisher.publish(cloud_marker);
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
    tf_listener = new tf::TransformListener();

    //Cloud Publisher
    std::string map_cloud_publisher_topic = nh->param<std::string>("map_cloud_publisher_topic", "live_cloud");
    std::string map_topic = nh->param<std::string>("map_publisher_topic", "live_map");
    std::string map_2d_topic = nh->param<std::string>("map_2d_publisher_topic", "live_map_2d");
    cloud_publisher = nh->advertise<visualization_msgs::Marker>(map_cloud_publisher_topic, 1);
    map_publisher = nh->advertise<visualization_msgs::Marker>(map_topic, 1);
    map_2d_publisher = nh->advertise<visualization_msgs::Marker>(map_2d_topic, 1);

    int hz;
    nh->param<int>("hz", hz, 30);

    bool viz;
    nh->param<bool>("viz", viz, true);

    //Camera params
    nh->param<double>("fx", camera.fx, 542.461710);
    nh->param<double>("fy", camera.fy, 543.536535);
    nh->param<double>("cx", camera.cx, 311.081384);
    nh->param<double>("cy", camera.cy, 236.535761);
    nh->param<int>("cols", camera.cols, 640);
    nh->param<int>("rows", camera.rows, 480);
    nh->param<double>("camera_distance_min", camera.min_distance, 0.4);
    nh->param<double>("camera_distance_max", camera.max_distance, 3.0);

    //SkiMap
    nh->param<float>("map_resolution", mapParameters.map_resolution, 0.05f);
    nh->param<float>("ground_level", mapParameters.ground_level, 0.15f);
    nh->param<int>("min_voxel_weight", mapParameters.min_voxel_weight, 10);
    nh->param<bool>("enable_chisel", mapParameters.enable_chisel, false);
    nh->param<bool>("height_color", mapParameters.height_color, false);
    nh->param<int>("chisel_step", mapParameters.chisel_step, 10);
    nh->param<float>("agent_height", mapParameters.agent_height, 1.0f);
    map = new SKIMAP(mapParameters.map_resolution, mapParameters.ground_level);

    //Topics
    std::string camera_rgb_topic, camera_depth_topic;
    nh->param<std::string>("camera_rgb_topic", camera_rgb_topic, "/camera/rgb/image_raw");
    nh->param<std::string>("camera_depth_topic", camera_depth_topic, "/camera/depth/image_raw");
    nh->param<std::string>("base_frame_name", base_frame_name, "slam_map");
    nh->param<std::string>("camera_frame_name", camera_frame_name, "camera");
    nh->param<int>("point_cloud_downscale", camera.point_cloud_downscale, 2);

    //Image/Depth synchronized callbacks
    int camera_queue_size = 100;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxSync;
    message_filters::Subscriber<sensor_msgs::Image> m_rgb_sub(*nh, camera_rgb_topic, camera_queue_size);
    message_filters::Subscriber<sensor_msgs::Image> m_depth_sub(*nh, camera_depth_topic, camera_queue_size);
    message_filters::Synchronizer<ApproxSync> sync(ApproxSync(100), m_rgb_sub, m_depth_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Spin & Time
    ros::Rate r(hz);

    // Spin
    while (nh->ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
