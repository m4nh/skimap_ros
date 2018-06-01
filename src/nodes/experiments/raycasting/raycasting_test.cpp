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
#include <cnpy.h>
#include <math.h>

// EIGEN
#include <Eigen/Core>
#include <Eigen/Geometry>

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
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

// Skimap
#include <skimap/voxels/VoxelDataRGBW.hpp>
#include <skimap/voxels/VoxelDataMultiLabel.hpp>
#include <skimap/SkiMap.hpp>
#include <skimap/operators/Raycasting.hpp>

#include <StorageUtils.hpp>

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
typedef double CoordinatesType;
typedef skimap::VoxelDataMultiLabel<20, LabelType, WeightType> VoxelData;

typedef skimap::SkiMap<VoxelData, int16_t, CoordinatesType> SKIMAP;
typedef skimap::SkiMap<VoxelData, int16_t, CoordinatesType>::Voxel3D Voxel3D;

typedef skimap::Raycasting<SKIMAP> Raycaster;
// typedef skimap::Octree<VoxelData, float, 13> SKIMAP;
// typedef skimap::Octree<VoxelData, float, 13>::Voxel3D Voxel3D;

// typedef skimap::InceptionQuadTree<VoxelData, uint16_t, float, 16> SKIMAP;
// typedef skimap::InceptionQuadTree<VoxelData, uint16_t, float, 16>::Voxel3D Voxel3D;

SKIMAP *map;
Raycaster *raycaster;

ros::NodeHandle *nh;

//Globals
float map_resolution = 0.1;
double center_x = 679919.953, center_y = 4931904.674, center_z = 89.178;

Eigen::MatrixXd camera_extrinsics, camera_pose, camera_orientation;
//double center_x = 0, center_y = 0, center_z = 0;

std::vector<std::string> findFilesInFolder(std::string folder, std::string tag, std::string ext, bool ordered = true)
{
    boost::filesystem::path targetDir(folder);
    boost::filesystem::recursive_directory_iterator iter(targetDir), eod;

    std::vector<std::string> files;
    BOOST_FOREACH (boost::filesystem::path const &i, std::make_pair(iter, eod))
    {

        if (is_regular_file(i))
        {
            std::string filename = i.string();
            // std::cout << "File: " << filename << " -> " << tag << " = " << filename.find(tag) << "," << std::string::npos << std::endl;
            if (filename.find(tag) != std::string::npos && filename.find(ext) != std::string::npos)
            {
                files.push_back(filename);
            }
        }
    }
    if (ordered)
        std::sort(files.begin(), files.end());

    return files;
}

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
                          std::vector<Voxel3D> &voxels, float size = 0.1, int min_weight_th = 1, cv::Scalar col = cv::Scalar(255, 255, 255))
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
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;

    cv::Mat colorSpace(1, voxels.size(), CV_32FC3);

    for (int i = 0; i < voxels.size(); i++)
    {
        if (voxels[i].data == NULL)
            continue;
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
        cv::Scalar lc = col;
        // if (l < labels_color_map.size())
        // {
        //     lc = labels_color_map[l];
        // }
        // else
        // {
        //     lc = labels_color_map[l % int(labels_color_map.size())];
        // }

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

void loadFromFile(std::string filename, Eigen::MatrixXd &mat, int cols)
{
    std::ifstream ff(filename);
    std::istream_iterator<double> start(ff), end;
    std::vector<double> numbers(start, end);
    int rows = numbers.size() / cols;
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data(numbers.data(), rows, cols);
    mat = data;
}

namespace Eigen
{
using namespace std;

template <class Matrix>
void write_binary(const char *filename, const Matrix &matrix)
{
    std::ofstream out(filename, ios::out | ios::binary | ios::trunc);
    typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
    out.write((char *)(&rows), sizeof(typename Matrix::Index));
    out.write((char *)(&cols), sizeof(typename Matrix::Index));
    out.write((char *)matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
    out.close();
}
template <class Matrix>
void read_binary(const char *filename, Matrix &matrix)
{
    std::ifstream in(filename, ios::in | std::ios::binary);
    typename Matrix::Index rows = 0, cols = 0;
    in.read((char *)(&rows), sizeof(typename Matrix::Index));
    in.read((char *)(&cols), sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read((char *)matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
    in.close();
}
} // namespace Eigen

void loadTXT(std::string dataset_path, std::vector<Eigen::MatrixXd> &points, bool write_binary = false)
{
    std::vector<std::string> files = findFilesInFolder(dataset_path, "DUCATI", "txt");
    BOOST_FOREACH (std::string f, files)
    {
        ROS_INFO_STREAM("F: " << f);
    }

    for (int i = 0; i < files.size(); i++)
    //for (int i = 0; i < 1; i++)
    {
        std::ifstream ff(files[i]);
        std::istream_iterator<double> start(ff), end;
        std::vector<double> numbers(start, end);

        int rows = numbers.size() / 4;
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data(numbers.data(), rows, 4);

        Eigen::MatrixXd test = data;
        test.transposeInPlace();
        points.push_back(test);

        if (write_binary)
            Eigen::write_binary(std::string(files[i] + ".bin").c_str(), test);
    }
}

void loadRaysLutTXT(std::string rays_lut_path, bool write_binary = false)
{

    std::ifstream ff(rays_lut_path);
    std::istream_iterator<double> start(ff), end;
    std::vector<double> numbers(start, end);

    ROS_INFO_STREAM("RAYS: " << numbers.size() / 8);
    int rows = numbers.size() / 8;
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data(numbers.data(), rows, 8);

    Eigen::MatrixXd test = data;
    if (write_binary)
        Eigen::write_binary(std::string(rays_lut_path + ".bin").c_str(), test);
    // int rows = numbers.size() / 4;
    // Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data(numbers.data(), rows, 4);

    // test.transposeInPlace();
    // points.push_back(test);
}

struct Ray
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    double cx, cy, cz;
    double rx, ry, rz;
    Eigen::Vector4d center;
    Eigen::Vector4d direction;
};
int rows = 2048;
int cols = 2448;
int center_row = rows / 2;
int center_col = cols / 2;
int crop = 300;
Ray *rays = new Ray[rows * cols];

Ray &getRay(int r, int c)
{
    return rays[r * cols + c];
}
void loadRaysLutBinary(std::string rays_lut_path)
{
    siteco::BinaryMatrixData<float> bdata = siteco::BinaryMatrixData<float>::loadFromFile(rays_lut_path);
    Eigen::MatrixXf dataf;
    bdata.getEigenMatrix(dataf);
    Eigen::MatrixXd data = dataf.cast<double>();

    Eigen::Vector4d center;
    int file_rows = data.rows();
    for (int i = 0; i < file_rows; i++)
    {
        if (i == 0)
        {
            center << data(i, 2), data(i, 3), data(i, 4);
            continue;
        }

        int r = data(i, 0);
        int c = data(i, 1);

        if (r < rows && c < cols && r >= 0 && c >= 0)
        {

            Ray &ray = getRay(r, c);
            ray.center = center;
            ray.direction << data(i, 2), data(i, 3), data(i, 4), 1.0;
        }
    }
}

void ladybugConversion(int x, int y, int &u, int &v)
{
    u = rows - x;
    v = y;
}

void ladybugConversionInv(int u, int v, int &x, int &y)
{
    x = rows - u;
    y = v;
}

struct Remap
{
    int u, v;
    Remap()
    {
        set(-1, -1);
    }
    void set(int u, int v)
    {
        this->u = u;
        this->v = v;
    }
    void get(int &u, int &v)
    {
        u = this->u;
        v = this->v;
    }
};

Remap *remap_r2u = new Remap[rows * cols];
Remap *remap_u2r = new Remap[rows * cols];

Remap &getRemap(Remap *&map, int r, int c)
{
    return map[r * cols + c];
}

void rectifyMapLut(std::string rect_lut_path)
{
    siteco::BinaryMatrixData<float> bdata = siteco::BinaryMatrixData<float>::loadFromFile(rect_lut_path);
    Eigen::MatrixXf dataf;
    bdata.getEigenMatrix(dataf);
    Eigen::MatrixXd rectify_lut = dataf.cast<double>();

    for (int i = 0; i < rectify_lut.rows(); i++)
    {

        if (rectify_lut(i, 0) < 0 || rectify_lut(i, 0) >= rows)
            continue;
        if (rectify_lut(i, 1) < 0 || rectify_lut(i, 1) >= cols)
            continue;
        if (rectify_lut(i, 2) < 0 || rectify_lut(i, 2) >= rows)
            continue;
        if (rectify_lut(i, 3) < 0 || rectify_lut(i, 3) >= cols)
            continue;
        int rr = round(rectify_lut(i, 0));
        int rc = round(rectify_lut(i, 1));
        int ur = round(rectify_lut(i, 2));
        int uc = round(rectify_lut(i, 3));
        getRemap(remap_r2u, rr, rc).set(ur, uc);
        getRemap(remap_u2r, ur, uc).set(rr, rc);
    }
}

void loadBinary(std::string dataset_path, std::vector<Eigen::MatrixXd> &points)
{
    std::vector<std::string> files = findFilesInFolder(dataset_path, "DUCATI", ".bin");
    BOOST_FOREACH (std::string f, files)
    {
        ROS_INFO_STREAM("F: " << f);
    }

    //for (int i = 0; i < files.size(); i++)
    for (int i = 0; i < 2; i++)
    {
        Eigen::MatrixXd test;
        Eigen::read_binary(files[i].c_str(), test);

        points.push_back(test);
        ROS_INFO_STREAM("Loaded: " << test.rows() << "," << test.cols());
    }
}

void integrateSkimapSimple(Eigen::MatrixXd &points)
{
    int cols = points.cols();

#pragma omp parallel for schedule(static)
    for (int n = 0; n < cols; n++)
    {
        double x = points(0, n) - center_x;
        double y = points(1, n) - center_y;
        double z = points(2, n) - center_z;
        //printf("Integrate points: %f, %f, %f\n", x, y, z);
        VoxelData voxel(0, 1.0);
        map->integrateVoxel(x, y, z, &voxel);
    }
}

struct Click
{
    int x, y;
} currentClick;

static void onMouse(int event, int x, int y, int, void *)
{

    if (event != cv::EVENT_LBUTTONDOWN)
        return;

    currentClick.x = x;
    currentClick.y = y;

    ROS_INFO_STREAM("Clcik: " << x << "," << y);
}

float numpyIndexConversion(cnpy::NpyArray &arr, int i, int j, int k)
{
    float *loaded_data = arr.data<float>();
    int h = arr.shape[0];
    int w = arr.shape[1];
    int d = arr.shape[2];
    return loaded_data[k + j * d + i * d * w];
}

cv::Mat produceRectifiedImage(cv::Mat &unrectified)
{

    cv::Mat temp;
    cv::rotate(unrectified, temp, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::Mat output = cv::Mat::zeros(temp.size(), CV_8UC3);

    for (int r = 0; r < temp.rows; r++)
    {
        for (int c = 0; c < temp.cols; c++)
        {
            Remap &remap = getRemap(remap_u2r, r, c);
            if (remap.u >= 0 && remap.v >= 0)
                output.at<cv::Vec3b>(remap.u, remap.v) = temp.at<cv::Vec3b>(r, c);
        }
    }
    cv::rotate(output, output, cv::ROTATE_90_CLOCKWISE);

    // }
    return output;
}
/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{

    // DEMO NPY

    // cnpy::NpyArray arr = cnpy::npy_load("/home/daniele/data/datasets/siteco/DucatiEXP/Segmentations/Images_Ladybug0_0/Frame_F_0_logits.npy");
    // ROS_INFO_STREAM("Shape: " << arr.shape[0] << "," << arr.shape[1] << "," << arr.shape[2]);
    // float *loaded_data = arr.data<float>();

    // int h = arr.shape[0];
    // int w = arr.shape[1];
    // int d = arr.shape[2];

    // int i = 393;
    // int j = 432;
    // int k = 10;
    // printf("F: %f\n", loaded_data[k + j * d + i * d * w]);

    // for (i = 0; i < 2 * 2 * 19; i++)
    // {
    //     printf("%d -> %f\n", i, loaded_data[i]);
    // }

    // Initialize ROS
    ros::init(argc, argv, "raycasting_test");
    nh = new ros::NodeHandle("~");

    //PUB SUB
    ros::Publisher map_publisher = nh->advertise<visualization_msgs::Marker>("raycast_map", 1);
    ros::Publisher temp_publisher = nh->advertise<visualization_msgs::Marker>("temp", 1);

    int hz;
    nh->param<int>("hz", hz, 1);

    std::string dataset_path = nh->param<std::string>("dataset_path", "/home/daniele/data/datasets/siteco/DucatiEXP");
    std::string rays_lut_path = nh->param<std::string>("rays_lut_path", "/home/daniele/data/datasets/siteco/DucatiEXP/LadybugData/Ladybug0_0.rays.bin");
    std::string rect_lut_path = nh->param<std::string>("rect_lut_path", "/home/daniele/data/datasets/siteco/DucatiEXP/LadybugData/Ladybug0_0.rectmap.bin");
    std::string camera_extrinsics_path = nh->param<std::string>("camera_extrinsics_path", "/home/daniele/data/datasets/siteco/DucatiEXP/LadybugData/Ladybug0_0.extrinsics.txt");
    std::string image_path_unrect = nh->param<std::string>("image_path_unrect", "/home/daniele/data/datasets/siteco/DucatiEXP/Images_Ladybug0_0/Frame_F_0.jpg");
    std::string image_path = nh->param<std::string>("image_path", "/home/daniele/data/datasets/siteco/DucatiEXP/LadybugData/temp/ladybug_20180426_153744_517612_Rectified_Cam0_1224x1024.bmp");
    std::string imagesegment_path = nh->param<std::string>("imagesegment_path", "/home/daniele/data/datasets/siteco/DucatiEXP/Segmentations/Images_Ladybug0_0/Frame_F_0_colorsegmentation.jpg");
    ROS_INFO_STREAM("Path: " << dataset_path);
    ROS_INFO_STREAM("Path: " << rays_lut_path);

    //Camera extrinsics

    cv::Mat img = cv::imread(image_path);
    cv::Mat imgunrect = cv::imread(image_path_unrect);
    cv::Mat imgsegment = cv::imread(imagesegment_path);

    cv::namedWindow("debug", cv::WINDOW_NORMAL);
    cv::namedWindow("Temp1", cv::WINDOW_NORMAL);
    cv::namedWindow("Temp2", cv::WINDOW_NORMAL);
    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::setMouseCallback("image", onMouse, 0);

    cv::namedWindow("imageunrect", cv::WINDOW_NORMAL);

    loadFromFile(camera_extrinsics_path, camera_extrinsics, 4);
    loadFromFile("/home/daniele/data/datasets/siteco/DucatiEXP/LadybugData/temp/sample_pose.txt", camera_pose, 4);
    camera_orientation = Eigen::Matrix4d::Identity();
    camera_orientation.block(0, 0, 3, 3) = camera_pose.block(0, 0, 3, 3);
    ROS_INFO_STREAM("Camera ext:" << camera_extrinsics.inverse());

    //RectyLUT
    rectifyMapLut(rect_lut_path);

    //RAYS
    //loadRaysLutTXT(rays_lut_path);
    loadRaysLutBinary(rays_lut_path);

    std::vector<Eigen::MatrixXd> points;
    //loadTXT(dataset_path, points);
    loadBinary(dataset_path, points);
    ROS_INFO_STREAM("Chunks: " << points.size());

    map_resolution = nh->param<float>("map_resolution", 0.1);
    ROS_INFO_STREAM("Resolution: " << map_resolution);
    map = new SKIMAP(map_resolution, 0.0);
    map->enableConcurrencyAccess(true);

    for (int i = 0; i < points.size(); i++)
    {
        ROS_INFO_STREAM("Integrating: " << i);
        integrateSkimapSimple(points[i]);
    }

    std::vector<Voxel3D> voxels;
    //map->fetchVoxels(voxels);
    map->radiusSearch(0, 0, 0, 151.0, 151.0, 115.0, voxels);
    ROS_INFO_STREAM("Voxels: " << voxels.size());

    raycaster = new Raycaster(map);

    // Spin & Time
    ros::Rate r(hz);

    Voxel3D *voxel_image = new Voxel3D[rows * cols];

    // Spin
    while (nh->ok())
    {

        int succ = 0;
        // for (int r = center_row - crop; r < center_row + crop; r += 10)
        // {
        //     for (int c = center_col - crop; c < center_col + crop; c += 10)
        //     {
        double max_distance = 60.0;
        cv::Mat depth = cv::Mat::zeros(img.size(), CV_8UC3);
        std::vector<Voxel3D> ray_voxels;
        timings.startTimer("ray");

        Ray &start_ray = getRay(0, 0);
        Eigen::Vector4d center = camera_pose * start_ray.center;
        ROS_INFO_STREAM("Cam: " << camera_pose(0, 3) << "," << camera_pose(1, 3) << "," << camera_pose(2, 3));
        ROS_INFO_STREAM("Ray: " << center(0) << "," << center(1) << "," << center(2));

        int pixel_jumps = 5;
        int radius_count = 0;
        boost::mutex io_mutex;

#pragma omp parallel for schedule(static)
        for (int r = 0; r < rows; r += pixel_jumps)
        {
            for (int c = 500; c < cols - 500; c += pixel_jumps)
            {
                //for (int r = center_row - crop; r < center_row + crop; r += 1)
                //{
                //  for (int c = center_col - crop; c < center_col + crop; c += 1)
                //{
                int x, y;
                ladybugConversionInv(r, c, x, y);
                if (img.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 0, 0))
                    continue;

                Ray &ray = getRay(r, c);
                Eigen::Vector4d direction = camera_orientation * ray.direction;

                Voxel3D v;
                if (raycaster->intersectVoxel(center, direction, v, 0.1, max_distance, 5.0))
                {
                    Eigen::Vector4d voxel_center;
                    voxel_center << v.x, v.y, v.z;
                    Eigen::Vector4d distance_vector = center - voxel_center;
                    double distance = distance_vector.norm();
                    double distance_nom = distance / max_distance;
                    voxel_image[r * cols + c] = v;

                    // std::vector<Voxel3D> radius_voxels;
                    // map->radiusSearch(v.x, v.y, v.z, 0.5, 0.5, 0.5, radius_voxels);
                    // {
                    //     boost::mutex::scoped_lock lock(io_mutex);
                    //     radius_count += radius_voxels.size();
                    // }

                    depth.at<cv::Vec3b>(y, x) = cv::Vec3b(distance_nom * 255, distance_nom * 255, distance_nom * 255);
                    succ++;
                    if (pixel_jumps > 1)
                    {
                        for (int nr = r; nr < r + pixel_jumps; nr++)
                        {
                            for (int nc = c; nc < c + pixel_jumps; nc++)
                            {
                                if (nr == r && nc == c)
                                    continue;
                                Ray &new_ray = getRay(nr, nc);
                                Eigen::Vector4d new_direction = camera_orientation * new_ray.direction;
                                Voxel3D new_v;
                                if (raycaster->intersectVoxel(center, new_direction, new_v, 0.1, distance - map_resolution * 3, distance + map_resolution * 3))
                                {
                                    voxel_center << new_v.x, new_v.y, new_v.z;
                                    distance_vector = center - voxel_center;
                                    double new_distance = distance_vector.norm();
                                    new_distance = new_distance / max_distance;
                                    voxel_image[nr * cols + nc] = new_v;
                                    //printf("PLUS %d -> %d\n", r, nr);
                                    int nx, ny;
                                    ladybugConversionInv(nr, nc, nx, ny);
                                    depth.at<cv::Vec3b>(ny, nx) = cv::Vec3b(new_distance * 255, new_distance * 255, new_distance * 255);
                                }
                            }
                        }
                    }
                }
                else
                {
                    voxel_image[r * cols + c] = Voxel3D();
                }
            }
        }

        ROS_INFO_STREAM("Siccess: " << succ);
        timings.printTime("ray");

        for (int r = 0; r < rows; r += pixel_jumps)
        {
            for (int c = 0; c < cols; c += pixel_jumps)
            {
                //for (int r = center_row - crop; r < center_row + crop; r++)
                //{
                //  for (int c = center_col - crop; c < center_col + crop; c++)
                //{
                if (voxel_image[r * cols + c].data != NULL)
                {
                    ray_voxels.push_back(voxel_image[r * cols + c]);
                }
            }
        }

        //timings.startTimer("postray");
        ROS_INFO_STREAM("Hitted voxels: " << ray_voxels.size());
        // for (int r = 0; r < rows; r += 1)
        // {
        //     for (int c = 0; c < cols; c += 1)
        //     {
        // #pragma omp parallel for schedule(static)
        //         for (int r = center_row - crop; r < center_row + crop; r += 1)
        //         {
        //             for (int c = center_col - crop; c < center_col + crop; c += 1)
        //             {
        //                 int x, y;
        //                 ladybugConversionInv(r, c, x, y);

        //                 if (img.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 0, 0))
        //                     continue;
        //                 float delta = map_resolution;
        //                 Ray &ray = getRay(r, c);
        //                 Eigen::Vector4d direction = camera_orientation * ray.direction;
        //                 skimap::Ray rr(
        //                     skimap::Vector3(center(0), center(1), center(2)),
        //                     skimap::Vector3(direction(0), direction(1), direction(2)));

        //                 for (int v = 0; v < ray_voxels.size(); v++)
        //                 {

        //                     Voxel3D voxel = ray_voxels[v];
        //                     skimap::Box box(
        //                         skimap::Vector3(voxel.x - delta * 0.5, voxel.y - delta * 0.5, voxel.z - delta * 0.5),
        //                         skimap::Vector3(voxel.x + delta * 0.5, voxel.y + delta * 0.5, voxel.z + delta * 0.5));

        //                     // Voxel3D v;
        //                     // v.x = p(0);
        //                     // v.y = p(1);
        //                     // v.z = p(2);
        //                     // v.data = d;
        //                     // voxel = v;
        //                     if (box.intersect(rr, 0, 100.0))
        //                     {
        //                         Eigen::Vector4d voxel_center;
        //                         voxel_center << voxel.x, voxel.y, voxel.z;
        //                         Eigen::Vector4d distance_vector = center - voxel_center;
        //                         double distance = distance_vector.norm();
        //                         distance = distance / max_distance;
        //                         voxel_image[r * cols + c] = voxel;

        //                         depth.at<cv::Vec3b>(y, x) = cv::Vec3b(distance * 255, distance * 255, distance * 255);
        //                         succ++;
        //                     }
        //                     else
        //                     {
        //                         voxel_image[r * cols + c] = Voxel3D();
        //                     }
        //                 }
        //             }
        //         }
        //         timings.printTime("postray");

        cv::Mat output = img.clone();
        cv::Mat output_unrect = imgunrect.clone();
        cv::circle(output, cv::Point(currentClick.x, currentClick.y), 20, cv::Scalar(255, 0, 0), 2);
        cv::circle(output, cv::Point(currentClick.x, currentClick.y), 3, cv::Scalar(255, 0, 0), -1);

        //Unrectified mapped circle
        int clicked_u, clicked_v;
        ladybugConversion(currentClick.x, currentClick.y, clicked_u, clicked_v);
        int clicked_u_unrect, clicked_v_unrect;
        getRemap(remap_r2u, clicked_u, clicked_v).get(clicked_u_unrect, clicked_v_unrect);
        int rx, ry;
        ladybugConversionInv(clicked_u_unrect, clicked_v_unrect, rx, ry);
        cv::circle(output_unrect, cv::Point(rx, ry), 20, cv::Scalar(255, 0, 0), 2);
        cv::circle(output_unrect, cv::Point(rx, ry), 3, cv::Scalar(255, 0, 0), -1);

        cv::Mat rectified = produceRectifiedImage(imgsegment);
        cv::imshow("Temp1", imgsegment);
        cv::imshow("Temp2", rectified);

        //IMshow
        cv::imshow("debug", depth);
        cv::imshow("image", output);
        cv::imshow("imageunrect", output_unrect);
        cv::waitKey(0);

        Eigen::Matrix4d trans;
        trans = Eigen::Matrix4d::Identity();
        trans.block(0, 3, 3, 0) << 1.0, 0, 0;
        Eigen::MatrixXd transX = trans;
        camera_pose = camera_pose * transX;
        camera_orientation = Eigen::Matrix4d::Identity();
        camera_orientation.block(0, 0, 3, 3) = camera_pose.block(0, 0, 3, 3);

        visualization_msgs::Marker rayMarker = createVisualizationMarker("world", ros::Time::now(), 10, ray_voxels, map_resolution * 1.01, 0, cv::Scalar(255, 0, 0));
        visualization_msgs::Marker voxelMarker = createVisualizationMarker("world", ros::Time::now(), 10, voxels, map_resolution, 0);

        map_publisher.publish(voxelMarker);
        temp_publisher.publish(rayMarker);
        ros::spinOnce();
        r.sleep();
    }
}
