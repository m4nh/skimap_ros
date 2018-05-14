/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

//BOOST
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

// OPENCV
#include <opencv2/opencv.hpp>

// ROS
#include <geometry_msgs/Pose.h>

// Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/OctreeMap.hpp>
#include <skimap/voxels/VoxelDataRGBW.hpp>

#include <skimap_ros/SkimapIntegrationService.h>
#include <skimap_ros/SkiMapServiceClient.hpp>

#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#define MAX_RANDOM_COORD 10.0
#define MIN_RANDOM_COORD -10.0
#define MAX_RANDOM_COLOR 1.0
#define MIN_RANDOM_COLOR 0.0

#define ALGO_SKIMAP 1
#define ALGO_OCTREE 2

#define ALGO ALGO_SKIMAP

#define MAP_RESOLUTION 0.01
#define MAX_SIZE 655.0
#define OCTREE_SPLITS 16 // MUST BE LOG(MAX_SIZE / MAP_RESOLUTION)

struct GenericDataset
{
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses;
    std::vector<Eigen::MatrixXf, Eigen::aligned_allocator<Eigen::MatrixXf>> points;

    std::vector<std::string> findFilesInFolder(std::string folder, std::string tag, bool ordered = true)
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
                if (filename.find(tag) != std::string::npos)
                {
                    files.push_back(filename);
                }
            }
        }
        if (ordered)
            std::sort(files.begin(), files.end());

        return files;
    }
};

struct FreiburgDataset : public GenericDataset
{

    FreiburgDataset(std::string path)
    {

        std::vector<std::string> points_files = findFilesInFolder(path, "points.dat");
        std::vector<std::string> poses_files = findFilesInFolder(path, "Poses.dat");

        for (int i = 0; i < poses_files.size(); i++)
        {
            std::fstream f(poses_files[i]);
            float x, y, z, rx, ry, rz;
            f >> x;
            f >> y;
            f >> z;
            f >> rx;
            f >> ry;
            f >> rz;
            //sECOND ROW
            f >> x;
            f >> y;
            f >> z;
            f >> rx;
            f >> ry;
            f >> rz;
            std::cout << x << "," << y << "," << z << "," << rx << "," << ry << "," << rz << "\n";
            Eigen::Matrix3f rot;
            rot = Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ());
            Eigen::Matrix4f frame;
            frame << Eigen::Matrix4f::Zero();
            frame.block<3, 3>(0, 0) = rot;
            frame.block<4, 1>(0, 3) << x, y, z, 1;
            poses.push_back(frame);
            f.close();
        }

        for (int i = 0; i < points_files.size(); i++)
        {
            points.push_back(Eigen::MatrixXf());
        }

#pragma omp parallel for
        for (int i = 0; i < points_files.size(); i++)
        {
            std::ifstream ff(points_files[i]);
            std::istream_iterator<float> start(ff), end;
            std::vector<float> numbers(start, end);

            int rows = numbers.size() / 6;
            Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data(numbers.data(), rows, 6);

            points[i] = Eigen::MatrixXf(rows, 3);
            points[i].block(0, 0, rows, 3) = data.block(0, 3, rows, 3);

            std::cout << "Loading: " << float(i) / float(points_files.size()) * 100.0 << "\% \n";
            ff.close();
        }
    }
};

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

    float getTime(std::string name)
    {
        return float(elapsedMicroseconds(name).count()) / 1000.0f;
    }
} timings;

/**
 * skimap definition
 */
typedef float CoordinateType;
typedef int16_t IndexType;
typedef uint16_t WeightType;
typedef skimap::VoxelDataRGBW<IndexType, WeightType> VoxelData;

typedef skimap::SkiMap<VoxelData, IndexType, CoordinateType> SKIMAP;
typedef skimap::SkiMap<VoxelData, IndexType, CoordinateType>::Voxel3D Voxel3D;
typedef skimap::SkiMap<VoxelData, IndexType, CoordinateType>::Tiles2D Tiles2D;
SKIMAP *map;

typedef skimap::Octree<VoxelData, CoordinateType, OCTREE_SPLITS> OCTREE;
typedef skimap::Octree<VoxelData, CoordinateType, OCTREE_SPLITS>::Voxel3D Voxel3D;
octomap::ColorOcTree *octree;

/**
 * INTEGRATE OCTOMAP OCTREE
 */
void integrateOctomap(Eigen::MatrixXf &points)
{
    int cols = points.cols();

    for (int i = 0; i < cols; i++)
    {
        octomap::point3d endpoint(points(0, i),
                                  points(1, i),
                                  points(2, i));

        octomap::ColorOcTreeNode *n = octree->updateNode(endpoint, true);
        n->setColor(0, 0, 0);
    }
}

/**
 * INTEGRATE SKIMAP
 */
void integrateSkimap(Eigen::MatrixXf &points)
{
    int cols = points.cols();
    for (int n = 0; n < cols; n++)
    {
        float x = points(0, n);
        float y = points(1, n);
        float z = points(2, n);

        VoxelData voxel(0, 0, 0, 1.0);

        map->integrateVoxel(x, y, z, &voxel);
    }
}

/**
 * ROS
 */
ros::NodeHandle *nh;

/**
 * Helper function to obtain a random double in range.
 */
double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv)
{

    srand(time(NULL));

    // Initialize ROS
    ros::init(argc, argv, "skimap_live");
    nh = new ros::NodeHandle("~");
    int hz;
    nh->param<int>("hz", hz, 30);
    ros::Rate r(hz);

    /**
     * DATASET
     */
    GenericDataset *dataset = new FreiburgDataset("/home/daniele/data/datasets/freiburgCampus360_3D");

    /**
     * ALGO 
     */
    std::string algo_name = nh->param<std::string>("algo_name", "skimap");

    /**
     * MAP CREATION
     */
    float map_resolution = nh->param<float>("map_resolution", 0.05);
    std::string map_algo = nh->param<std::string>("map_algo", "skimap");

    if (algo_name.compare("skimap") == 0)
    {
        map = new SKIMAP(map_resolution, 0);
    }
    if (algo_name.compare("octree") == 0)
    {
        octree = new octomap::ColorOcTree(map_resolution);
    }

    //map = new OCTREE(map_resolution, 0);

    int i = 0;
    while (ros::ok())
    {
        if (i >= dataset->points.size())
        {
            i = 0;
            break;
        }

        /**
         * POINTS WORLD TRANSFORM
         */
        int rows = dataset->points[i].rows();
        int cols = dataset->points[i].cols();
        ROS_INFO_STREAM("" << algo_name << " Frame: " << i << "/ " << dataset->points.size());

        Eigen::MatrixXf hpoints(rows, 4);
        Eigen::MatrixXf ones = Eigen::MatrixXf::Ones(rows, 1);
        hpoints << dataset->points[i], ones;

        Eigen::MatrixXf transformedPoints;
        transformedPoints = dataset->poses[i] * hpoints.transpose();

        /**
         * INTEGRATION
         */
        timings.startTimer("integration");

        if (algo_name.compare("skimap") == 0)
        {
            integrateSkimap(transformedPoints);
        }
        if (algo_name.compare("octree") == 0)
        {
            integrateOctomap(transformedPoints);
        }

        timings.printTime("integration");

        i++;
        ros::spinOnce();
        r.sleep();
    }

    std::vector<Voxel3D> voxels;
    //map->fetchVoxels(voxels);

    ROS_INFO_STREAM("Integrated voxels:" << voxels.size());

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    //     // Builds the map
    //     float map_resolution = 0.05;
    //     map = new SKIMAP(map_resolution);

    //     /**
    //    * This command enables the Concurrent Access Self Management. If it is
    //    * enabled
    //    * you can use OPENMP to call the method 'integrateVoxel' in a concurrent
    //    * manner safely.
    //    */
    //     map->enableConcurrencyAccess();

    //     /**
    //    * With this two parameters we can simulate N_MEASUREMENTS sensor measurements
    //    * each of them with N_POINTS points
    //    */
    //     int N_MEASUREMENTS = 100;
    //     int N_POINTS = 640 * 480 / 2;

    //     for (int m = 0; m < N_MEASUREMENTS; m++)
    //     {
    //         /**
    //      * Integration Timer
    //      */

    //         // #pragma omp parallel for
    //         for (int i = 0; i < N_POINTS; i++)
    //         {

    //             /**
    //        * Generates a random 3D Point
    //        */
    //             double x = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);
    //             double y = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);
    //             double z = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);

    //             /**
    //        * Creates a Voxel. In this case the voxel is a VoxelDataColor data
    //        * structure with r,g,b color information and a w weight. In general
    //        * the 'weight' is used to fuse voxel togheter: a positive weight 1.0
    //        * means an addiction, a negative weight -1.0 means a subtraction
    //        */
    //             VoxelDataColor voxel;
    //             voxel.r = fRand(MIN_RANDOM_COLOR, MAX_RANDOM_COLOR);
    //             voxel.g = fRand(MIN_RANDOM_COLOR, MAX_RANDOM_COLOR);
    //             voxel.b = fRand(MIN_RANDOM_COLOR, MAX_RANDOM_COLOR);
    //             voxel.w = 1.0;

    //             /**
    //        * Integration of custom voxel in the SkiMap data structure.
    //        */
    //             map->integrateVoxel(CoordinateType(x), CoordinateType(y),
    //                                 CoordinateType(z), &voxel);
    //         }

    //         /**
    //      * Map Visiting. With this command you can extract all voxels in SkiMap.
    //      * You can iterate on results to display Voxels in your viewer
    //      */
    //         std::vector<Voxel3D> voxels;
    //         map->fetchVoxels(voxels);

    //         printf("Map voxels: %d\n", int(voxels.size()));
    //     }
}
