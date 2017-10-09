/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <utility> // std::pair
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <algorithm>
#include <fstream>
#include <iostream>

#include "sys/types.h"
#include "sys/sysinfo.h"

#include <ANN/ANN.h> // ANN declarations
#include <flann/flann.hpp>

//Eigen
#include <Eigen/Core>

// OPENCV
#include <opencv2/opencv.hpp>

// Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/SkipListGrid.hpp>
#include <skimap/voxels/VoxelDataMatrix.hpp>

#include <skimap/utils/ArgParse.hpp>

#define MAX_RANDOM_COLOR 1.0
#define MIN_RANDOM_COLOR 0.0

struct sysinfo memInfo;

template <class K, class V, int D = 3>
class Skippa
{
public:
  K index;
  V data;
};

typedef int Index;
typedef Skippa<Index, void *, 8> KNODE;

struct QPoint
{
  double coords[2];

  QPoint(){};

  QPoint(const QPoint &other)
  {
    coords[0] = other.coords[0];
    coords[1] = other.coords[1];
  }

  double operator[](size_t idx) const { return coords[idx]; }
  double &operator[](size_t idx) { return coords[idx]; }
};

void process_mem_usage(double &vm_usage, double &resident_set)
{
  using std::ios_base;
  using std::ifstream;
  using std::string;

  vm_usage = 0.0;
  resident_set = 0.0;

  // 'file' stat seems to give the most reliable results
  //
  ifstream stat_stream("/proc/self/stat", ios_base::in);

  // dummy vars for leading entries in stat that we don't care about
  //
  string pid, comm, state, ppid, pgrp, session, tty_nr;
  string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  string utime, stime, cutime, cstime, priority, nice;
  string O, itrealvalue, starttime;

  // the two fields we want
  //
  unsigned long vsize;
  long rss;

  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >> nice >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

  stat_stream.close();

  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;
}

/**
 * skimap definition
 */
typedef float CoordinateType;
typedef int16_t IndexType;
typedef uint16_t WeightType;
typedef Eigen::Vector2f PointType;
typedef float CoordType;
typedef skimap::VoxelDataMatrix<CoordType> VoxelData;
typedef skimap::SkipListGrid<VoxelData, IndexType, CoordinateType, 10, 10> SkiGrid;
typedef skimap::SkipListGrid<VoxelData, IndexType, CoordinateType, 10, 10>::Voxel2D Voxel2D;

SkiGrid *map;

auto _current_time = std::chrono::high_resolution_clock::now();
auto getTime()
{
  _current_time = std::chrono::high_resolution_clock::now();
  return _current_time;
}

auto deltaTime()
{
  auto t1 = _current_time;
  auto t2 = getTime();
  auto dt = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  return dt / 1000000.0;
}
/**
 * Helper function to obtain a random double in range.
 */
float fRand(float fMin, float fMax)
{
  float f = (float)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

typedef std::vector<std::vector<CoordType>> Points;
bool _debug = true;
double MAX_RANDOM_COORD = 1000.0;
double MIN_RANDOM_COORD = 0.0;
int N_POINTS = 100000;
float sparsity;
/**
 * ##########
 */
Points generateFeatures(int n)
{
  Points features;
  for (int i = 0; i < n; i++)
  {
    float x = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);
    float y = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);

    features.push_back(std::vector<CoordType>{x, y});
  }
  return features;
}

CoordType distance(std::vector<CoordType> p1, std::vector<CoordType> p2)
{

  return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
}

void drawPoints(cv::Mat &image, Points &points, cv::Scalar color)
{

  CoordType x, y;
  for (int i = 0; i < points.size(); i++)
  {
    x = points[i][0];
    y = points[i][1];
    image.at<cv::Vec3b>(y, x)[0] = color[2];
    image.at<cv::Vec3b>(y, x)[1] = color[1];
    image.at<cv::Vec3b>(y, x)[2] = color[0];
  }
}

void printResults(std::string name, double t_c, double t_s, double memory)
{
  printf("%f %f %f", t_c, t_s, memory);
}
/**
 * ##########
 */
void computeSkigrid(cv::Mat &image, Points &points, CoordType radius, bool print_output)
{

  float map_resolution = 1;
  SkiGrid *map = new SkiGrid(map_resolution);
  map->enableConcurrencyAccess(true);

  getTime();
#pragma omp parallel for
  for (int i = 0; i < points.size(); i++)
  {
    VoxelData voxel;
    voxel.matrix.push_back(points[i]);
    map->integrateVoxel(CoordinateType(points[i][0]), CoordinateType(points[i][1]), &voxel);
  }
  double time_creation = deltaTime();

  getTime();
  std::vector<Voxel2D> voxels;
  map->radiusSearch(CoordinateType(MAX_RANDOM_COORD / 2),
                    CoordinateType(MAX_RANDOM_COORD / 2),
                    radius, radius, voxels);
  Points result;
  for (int i = 0; i < voxels.size(); i++)
  {
    result.insert(result.end(), voxels[i].data->matrix.begin(), voxels[i].data->matrix.end());
  }
  double time_search = deltaTime();

  double vm, rss;
  process_mem_usage(vm, rss);
  double memory = rss;

  if (print_output)
  {
    printResults("skigrid", time_creation, time_search, memory);
  }

  if (_debug)
  {
    drawPoints(image, points, cv::Scalar(255, 234, 0));
    cv::imshow("img", image);
    cv::waitKey(0);

    drawPoints(image, result, cv::Scalar(0, 255, 234));
    cv::imshow("img", image);
    cv::waitKey(0);
  }
}

/**
 * ##########
 */
void computeDenseGrid(cv::Mat &image, Points &points, CoordType radius, float &sparsity, bool print_output)
{
  float grid_resolution = 1;
  int grid_side = MAX_RANDOM_COORD / grid_resolution;
  Points *grid = new Points[grid_side * grid_side];

  getTime();
  CoordType x, y;
  int ix, iy, index;

  for (int i = 0; i < points.size(); i++)
  {
    x = points[i][0];
    y = points[i][1];
    ix = x / grid_resolution;
    iy = y / grid_resolution;
    if (ix > 0 && iy > 0 && ix < grid_side && iy < grid_side)
    {
      index = ix * grid_side + iy;

      grid[index].push_back(std::vector<CoordType>{x, y});
    }
  }
  double time_creation = deltaTime();

  int void_counter = 0;
  for (int i = 0; i < grid_side * grid_side; i++)
  {
    if (grid[i].size() == 0)
    {
      void_counter++;
    }
  }
  sparsity = float(void_counter) / (grid_side * grid_side);

  CoordType cx = MAX_RANDOM_COORD / 2.0;
  CoordType cy = MAX_RANDOM_COORD / 2.0;
  std::vector<CoordType> center{cx, cy};

  CoordType minx = cx - radius;
  CoordType miny = cy - radius;
  CoordType maxx = cx + radius;
  CoordType maxy = cy + radius;
  int min_ix = minx / grid_resolution;
  int min_iy = miny / grid_resolution;
  int max_ix = maxx / grid_resolution;
  int max_iy = maxy / grid_resolution;

  getTime();
  Points result;

#pragma omp parallel
  {
    Points result_private;

#pragma omp for nowait
    for (int ix = min_ix; ix <= max_ix; ix++)
    {
      for (int iy = min_iy; iy <= max_iy; iy++)
      {
        int index = ix * grid_side + iy;
        if (index > 0 && index < grid_side * grid_side)
        {
          for (int i = 0; i < grid[index].size(); i++)
          {
            CoordType dist = distance(grid[index][i], center);
            if (dist <= radius)
            {
              result_private.push_back(grid[index][i]);
            }
          }
        }
      }
    }

#pragma omp critical
    result.insert(result.end(), result_private.begin(), result_private.end());
  }

  double time_search = deltaTime();

  double vm, rss;
  process_mem_usage(vm, rss);
  double memory = rss;

  if (print_output)
  {
    printResults("densegrid", time_creation, time_search, memory);
  }
  if (_debug)
  {
    drawPoints(image, points, cv::Scalar(255, 0, 0));
    cv::imshow("img", image);
    cv::waitKey(0);

    drawPoints(image, result, cv::Scalar(0, 255, 0));
    cv::imshow("img", image);
    cv::waitKey(0);
  }
  delete[] grid;
}

void computeAnn(cv::Mat &image, Points &points, CoordType radius, std::string substring, bool print_output)
{

  ANNpointArray dataPts; // data points
  ANNpoint queryPt;      // query point
  ANNidxArray nnIdx;     // near neighbor indices
  ANNdistArray dists;    // near neighbor distances

  int maxPts = points.size();
  int k = N_POINTS;
  int dim = 2;
  queryPt = annAllocPt(dim); // allocate query point
  dataPts = annAllocPts(maxPts, dim);

  nnIdx = new ANNidx[k]; // allocate near neigh indices
  dists = new ANNdist[k];

  for (int i = 0; i < maxPts; i++)
  {
    dataPts[i][0] = points[i][0];
    dataPts[i][1] = points[i][1];
  }

  ANNpoint q = annAllocPt(dim);
  q[0] = CoordinateType(MAX_RANDOM_COORD / 2);
  q[1] = CoordinateType(MAX_RANDOM_COORD / 2);

  ANNdist sqRad = radius;

  getTime();
  auto kdTree = new ANNkd_tree( // build search structure
      dataPts,                  // the data points
      N_POINTS,                 // number of points
      dim);
  double time_creation = deltaTime();

  getTime();
  int res = kdTree->annkFRSearch( // approx fixed-radius kNN search
      q,                          // query point
      sqRad * sqRad,              // squared radius
      N_POINTS,                   // number of near neighbors to return
      nnIdx,                      // nearest neighbor array (modified)
      dists);
  double time_search = deltaTime();
  double vm, rss;
  process_mem_usage(vm, rss);
  double memory = rss;

  if (print_output)
  {
    printResults(substring, time_creation, time_search, memory);
  }
  if (_debug)
  {
    Points result;
    for (int i = 0; i < res; i++)
    {
      result.push_back(points[nnIdx[i]]);
    }

    drawPoints(image, points, cv::Scalar(255, 0, 0));
    cv::imshow("img", image);
    cv::waitKey(0);

    drawPoints(image, result, cv::Scalar(0, 255, 0));
    cv::imshow("img", image);
    cv::waitKey(0);
  }
}
void computeFlann(cv::Mat &image, Points &points, CoordType radius, std::string substring, bool print_output)
{

  int ndim = 2;
  // using namespace flann;
  // Matrix<float> dataset(new CoordType[points.size() * ndim], points.size(), ndim);
  // Matrix<float> query(new CoordType[1], 1, 1);

  // int i = 0;
  // for (auto &&point : points)
  // {
  //   dataset[i][0] = point[0];
  //   dataset[i][1] = point[1];
  //   // printf("%f,%f\n", dataset[i][0], dataset[i][1]);
  //   i++;
  //   //Fill matrix
  // }

  // query[0][0] = MAX_RANDOM_COORD / 2.0;
  // query[0][1] = MAX_RANDOM_COORD / 2.0;

  // Matrix<int> indices(new int[points.size() * ndim], points.size(), ndim);
  // Matrix<float> dists(new CoordType[points.size() * ndim], points.size(), ndim);

  // printf("Buildin\n");
  // Index<L2<float>> index(dataset, flann::LinearIndexParams());
  // index.buildIndex();
  // printf("Done\n");
  // // do a knn search, using 128 checks
  // int res = index.radiusSearch(query, indices, dists, radius * radius, flann::SearchParams(128));

  // if (_debug)
  // {
  //   printf("RES %d\n", res);
  //   Points result;
  //   for (int i = 0; i < res; i++)
  //   {
  //     result.push_back(points[indices[i][0]]);
  //   }

  //   drawPoints(image, points, cv::Scalar(255, 0, 0));
  //   cv::imshow("img", image);
  //   cv::waitKey(0);

  //   drawPoints(image, result, cv::Scalar(0, 255, 0));
  //   cv::imshow("img", image);
  //   cv::waitKey(0);
  // }

  cv::Mat_<CoordType> features(points.size(), 2);
  for (int i = 0; i < points.size(); i++)
  {
    features.at<CoordType>(i, 0) = points[i][0];
    features.at<CoordType>(i, 1) = points[i][1];
    // cv::Mat row = (cv::Mat_<CoordType>(1, 2) << point[0], point[1]);
    // features.push_back(row);
  }

  cv::flann::IndexParams *ip;
  if (substring.compare("flann_brute") == 0)
  {
    ip = new cv::flann::LinearIndexParams();
  }
  else if (substring.compare("flann_kdtree") == 0)
  {
    ip = new cv::flann::KDTreeIndexParams(2);
  }
  else if (substring.compare("flann_lsh") == 0)
  {
    ip = new cv::flann::LshIndexParams(20, 15, 2);
  }
  else if (substring.compare("flann_auto") == 0)
  {
    ip = new cv::flann::AutotunedIndexParams();
  }
  else if (substring.compare("flann_composite") == 0)
  {
    ip = new cv::flann::CompositeIndexParams();
  }
  else if (substring.compare("flann_kmeans") == 0)
  {
    ip = new cv::flann::KMeansIndexParams(32, 50);
  }

  else
  {
    printf("ERORRO UNRECOGNIZED ALGO: %s\n", substring.c_str());
    exit(0);
  }
  cv::flann::Index flann_index(features, *ip);
  double time_creation = deltaTime();

  CoordType cx = MAX_RANDOM_COORD / 2.0;
  CoordType cy = MAX_RANDOM_COORD / 2.0;
  unsigned int max_neighbours = points.size() * 10;
  cv::Mat query = (cv::Mat_<CoordType>(1, 2) << cx, cy);
  std::vector<int> indices; //neither assume type nor size here !
  std::vector<CoordType> dists;

  getTime();
  flann_index.radiusSearch(query, indices, dists, radius * radius, max_neighbours,
                           cv::flann::SearchParams(points.size() * 10));
  double time_search = deltaTime();

  double vm, rss;
  process_mem_usage(vm, rss);
  double memory = rss;

  if (print_output)
  {
    printResults("flann_kdtree", time_creation, time_search, memory);
  }

  if (_debug)
  {

    Points result;
    for (int i = 0; i < indices.size(); i++)
    {
      result.push_back(points[indices[i]]);
    }
    drawPoints(image, points, cv::Scalar(255, 0, 0));
    cv::imshow("img", image);
    cv::waitKey(0);

    drawPoints(image, result, cv::Scalar(0, 255, 0));
    cv::imshow("img", image);
    cv::waitKey(0);
  }
}
/**
 * ##########
 */
int main(int argc, const char **argv)
{

  N_POINTS = atoi(argv[1]);
  MAX_RANDOM_COORD = atof(argv[2]);
  MIN_RANDOM_COORD = 0.0;
  std::string algo(argv[3]);
  _debug = atoi(argv[4]);

  std::vector<std::vector<CoordType>> features = generateFeatures(N_POINTS);

  cv::Mat image(MAX_RANDOM_COORD, MAX_RANDOM_COORD, CV_8UC3, cv::Scalar(0, 0, 0));

  //computeDenseGrid(image, features, MAX_RANDOM_COORD / 4.0, sparsity, false);
  printf("%s %d %f %f ", algo.c_str(), N_POINTS, MAX_RANDOM_COORD, sparsity);

  if (algo.compare("dense") == 0)
  {
    computeDenseGrid(image, features, MAX_RANDOM_COORD / 4.0, sparsity, true);
  }

  else if (algo.compare("skigrid") == 0)
  {
    computeSkigrid(image, features, MAX_RANDOM_COORD / 4.0, true);
  }

  else if (algo.find("flann") == 0)
  {
    computeFlann(image, features, MAX_RANDOM_COORD / 4.0, algo, true);
  }

  else if (algo.find("ann") == 0)
  {
    computeAnn(image, features, MAX_RANDOM_COORD / 4.0, algo, true);
  }
}

int main2(int argc, char **argv)
{

  srand(time(NULL));

  // Builds the map
  float map_resolution = 1;
  map = new SkiGrid(map_resolution);

  /**
   * This command enables the Concurrent Access Self Management. If it is
   * enabled
   * you can use OPENMP to call the method 'integrateVoxel' in a concurrent
   * manner safely.
   */
  map->enableConcurrencyAccess();

  /**
   * With this two parameters we can simulate N_MEASUREMENTS sensor measurements
   * each of them with N_POINTS points
   */
  int N_MEASUREMENTS = 100;
  int N_POINTS = 300000;
  using namespace cv;

  cv::Mat image(MAX_RANDOM_COORD, MAX_RANDOM_COORD, CV_8UC3, cv::Scalar(0, 0, 0));

  std::vector<cv::Point2f> features;

  auto t0 = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < N_POINTS; i++)
  {
    float x = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);
    float y = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);

    // Fill matrix
    features.push_back(cv::Point2f(x, y));
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  auto dt =
      std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  std::cout << "Raw Integration:" << dt << "\n";

  /**
   * Integration Timer
   */
  t0 = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
  for (int i = 0; i < N_POINTS; i++)
  {

    /**
     * Generates a random 3D Point
     */
    float x = features[i].x;
    float y = features[i].y;

    image.at<cv::Vec3b>(y, x)[0] = 255;
    /**
     * Creates a Voxel. In this case the voxel is a VoxelDataColor data
     * structure with r,g,b color information and a w weight. In general
     * the 'weight' is used to fuse voxel togheter: a positive weight 1.0
     * means an addiction, a negative weight -1.0 means a subtraction
     */
    VoxelData voxel;
    voxel.matrix.push_back(std::vector<CoordType>{x, y});
    /**
     * Integration of custom voxel in the SkiMap data structure.
     */
    map->integrateVoxel(CoordinateType(x), CoordinateType(y), &voxel);
  }
  t1 = std::chrono::high_resolution_clock::now();
  dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  std::cout << "Integration:" << dt << "\n";

  cv::imshow("img", image);
  cv::waitKey(0);

  t0 = std::chrono::high_resolution_clock::now();
  std::vector<Voxel2D> voxels;
  map->radiusSearch(CoordinateType(MAX_RANDOM_COORD / 2),
                    CoordinateType(MAX_RANDOM_COORD / 2),
                    CoordinateType(MAX_RANDOM_COORD / 4),
                    CoordinateType(MAX_RANDOM_COORD / 4), voxels);
  std::vector<std::vector<CoordType>> points;
  for (int i = 0; i < voxels.size(); i++)
  {
    points.insert(points.end(), voxels[i].data->matrix.begin(), voxels[i].data->matrix.end());
  }

  t1 = std::chrono::high_resolution_clock::now();
  dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  std::cout << "Search:" << dt << "\n";

  for (int i = 0; i < points.size(); i++)
  {
    std::vector<CoordType> v = points[i];
    image.at<cv::Vec3b>(v[1], v[0])[0] = 255;
    image.at<cv::Vec3b>(v[1], v[0])[1] = 255;
    image.at<cv::Vec3b>(v[1], v[0])[2] = 255;
  }
  cv::imshow("img", image);
  cv::waitKey(0);

  ANNpointArray dataPts; // data points
  ANNpoint queryPt;      // query point
  ANNidxArray nnIdx;     // near neighbor indices
  ANNdistArray dists;    // near neighbor distances

  int maxPts = N_POINTS;
  int k = N_POINTS;
  int dim = 2;
  queryPt = annAllocPt(dim); // allocate query point
  dataPts = annAllocPts(maxPts, dim);

  nnIdx = new ANNidx[k]; // allocate near neigh indices
  dists = new ANNdist[k];

  for (int i = 0; i < N_POINTS; i++)
  {

    float x = features[i].x;
    float y = features[i].y;

    dataPts[i][0] = x;
    dataPts[i][1] = y;
  }

  ANNpoint q = annAllocPt(dim);
  q[0] = CoordinateType(MAX_RANDOM_COORD / 2);
  q[1] = CoordinateType(MAX_RANDOM_COORD / 2);

  ANNdist sqRad = CoordinateType(MAX_RANDOM_COORD / 4);

  t0 = std::chrono::high_resolution_clock::now();
  auto kdTree = new ANNkd_tree( // build search structure
      dataPts,                  // the data points
      N_POINTS,                 // number of points
      dim);

  t1 = std::chrono::high_resolution_clock::now();
  dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  std::cout << "ANN Creation:" << dt << "\n";

  printf("ANN Query on radius: %f\n", sqRad);

  t0 = std::chrono::high_resolution_clock::now();
  int res = kdTree->annkFRSearch( // approx fixed-radius kNN search
      q,                          // query point
      sqRad * sqRad,              // squared radius
      N_POINTS,                   // number of near neighbors to return
      nnIdx,                      // nearest neighbor array (modified)
      dists);

  t1 = std::chrono::high_resolution_clock::now();
  dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  std::cout << "ANN Search:" << dt << "\n";

  printf("Res %d\n", res);

  for (int i = 0; i < res; i++)
  {

    double x = dataPts[nnIdx[i]][0];
    double y = dataPts[nnIdx[i]][1];
    image.at<cv::Vec3b>(y, x)[0] = 0;
    image.at<cv::Vec3b>(y, x)[1] = 255;
    image.at<cv::Vec3b>(y, x)[2] = 0;
  }
  cv::imshow("img", image);
  cv::waitKey(0);
}
