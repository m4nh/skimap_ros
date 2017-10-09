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
#include <skimap/KDSkipList.hpp>
#include <skimap/voxels/VoxelDataMatrix.hpp>

#define MAX_RANDOM_COLOR 1.0
#define MIN_RANDOM_COLOR 0.0
double MAX_RANDOM_COORD = 500.0;
double MIN_RANDOM_COORD = 0.0;

float fRand(float fMin, float fMax)
{
  float f = (float)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv)
{

  srand(time(NULL));

  typedef float DataType;
  typedef int IndexType;
  typedef float CoordinatesType;

  skimap::KDSkipList<DataType, IndexType, CoordinatesType, 2> kd_skip_list(0.001);

  cv::Mat image(MAX_RANDOM_COORD, MAX_RANDOM_COORD, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat image2(MAX_RANDOM_COORD, MAX_RANDOM_COORD, CV_8UC3, cv::Scalar(0, 0, 0));

  for (int i = 0; i < 1000; i++)
  {
    float x = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);
    float y = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);

    std::vector<CoordinatesType> cds{x, y};

    float d = 1;
    kd_skip_list.integrateVoxel(cds, &d);
    image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);

    // DataType *d = kd_skip_list.find(cds);

    // printf("Chedk %f\n", x);
    // if (d == NULL)
    // {
    //   printf("  Null\n");
    // }
  }
  cv::imshow("img", image);
  cv::waitKey(0);
  printf("Ciao\n");
}
