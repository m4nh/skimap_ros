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
const int DIM = 2;

typedef float CoordinatesType;
typedef skimap::VoxelDataMatrix<CoordinatesType> VoxelData;
typedef int IndexType;

float fRand(float fMin, float fMax)
{
  float f = (float)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

float distance(std::vector<CoordinatesType> cds1, std::vector<CoordinatesType> cds2)
{
  float d = 0.0;
  for (int i = 0; i < cds1.size(); i++)
  {
    d += pow(cds1[i] - cds2[i], 2);
  }
  return sqrt(d);
}

bool checkPresence(std::vector<std::vector<CoordinatesType>> &search_data, std::vector<CoordinatesType> cds)
{

  for (int i = 0; i < search_data.size(); i++)
  {

    if (distance(search_data[i], cds) < 0.1)
    {
      return true;
    }
  }
  return false;
}

int main(int argc, char **argv)
{

  srand(time(NULL));

  skimap::KDSkipList<VoxelData, IndexType, CoordinatesType, DIM> kd_skip_list(0.01);
  typedef skimap::KDSkipList<VoxelData, IndexType, CoordinatesType, DIM>::VoxelKD Voxel;

  cv::Mat image(MAX_RANDOM_COORD, MAX_RANDOM_COORD, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat image2(MAX_RANDOM_COORD, MAX_RANDOM_COORD, CV_8UC3, cv::Scalar(0, 0, 0));

  std::vector<std::vector<CoordinatesType>> integration_data;

  for (int i = 0; i < 5000000; i++)
  {

    std::vector<CoordinatesType> cds;
    for (int d = 0; d < DIM; d++)
    {
      cds.push_back(fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD));
    }

    VoxelData voxel;
    voxel.matrix.push_back(cds);
    kd_skip_list.integrateVoxel(cds, &voxel);
    integration_data.push_back(cds);
    if (DIM == 2)
    {
      image.at<cv::Vec3b>(cds[1], cds[0]) = cv::Vec3b(255, 255, 255);
    }
  }

  std::vector<Voxel> voxels;
  kd_skip_list.fetchVoxels(voxels);

  bool consistency = true;
  for (int i = 0; i < voxels.size(); i++)
  {
    Voxel &v = voxels[i];
    consistency &= checkPresence(integration_data, v.coordinates);
    if (DIM == 2)
    {
      image2.at<cv::Vec3b>(v.coordinates[1], v.coordinates[0]) = cv::Vec3b(255, 255, 255);
    }
  }
  if (consistency)
  {
    printf("Ski is good!\n");
  }

  cv::imshow("img", image);
  cv::imshow("img2", image2);
  cv::waitKey(0);
  printf("Ciao\n");
}
