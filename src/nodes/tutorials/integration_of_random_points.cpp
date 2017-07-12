/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

//OPENCV
#include <opencv2/opencv.hpp>

//Skimap
#include <skimap/SkiMap.hpp>
#include <skimap/voxels/VoxelDataRGBW.hpp>

//Utils
#include <skimap/utils/TimingsUtils.hpp>

#define MAX_RANDOM_COORD 10.0
#define MIN_RANDOM_COORD -10.0
#define MAX_RANDOM_COLOR 1.0
#define MIN_RANDOM_COLOR 0.0

/** 
 * skimap definition
 */
typedef float CoordinateType;
typedef int16_t IndexType;
typedef uint16_t WeightType;
typedef skimap::VoxelDataRGBW<IndexType, WeightType> VoxelDataColor;
typedef skimap::SkiMap<VoxelDataColor, IndexType, CoordinateType> SKIMAP;
typedef skimap::SkiMap<VoxelDataColor, IndexType, CoordinateType>::Voxel3D Voxel3D;
typedef skimap::SkiMap<VoxelDataColor, IndexType, CoordinateType>::Tiles2D Tiles2D;
SKIMAP *map;

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

    //Builds the map
    float map_resolution = 0.05;
    map = new SKIMAP(map_resolution);

    /**
     * This command enables the Concurrent Access Self Management. If it is enabled
     * you can use OPENMP to call the method 'integrateVoxel' in a concurrent manner safely. 
     */
    map->enableConcurrencyAccess();

    /**
     * With this two parameters we can simulate N_MEASUREMENTS sensor measurements
     * each of them with N_POINTS points
     */
    int N_MEASUREMENTS = 100;
    int N_POINTS = 640 * 480 / 2;

    for (int m = 0; m < N_MEASUREMENTS; m++)
    {
        /**
         * Integration Timer
         */
        DEBUG_TIMINGS.startTimer("Integration");

#pragma omp parallel for
        for (int i = 0; i < N_POINTS; i++)
        {

            /**
             * Generates a random 3D Point
             */
            double x = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);
            double y = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);
            double z = fRand(MIN_RANDOM_COORD, MAX_RANDOM_COORD);

            /**
             * Creates a Voxel. In this case the voxel is a VoxelDataColor data
             * structure with r,g,b color information and a w weight. In general
             * the 'weight' is used to fuse voxel togheter: a positive weight 1.0 
             * means an addiction, a negative weight -1.0 means a subtraction
             */
            VoxelDataColor voxel;
            voxel.r = fRand(MIN_RANDOM_COLOR, MAX_RANDOM_COLOR);
            voxel.g = fRand(MIN_RANDOM_COLOR, MAX_RANDOM_COLOR);
            voxel.b = fRand(MIN_RANDOM_COLOR, MAX_RANDOM_COLOR);
            voxel.w = 1.0;

            /**
             * Integration of custom voxel in the SkiMap data structure.
             */
            map->integrateVoxel(
                CoordinateType(x),
                CoordinateType(y),
                CoordinateType(z),
                &voxel);
        }

        DEBUG_TIMINGS.printTime("Integration");

        /**
         * Map Visiting. With this command you can extract all voxels in SkiMap.
         * You can iterate on results to display Voxels in your viewer
         */
        DEBUG_TIMINGS.startTimer("Visiting");
        std::vector<Voxel3D> voxels;
        map->fetchVoxels(voxels);
        DEBUG_TIMINGS.printTime("Visiting");

        printf("Map voxels: %d\n", int(voxels.size()));
    }
}
