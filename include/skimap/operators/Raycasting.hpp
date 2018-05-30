/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef RAYCASTING_HPP
#define RAYCASTING_HPP

#include <boost/thread.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <omp.h>
#include <skimap/SkipList.hpp>
#include <skimap/SkipListDense.hpp>
#include <skimap/voxels/GenericVoxel3D.hpp>
#include <vector>
// EIGEN
#include <Eigen/Core>
#include <Eigen/Geometry>

#define SkipListMapV2_MAX_DEPTH 16

#define THREAD_SAFE

namespace skimap
{

template <class M>
class Raycasting
{
  public:
    typedef typename M::Voxel3D Voxel3D;
    typedef typename M::VoxelData VoxelData;

    Raycasting(M *map)
    {
        this->_map = map;
    }

    void intersect(double cx, double cy, double cz, double rx, double ry, double rz, std::vector<Voxel3D> &voxels, double delta = 0.1)
    {
        Eigen::Vector3d center, dir;
        center << cx, cy, cz;
        dir << rx, ry, rz;
        dir.normalize();

        int iterations = 10.0 / delta;
        for (int i = 0; i < iterations; i++)
        {
            Eigen::Vector3d p;
            p = center + dir * (delta * i);
            //std::cout << i << " -> " << center << "," << dir << "," << p << "\n";
            VoxelData *d = this->_map->find(p(0), p(1), p(2));
            if (d != NULL)
            {
                Voxel3D v;
                v.x = p(0);
                v.y = p(1);
                v.z = p(2);
                voxels.push_back(v);
            }
        }
    }

    void intersect(Eigen::Vector4d center, Eigen::Vector4d dir, std::vector<Voxel3D> &voxels, double delta = 0.1, double max_distance = 30.0)
    {
        dir.normalize();
        int iterations = max_distance / delta;
        for (int i = 0; i < iterations; i++)
        {
            Eigen::Vector4d p;
            p = center + dir * (delta * i);
            VoxelData *d = this->_map->find(p(0), p(1), p(2));
            if (d != NULL)
            {
                Voxel3D v;
                v.x = p(0);
                v.y = p(1);
                v.z = p(2);
                voxels.push_back(v);
            }
        }
    }

    bool intersectVoxel(Eigen::Vector4d center, Eigen::Vector4d dir, Voxel3D &voxel, double delta = 0.1, double max_distance = 30.0)
    {
        dir.normalize();
        int iterations = max_distance / delta;
        for (int i = 0; i < iterations; i++)
        {
            Eigen::Vector4d p;
            p = center + dir * (delta * i);
            VoxelData *d = this->_map->find(p(0), p(1), p(2));
            if (d != NULL)
            {
                Voxel3D v;
                v.x = p(0);
                v.y = p(1);
                v.z = p(2);
                v.data = d;
                voxel = v;
                return true;
            }
        }
        return false;
    }

  protected:
    M *_map;
};
} // namespace skimap

#endif /* RAYCASTING_HPP */
