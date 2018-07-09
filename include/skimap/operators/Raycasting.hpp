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

class Vector3
{
  private:
    double _x, _y, _z;

  public:
    Vector3(double x, double y, double z) : _x(x), _y(y), _z(z)
    {
    }
    Vector3() : _x(0), _y(0), _z(0) {}

    double x() const { return this->_x; }
    double y() const { return this->_y; }
    double z() const { return this->_z; }
    void x(double x) { this->_x = x; }
    void y(double y) { this->_y = y; }
    void z(double z) { this->_z = z; }
};

class Ray
{
  public:
    Ray(Vector3 o, Vector3 d)
    {
        origin = o;
        direction = d;
        inv_direction = Vector3(1 / d.x(), 1 / d.y(), 1 / d.z());
        sign[0] = (inv_direction.x() < 0);
        sign[1] = (inv_direction.y() < 0);
        sign[2] = (inv_direction.z() < 0);
    }
    Vector3 origin;
    Vector3 direction;
    Vector3 inv_direction;
    int sign[3];
};

class Box
{
  public:
    Box(const Vector3 &min, const Vector3 &max)
    {
        bounds[0] = min;
        bounds[1] = max;
    }
    Vector3 bounds[2];

    // Smits’ method
    bool intersect(const Ray &r, float t0, float t1) const
    {
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
        tmin = (bounds[r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
        tmax = (bounds[1 - r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
        tymin = (bounds[r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
        tymax = (bounds[1 - r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
        if ((tmin > tymax) || (tymin > tmax))
            return false;
        if (tymin > tmin)
            tmin = tymin;
        if (tymax < tmax)
            tmax = tymax;
        tzmin = (bounds[r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
        tzmax = (bounds[1 - r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
        if ((tmin > tzmax) || (tzmin > tmax))
            return false;
        if (tzmin > tmin)
            tmin = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;
        return ((tmin < t1) && (tmax > t0));
    }
};

// template <class M>
// class Raycasting
// {
//   public:
//     typedef typename M::Voxel3D Voxel3D;
//     typedef typename M::VoxelData VoxelData;

//     Raycasting(M *map)
//     {
//         this->_map = map;
//     }

//     void intersect(Eigen::Vector4d center, Eigen::Vector4d dir, std::vector<Voxel3D> &voxels, double delta = 0.1, double max_distance = 30.0)
//     {
//         dir.normalize();
//         int iterations = max_distance / delta;
//         for (int i = 0; i < iterations; i++)
//         {
//             Eigen::Vector4d p;
//             p = center + dir * (delta * i);
//             VoxelData *d = this->_map->find(p(0), p(1), p(2));
//             if (d != NULL)
//             {
//                 Voxel3D v;
//                 v.x = p(0);
//                 v.y = p(1);
//                 v.z = p(2);
//                 voxels.push_back(v);
//             }
//         }
//     }

//     bool intersectVoxel(Eigen::Vector4d center, Eigen::Vector4d dir, Voxel3D &voxel, double delta = 0.1, double max_distance = 30.0, double min_distance = 5.0)
//     {
//         dir.normalize();
//         int iterations = max_distance / delta;
//         for (int i = 0; i < iterations; i++)
//         {
//             Eigen::Vector4d p;
//             p = center + dir * (min_distance + delta * i);

//             uint16_t ix, iy, iz;
//             Voxel3D voxel_out;
//             if (this->_map->findVoxel(p(0), p(1), p(2), voxel))
//             {
//                 // voxel = voxel_out;
//                 // Box box(
//                 //     Vector3(voxel.x - delta * 0.5, voxel.y - delta * 0.5, voxel.z - delta * 0.5),
//                 //     Vector3(voxel.x + delta * 0.5, voxel.y + delta * 0.5, voxel.z + delta * 0.5));
//                 // Ray ray(
//                 //     Vector3(center(0), center(1), center(2)),
//                 //     Vector3(dir(0), dir(1), dir(2)));

//                 // // Voxel3D v;
//                 // // v.x = p(0);
//                 // // v.y = p(1);
//                 // // v.z = p(2);
//                 // // v.data = d;
//                 // // voxel = v;
//                 // return box.intersect(ray, 0, 100.0);
//                 return true;
//             }
//         }
//         return false;
//     }

//     bool intersectVoxel(Eigen::Vector3d center, Eigen::Vector3d dir, Voxel3D &voxel, double delta = 0.1, double max_distance = 30.0, double min_distance = 5.0)
//     {
//         dir.normalize();
//         int iterations = max_distance / delta;
//         for (int i = 0; i < iterations; i++)
//         {
//             Eigen::Vector3d p;
//             p = center + dir * (min_distance + delta * i);

//             uint16_t ix, iy, iz;
//             Voxel3D voxel_out;
//             if (this->_map->findVoxel(p(0), p(1), p(2), voxel))
//             {
//                 return true;
//             }
//         }
//         return false;
//     }

//     bool intersectVoxel(Eigen::Vector3d center, Eigen::Vector3d dir, Voxel3D &voxel, double delta, double min_distance, double max_distance)
//     {
//         dir.normalize();
//         int iterations = max_distance / delta;
//         for (int i = 0; i < iterations; i++)
//         {
//             Eigen::Vector3d p;
//             p = center + dir * (min_distance + delta * i);

//             uint16_t ix, iy, iz;
//             Voxel3D voxel_out;
//             if (this->_map->findVoxel(p(0), p(1), p(2), voxel))
//             {
//                 return true;
//             }
//         }
//         return false;
//     }

//   protected:
//     M *_map;
// };

template <class M>
class Raycasting2
{
  public:
    typedef typename M::Voxel3D Voxel3D;

    Raycasting2(M *map)
    {
        this->_map = map;
    }

    bool intersectVoxel(Eigen::Vector3d center, Eigen::Vector3d dir, Voxel3D &voxel, double delta, double min_distance, double max_distance)
    {
        dir.normalize();
        int iterations = max_distance / delta;
        for (int i = 0; i < iterations; i++)
        {
            Eigen::Vector3d p;
            p = center + dir * (min_distance + delta * i);

            uint16_t ix, iy, iz;
            Voxel3D voxel_out;
            if (this->_map->findVoxel(p(0), p(1), p(2), voxel))
            {
                return true;
            }
        }
        return false;
    }

    //TODO: in this function we use Voxels with an HIDDEN COUNTER to track hits! Why? boh!
    template <class VoxelData>
    bool intersectVoxelWithWeight(Eigen::Vector3d center, Eigen::Vector3d dir, double min_weight, Voxel3D &voxel, double delta, double min_distance, double max_distance)
    {
        dir.normalize();
        int iterations = max_distance / delta;
        for (int i = 0; i < iterations; i++)
        {
            Eigen::Vector3d p;
            p = center + dir * (min_distance + delta * i);

            uint16_t ix, iy, iz;

            if (this->_map->findVoxel(p(0), p(1), p(2), voxel))
            {
                VoxelData *data = voxel.data;
                if (data != NULL)
                    if (data->hiddenCounter >= min_weight)
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
