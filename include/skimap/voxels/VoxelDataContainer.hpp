/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef VOXELDATACONTAINER_HPP
#define VOXELDATACONTAINER_HPP

#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <iomanip>
#include <limits>

namespace skimap
{

/**
 * Voxel Data with a generic container
 * D template represents datatype for container.
 * COLUMNS template represents fixed columns size.
 */
template <typename D>
struct VoxelDataContainer
{
    std::vector<D> container;

    /**
     * Pointer Copy Constructor.
     * @param data source data
     */
    VoxelDataContainer(VoxelDataContainer *data)
    {
        container = data->container;
    }

    /**
     * Void Constructor.
     */
    VoxelDataContainer()
    {
    }

    /**
     * Sum Overload. Defines Sum operations between Voxels
     * @param v2 second addend
     * @return sum
     */
    VoxelDataContainer operator+(const VoxelDataContainer &v2) const
    {
        VoxelDataContainer v1 = *this;
        if (v1.container.size() == 0)
            return v2;

        VoxelDataContainer d;
        d.container = v1.container;
        d.container.insert(d.container.end(), v2.container.begin(), v2.container.end());
        return d;
    }

    /**
     * Subtraction Overload. Defines Subtraction operations between Voxels
     * @param v2 second minuend
     * @return subtraction
     */
    VoxelDataContainer operator-(const VoxelDataContainer &v2) const
    {
        VoxelDataContainer v1 = *this;
        if (v2.container.size() == 0)
            return v1;
        VoxelDataContainer d;
        d.container = this->container;
        printf("VOXELDATACONTAINER OPERATOR- NOT IMPLEMENTED YET!\n");
        return d;
    }

    /**
     * Serializes object into stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const VoxelDataContainer<D> &voxel)
    {

        int count = 0;
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << voxel.container.size() << " ";
        for (int i = 0; i < voxel.container.size(); i++)
        {
            os << voxel.container[i];
            if (i < voxel.container.size() - 1)
            {
                os << " ";
            }
        }
        return os;
    }

    /**
     * Hydrates object from stream.
     */
    friend std::istream &operator>>(std::istream &is, VoxelDataContainer<D> &voxel)
    {

        int size;
        is >> size;
        voxel.container.resize(size);
        for (int i = 0; i < size; i++)
        {
            D el;
            is >> el;
            voxel.container[i] = el;
        }

        return is;
    }
};
}

#endif /* VOXELDATACONTAINER_HPP */
