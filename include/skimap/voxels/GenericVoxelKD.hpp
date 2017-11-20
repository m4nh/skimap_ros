/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef GENERICVOXELKD_HPP
#define GENERICVOXELKD_HPP

#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <stdio.h>
#include <string.h>
#include <limits>
#include <iomanip>

namespace skimap
{

/**
     * Generic KD Voxel representation.
     * V template represents datatype for user data.
     * D template represents datatype for coordinates. 
     */
template <typename V, typename D = double>
struct GenericVoxelKD
{
    std::vector<D> coordinates;
    V *data;

    GenericVoxelKD() : data(NULL)
    {
    }

    GenericVoxelKD(std::vector<D> coordinates, V *data) : coordinates(coordinates), data(data)
    {
    }

    /**
         * Serializes object into stream.
         */
    friend std::ostream &operator<<(std::ostream &os, const GenericVoxelKD<V, D> &voxel)
    {
        /* os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << voxel.x << " ";
        os << voxel.y << " ";
        os << voxel.z << " ";
        if (voxel.data != NULL)
            os << *(voxel.data);
        return os;*/
    }

    /**
         * Hydrates object from stream.
         */
    friend std::istream &operator>>(std::istream &is, GenericVoxelKD<V, D> &voxel)
    {
        /* double x, y, z;
        is >> x;
        is >> y;
        is >> z;
        voxel.x = D(x);
        voxel.y = D(y);
        voxel.z = D(z);
        voxel.data = new V();
        is >> (*voxel.data);
        return is;*/
    }
};
}

#endif /* GenericVoxelKD_HPP */
