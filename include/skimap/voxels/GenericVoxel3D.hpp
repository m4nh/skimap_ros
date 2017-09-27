/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef GENERICVOXEL3D_HPP
#define GENERICVOXEL3D_HPP

#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <stdio.h>
#include <string.h>
#include <limits>

namespace skimap
{

/**
     * Generic 3D Voxel representation.
     * V template represents datatype for user data.
     * D template represents datatype for coordinates. 
     */
template <typename V, typename D = double>
struct GenericVoxel3D
{
    D x, y, z;
    V *data;

    /**
         * Void constructor. Sets NULL for user data.
         */
    GenericVoxel3D() : x(D(0)), y(D(0)), z(D(0)), data(NULL)
    {
    }

    /**
         * Full Constructor
         * @param x X coordinate
         * @param y Y coordinate
         * @param z Z coordinate
         * @param data User Data pointer
         */
    GenericVoxel3D(D x, D y, D z, V *data) : x(x), y(y), z(z), data(data)
    {
    }

    /**
         * Full Constructor
         * @param x X coordinate
         * @param y Y coordinate
         * @param z Z coordinate
         * @param data User Data pointer
         */
    GenericVoxel3D(D x, D y, V *data) : x(x), y(y), z(0.0), data(data)
    {
    }

    /**
         * Serializes object into stream.
         */
    friend std::ostream &operator<<(std::ostream &os, const GenericVoxel3D<V, D> &voxel)
    {
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << voxel.x << " ";
        os << voxel.y << " ";
        os << voxel.z << " ";
        if (voxel.data != NULL)
            os << *(voxel.data);
        return os;
    }

    /**
         * Hydrates object from stream.
         */
    friend std::istream &operator>>(std::istream &is, GenericVoxel3D<V, D> &voxel)
    {
        double x, y, z;
        is >> x;
        is >> y;
        is >> z;
        voxel.x = D(x);
        voxel.y = D(y);
        voxel.z = D(z);
        voxel.data = new V();
        is >> (*voxel.data);
        return is;
    }
};
}

#endif /* GENERICVOXEL3D_HPP */
