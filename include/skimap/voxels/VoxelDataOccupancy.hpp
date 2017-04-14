/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef VOXELDATAOCCUPANCY_HPP
#define VOXELDATAOCCUPANCY_HPP

#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <stdio.h>
#include <string.h>

namespace skimap
{

/**
     * Voxel Data containing RGB color values and Weight.
     * C template represents datatype for color component.
     * W template represents datatype for weight.
     */
template <typename W>
struct VoxelDataOccupancy
{
    W w;

    /**
    * Pointer Copy Constructor.
    * @param data source data
    */
    VoxelDataOccupancy(VoxelDataOccupancy *data)
    {
        w = data->w;
    }

    /**
    * Void constructor.
    */
    VoxelDataOccupancy()
    {
        w = W(0);
    }

    /**
    * Full constructor
    * @param w Weight value
    */
    VoxelDataOccupancy(W w)
    {
        this->w = w;
    }

    /**
    * Sum Overload. Defines Sum operations between Voxels
    * @param v2 second addend
    * @return sum
    */
    VoxelDataOccupancy operator+(const VoxelDataOccupancy &v2) const
    {
        VoxelDataOccupancy c1 = *this;
        if (c1.w == 0)
            return v2;
        VoxelDataOccupancy d;
        d.w = (c1.w + v2.w);
        return d;
    }

    /**
         * Subtraction Overload. Defines Subtraction operations between Voxels
         * @param v2 second minuend
         * @return subtraction
         */
    VoxelDataOccupancy operator-(const VoxelDataOccupancy &v2) const
    {
        VoxelDataOccupancy v1 = *this;
        if (v1.w == 0)
            return v1;
        VoxelDataOccupancy d;
        d.w = (v1.w - v2.w);
        return d;
    }

    /**
         * Serializes object into stream.
         */
    friend std::ostream &operator<<(std::ostream &os, const VoxelDataOccupancy<W> &voxel)
    {
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << double(voxel.w);
        return os;
    }

    /**
         * Hydrates object from stream.
         */
    friend std::istream &operator>>(std::istream &is, VoxelDataOccupancy<W> &voxel)
    {
        double w;
        is >> w;
        voxel.w = w;
        return is;
    }
};
}

#endif /* VOXELDATAOCCUPANCY_HPP */
