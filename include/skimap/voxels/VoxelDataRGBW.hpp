/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef VOXELDATARGBW_HPP
#define VOXELDATARGBW_HPP

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
template <typename C, typename W>
struct VoxelDataRGBW
{
    C r, g, b;
    W w;

    /**
         * Pointer Copy Constructor.
         * @param data source data
         */
    VoxelDataRGBW(VoxelDataRGBW *data)
    {
        r = data->r;
        g = data->g;
        b = data->b;
        w = data->w;
    }

    /**
         * Void constructor.
         */
    VoxelDataRGBW()
    {
        r = g = b = C(0);
        w = W(0);
    }

    /**
         * Gray color constructor.
         * @param v RGB level
         */
    VoxelDataRGBW(C v)
    {
        r = g = b = v;
        w = W(0);
    }

    /**
         * Full constructor
         * @param r Red value 
         * @param g Green value
         * @param b Blue value
         * @param w Weight value
         */
    VoxelDataRGBW(C r, C g, C b, W w)
    {
        this->r = r;
        this->g = g;
        this->b = b;
        this->w = w;
    }

    /**
         * Sum Overload. Defines Sum operations between Voxels
         * @param v2 second addend
         * @return sum
         */
    VoxelDataRGBW operator+(const VoxelDataRGBW &v2) const
    {
        VoxelDataRGBW c1 = *this;
        if (c1.w <= W(0))
            return v2;
        VoxelDataRGBW d;
        d.w = (c1.w + v2.w);
        if (d.w <= W(0))
        {
            d.r = d.g = d.b = C(0);
            d.w = W(0);
        }
        else
        {
            d.r = C((double(c1.r) * double(c1.w) + double(v2.r) * double(v2.w)) / double(d.w));
            d.g = C((double(c1.g) * double(c1.w) + double(v2.g) * double(v2.w)) / double(d.w));
            d.b = C((double(c1.b) * double(c1.w) + double(v2.b) * double(v2.w)) / double(d.w));
        }
        return d;
    }

    /**
         * Subtraction Overload. Defines Subtraction operations between Voxels
         * @param v2 second minuend
         * @return subtraction
         */
    VoxelDataRGBW operator-(const VoxelDataRGBW &v2) const
    {
        VoxelDataRGBW v1 = *this;
        if (v1.w <= W(0))
            return v1;
        VoxelDataRGBW d;
        d.w = (v1.w - v2.w);
        if (d.w <= W(0))
        {
            d.r = d.g = d.b = C(0);
            d.w = W(0);
        }
        else
        {
            d.r = C((double(v1.r) * double(v1.w) - double(v2.r) * double(v2.w)) / double(d.w));
            d.g = C((double(v1.g) * double(v1.w) - double(v2.g) * double(v2.w)) / double(d.w));
            d.b = C((double(v1.b) * double(v1.w) - double(v2.b) * double(v2.w)) / double(d.w));
        }
        return d;
    }

    /**
         * Serializes object into stream.
         */
    friend std::ostream &operator<<(std::ostream &os, const VoxelDataRGBW<C, W> &voxel)
    {
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << double(voxel.r) << " ";
        os << double(voxel.g) << " ";
        os << double(voxel.b) << " ";
        os << double(voxel.w);
        return os;
    }

    /**
         * Hydrates object from stream.
         */
    friend std::istream &operator>>(std::istream &is, VoxelDataRGBW<C, W> &voxel)
    {
        double r, g, b, w;
        is >> r;
        is >> g;
        is >> b;
        is >> w;
        voxel.r = r;
        voxel.g = g;
        voxel.b = b;
        voxel.w = w;
        return is;
    }
};
}

#endif /* VOXELDATARGBW_HPP */
