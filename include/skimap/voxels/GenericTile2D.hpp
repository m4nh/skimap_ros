/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef GENERICTILE2D_HPP
#define GENERICTILE2D_HPP
#include <skimap/voxels/GenericVoxel3D.hpp>

namespace skimap
{

/**
     * Generic 3D Voxel representation.
     * V template represents datatype for user data.
     * D template represents datatype for coordinates. 
     */
template <typename V, typename D = double>
struct GenericTile2D : public GenericVoxel3D<V, D>
{

    /**
         * Void constructor. Sets NULL for user data.
         */
    GenericTile2D() : GenericVoxel3D<V, D>(0, 0, 0, NULL)
    {
    }

    /**
         * Full Constructor
         * @param x X coordinate
         * @param y Y coordinate
         * @param z Z coordinate
         * @param data User Data pointer
         */
    GenericTile2D(D x, D y, D z, V *data) : GenericVoxel3D<V, D>(x, y, z, data)
    {
    }

    /**
         * Serializes object into stream.
         */
    friend std::ostream &operator<<(std::ostream &os, const GenericTile2D<V, D> &voxel)
    {
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << voxel.x << " ";
        os << voxel.y << " ";
        os << voxel.z << " ";
        os << 1 << " ";
        if (voxel.data != NULL)
            os << *(voxel.data);
        return os;
    }

    /**
         * Hydrates object from stream.
         */
    friend std::istream &operator>>(std::istream &is, GenericTile2D<V, D> &voxel)
    {
        double x, y, z;
        int t;
        is >> x;
        is >> y;
        is >> z;
        is >> t;
        voxel.x = D(x);
        voxel.y = D(y);
        voxel.z = D(z);
        voxel.data = new V();
        is >> (*voxel.data);
        return is;
    }
};
}

#endif /* GENERICTILE2D_HPP */
