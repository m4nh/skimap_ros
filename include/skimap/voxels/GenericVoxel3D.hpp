/*
 * Copyright (c) 2017, daniele de gregorio, University of Bologna <d.degregorio at unibo.it>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef GENERICVOXEL3D_HPP
#define GENERICVOXEL3D_HPP

#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <stdio.h>
#include <string.h>
#include <limits>

namespace skimap {

    /**
     * Generic 3D Voxel representation.
     * V template represents datatype for user data.
     * D template represents datatype for coordinates. 
     */
    template <typename V, typename D = double>
    struct GenericVoxel3D {
        D x, y, z;
        V* data;

        /**
         * Void constructor. Sets NULL for user data.
         */
        GenericVoxel3D() : x(D(0)), y(D(0)), z(D(0)), data(NULL) {
        }

        /**
         * Full Constructor
         * @param x X coordinate
         * @param y Y coordinate
         * @param z Z coordinate
         * @param data User Data pointer
         */
        GenericVoxel3D(D x, D y, D z, V* data) :
        x(x), y(y), z(z), data(data) {
        }

        /**
         * Serializes object into stream.
         */
        friend std::ostream& operator<<(std::ostream &os, const GenericVoxel3D< V, D> &voxel) {
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
        friend std::istream& operator>>(std::istream &is, GenericVoxel3D< V, D> &voxel) {
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

