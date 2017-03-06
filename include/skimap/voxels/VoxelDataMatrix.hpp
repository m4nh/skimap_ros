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

#ifndef VOXELDATAMATRIX_HPP
#define VOXELDATAMATRIX_HPP

#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <iomanip> 
#include <limits>

namespace skimap {

/**
 * Voxel Data containing matrix-arranged values
 * D template represents datatype for cells.
 * COLUMNS template represents fixed columns size.
 */
template <typename D>
struct VoxelDataMatrix {
    std::vector<std::vector<D> > matrix;

    /**
     * Pointer Copy Constructor.
     * @param data source data
     */
    VoxelDataMatrix(VoxelDataMatrix* data) {
        matrix = data->matrix;
    }

    /**
     * Void Constructor.
     */
    VoxelDataMatrix() {

    }

    /**
     * Sum Overload. Defines Sum operations between Voxels
     * @param v2 second addend
     * @return sum
     */
    VoxelDataMatrix operator+(const VoxelDataMatrix &v2) const {
        VoxelDataMatrix v1 = *this;
        if (v1.matrix.size() <= 0)return v2;
        VoxelDataMatrix d;
        d.matrix = v1.matrix;
        for (int i = 0; i < v2.matrix.size(); i++) {
            d.matrix.push_back(v2.matrix[i]);
        }
        return d;
    }

    /**
     * Subtraction Overload. Defines Subtraction operations between Voxels
     * @param v2 second minuend
     * @return subtraction
     */
    VoxelDataMatrix operator-(const VoxelDataMatrix &v2) const {
        VoxelDataMatrix v1 = *this;
        if (v1.matrix.size() <= 0)return v1;
        VoxelDataMatrix d;
        d.matrix = v1.matrix;
        std::vector<std::vector<D> > temp = v2.matrix;
        std::vector<bool> to_delete_mask(d.matrix.size());
        std::fill(to_delete_mask.begin(),to_delete_mask.end(),false);
        while (temp.size() > 0) {
            for (int j = 0; j < d.matrix.size(); j++) {
                D distance = std::numeric_limits<D>::max();
                std::vector<D> r1 = d.matrix[j];
                std::vector<D> r2 = temp[0];
                if (r1.size() == r2.size()) {
                    distance = D(0.0);
                    for (int i = 0; i < r1.size(); i++) {
                        distance += (r1[i] - r2[i])*(r1[i] - r2[i]);
                    }
                    distance = sqrt(distance);
                }
                if (distance <= std::numeric_limits<D>::epsilon()) {
                    to_delete_mask[j] = true;
                    break;
                }
            }
            temp.erase(temp.begin());
        }
        
        VoxelDataMatrix d2;
        for (int i = 0; i < to_delete_mask.size(); i++) {
            if(!to_delete_mask[i]){
                d2.matrix.push_back(d.matrix[i]);
            }
        }

        return d;
    }

    /**
     * 
     * @param row
     * @return 
     */
    int contains(std::vector<D>& row) {
        for (int j = 0; j < matrix.size(); j++) {
            D distance = std::numeric_limits<D>::max();
            std::vector<D> r1 = matrix[j];
            std::vector<D> r2 = row;
            if (r1.size() == r2.size()) {
                distance = D(0.0);
                for (int i = 0; i < r1.size(); i++) {
                    distance += (r1[i] - r2[i])*(r1[i] - r2[i]);
                }
                distance = sqrt(distance);
            }
            if (distance <= std::numeric_limits<D>::epsilon()) {
                return j;
            }
        }
        return -1;
    }

    /**
     * Serializes object into stream.
     */
    friend std::ostream& operator<<(std::ostream &os, const VoxelDataMatrix< D> &voxel) {

        int count = 0;
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << voxel.matrix.size() << " ";
        for (int i = 0; i < voxel.matrix.size(); i++) {
            os << voxel.matrix[i].size() << " ";
            int sub_count = 0;
            for (int j = 0; j < voxel.matrix[i].size(); j++) {
                os << voxel.matrix[i][j];
                if (sub_count < voxel.matrix[i].size() - 1) {
                    os << " ";
                }
                sub_count++;
            }
            if (count < voxel.matrix.size() - 1) {
                os << " ";
            }
            count++;
        }
        return os;
    }

    /**
     * Hydrates object from stream.
     */
    friend std::istream& operator>>(std::istream &is, VoxelDataMatrix< D> &voxel) {

        int size;
        is >> size;
        voxel.matrix.resize(size);
        for (int i = 0; i < size; i++) {
            int row_size;
            is >> row_size;
            std::vector<D> row(row_size);
            for (int j = 0; j < row_size; j++) {
                double d;
                is >> d;
                row[j] = D(d);
            }
            voxel.matrix[i] = row;
        }

        return is;
    }

};
}


#endif /* VOXELDATAMATRIX_HPP */

