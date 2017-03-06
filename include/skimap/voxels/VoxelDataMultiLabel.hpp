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

#ifndef VOXELDATAMULTILABEL_HPP
#define VOXELDATAMULTILABEL_HPP

#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <stdio.h>
#include <string.h>
#include <map>

/**
 * Voxel Data containing Multi-Labels values and relatives Weight.
 * L template represents datatype for a Label.
 * W template represents datatype for weights.
 */
template <typename L, typename W>
struct VoxelDataMultiLabel {
    std::map<L, W> labels_map;

    /**
     * Pointer Copy Constructor.
     * @param data source data
     */
    VoxelDataMultiLabel(VoxelDataMultiLabel* data) {
        labels_map = data->labels_map;
    }

    /**
     * Void Constructor.
     */
    VoxelDataMultiLabel() {

    }

    /**
     * Constructor with start Label.
     * @param label single Label
     * @param weight label weight
     */
    VoxelDataMultiLabel(L label, W weight) {
        this->labels_map[label] = weight;
    }

    /**
     * Sum Overload. Defines Sum operations between Voxels
     * @param v2 second addend
     * @return sum
     */
    VoxelDataMultiLabel operator+(const VoxelDataMultiLabel &v2) const {
        VoxelDataMultiLabel v1 = *this;
        if (v1.labels_map.size() <= 0)return v2;
        VoxelDataMultiLabel d;
        d.labels_map = v1.labels_map;
        for (auto it = v2.labels_map.begin(); it != v2.labels_map.end(); ++it) {
            if (d.labels_map.find(it->first) != d.labels_map.end()) {
                d.labels_map[it->first] += it->second;
            } else {
                d.labels_map[it->first] = it->second;
            }
        }
        return d;
    }

    /**
     * Subtraction Overload. Defines Subtraction operations between Voxels
     * @param v2 second minuend
     * @return subtraction
     */
    VoxelDataMultiLabel operator-(const VoxelDataMultiLabel &v2) const {
        VoxelDataMultiLabel v1 = *this;
        if (v1.labels_map.size() <= 0)return v2;
        VoxelDataMultiLabel d;
        d.labels_map = v1.labels_map;
        for (auto it = v2.labels_map.begin(); it != v2.labels_map.end(); ++it) {
            if (d.labels_map.find(it->first) != d.labels_map.end()) {
                d.labels_map[it->first] -= it->second;
            }
        }
        return d;
    }

    /**
     * @return the Label with higher Weight
     */
    L heavierLabel() {
        W max = W(0);
        L max_label = L(-1);
        for (auto it = labels_map.begin(); it != labels_map.end(); ++it) {
            if (it->second > max) {
                max = it->second;
                max_label = it->first;
            }
        }
        return max_label;
    }

    /**
     * @return the higher Weight
     */
    W heavierWeight() {
        W max = W(0);
        for (auto it = labels_map.begin(); it != labels_map.end(); ++it) {
            if (it->second > max) {
                max = it->second;
            }
        }
        return max;
    }
    
    /**
     * @param label target Label
     * @return weight of target label
     */
    W weightOf(L label) {
        if (labels_map.find(label) != labels_map.end()) {
            return labels_map[label];
        }
        return -1.0;
    }

    /**
     * Serializes object into stream.
     */
    friend std::ostream& operator<<(std::ostream &os, const VoxelDataMultiLabel< L, W> &voxel) {
        int count = 0;
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << voxel.labels_map.size() << " ";
        for (auto it = voxel.labels_map.begin(); it != voxel.labels_map.end(); ++it) {
            os << it->first << " " << it->second;
            if (count < voxel.labels_map.size() - 1) {
                os << " ";
            }
            count++;
        }
        return os;
    }

    /**
     * Hydrates object from stream.
     */
    friend std::istream& operator>>(std::istream &is, VoxelDataMultiLabel< L, W> &voxel) {

        int size;
        is >> size;
        for (int i = 0; i < size; i++) {
            double l;
            double w;
            is >> l;
            is >> w;
            voxel.labels_map[L(l)] = W(w);
        }

        return is;
    }

};

#endif /* VOXELDATAMULTILABEL_HPP */

