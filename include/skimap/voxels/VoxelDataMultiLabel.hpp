/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef VOXELDATAMULTILABEL_HPP
#define VOXELDATAMULTILABEL_HPP

#include <cstddef>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <map>

namespace skimap
{
/**
 * Voxel Data containing Multi-Labels values and relatives Weight.
 * L template represents datatype for a Label.
 * W template represents datatype for weights.
 */
template <uint32_t LABELS_NUMBER, typename L, typename W>
struct VoxelDataMultiLabel
{

    W histogram[LABELS_NUMBER];

    /**
     * Pointer Copy Constructor.
     * @param data source data
     */
    VoxelDataMultiLabel(const VoxelDataMultiLabel *data)
    {
        memcpy(histogram, data->histogram, LABELS_NUMBER * sizeof(W));
    }

    /**
     * Void Constructor.
     */
    VoxelDataMultiLabel()
    {
        memset(histogram, 0, LABELS_NUMBER * sizeof(W));
    }

    /**
     * Constructor with start Label.
     * @param label single Label
     * @param weight label weight
     */
    VoxelDataMultiLabel(L label, W weight)
    {
        memset(histogram, 0, LABELS_NUMBER * sizeof(W));
        if (label < LABELS_NUMBER)
        {
            histogram[label] = weight;
        }
    }

    /**
     * Sum Overload. Defines Sum operations between Voxels
     * @param v2 second addend
     * @return sum
     */
    VoxelDataMultiLabel operator+(const VoxelDataMultiLabel &v2) const
    {
        VoxelDataMultiLabel v1(this);

        for (L i = 0; i < LABELS_NUMBER; i++)
        {
            v1.histogram[i] += v2.histogram[i];
        }
        return v1;
    }

    /**
     * Subtraction Overload. Defines Subtraction operations between Voxels
     * @param v2 second minuend
     * @return subtraction
     */
    VoxelDataMultiLabel operator-(const VoxelDataMultiLabel &v2) const
    {
        VoxelDataMultiLabel v1(this);
        for (L i = 0; i < LABELS_NUMBER; i++)
        {
            if (v1.histogram[i] > v2.histogram[i])
                v1.histogram[i] -= v2.histogram[i];
            else
                v1.histogram[i] = 0;
        }
        return v1;
    }

    /**
     * @return the Label with higher Weight
     */
    L heavierLabel(bool only_positive = false)
    {
        W max = W(0);
        L max_label = L(-1);
        for (L i = 0; i < LABELS_NUMBER; i++)
        {
            //printf("Bin %d = %d \n", i, histogram[i]);
            if (histogram[i] > max)
            {
                max = histogram[i];
                max_label = i;
            }
        }
        //printf("Max %d \n", max_label);
        return max_label;
    }

    /**
     * @return the higher Weight
     */
    W heavierWeight(bool only_positive = false)
    {
        W max = W(0);
        for (L i = 0; i < LABELS_NUMBER; i++)
        {
            if (histogram[i] > max)
            {
                max = histogram[i];
            }
        }
        return max;
    }

    /**
     * @param label target Label
     * @return weight of target label
     */
    W weightOf(L label)
    {
        if (label < LABELS_NUMBER)
            return histogram[label];
        else
            return 0;
    }

    /**
     * Serializes object into stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const VoxelDataMultiLabel<LABELS_NUMBER, L, W> &voxel)
    {
        int count = 0;
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);
        os << LABELS_NUMBER << " ";
        for (L i = 0; i < LABELS_NUMBER; i++)
        {
            os << voxel.histogram[i];
            if (i < LABELS_NUMBER - 1)
            {
                os << " ";
            }
        }
        return os;
    }

    /**
     * Hydrates object from stream.
     */
    friend std::istream &operator>>(std::istream &is, VoxelDataMultiLabel<LABELS_NUMBER, L, W> &voxel)
    {

        int size;
        is >> size;
        for (L i = 0; i < LABELS_NUMBER; i++)
        {
            double w;
            is >> w;
            voxel.histogram[i] = W(w);
        }

        return is;
    }
};
}

#endif /* VOXELDATAMULTILABEL_HPP */
