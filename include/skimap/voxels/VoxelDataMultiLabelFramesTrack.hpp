/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef VOXELDATAMULTILABELFRAMESTRACK_HPP
#define VOXELDATAMULTILABELFRAMESTRACK_HPP

#include <cstddef>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <map>
#include <set>

namespace skimap
{
/**
 * Voxel Data containing Multi-Labels values and relatives Weight and an hidden counter.
 * L template represents datatype for a Label.
 * W template represents datatype for weights.
 */
template <uint32_t LABELS_NUMBER, typename L, typename W, typename F>
struct VoxelDataMultiLabelFramesTrack
{

    W histogram[LABELS_NUMBER];
    W hiddenCounter;
    std::set<F> frames;

    /**
     * Pointer Copy Constructor.
     * @param data source data
     */
    VoxelDataMultiLabelFramesTrack(const VoxelDataMultiLabelFramesTrack *data)
    {
        memcpy(histogram, data->histogram, LABELS_NUMBER * sizeof(W));
        hiddenCounter = data->hiddenCounter;

        this->frames = data->frames;
    }

    /**
     * Void Constructor.
     */
    VoxelDataMultiLabelFramesTrack()
    {
        memset(histogram, 0, LABELS_NUMBER * sizeof(W));
        hiddenCounter = 0;
    }

    /**
     * Constructor with start Label.
     * @param label single Label
     * @param weight label weight
     */
    VoxelDataMultiLabelFramesTrack(L label, W weight, F frame)
    {
        memset(histogram, 0, LABELS_NUMBER * sizeof(W));
        if (label < LABELS_NUMBER)
        {
            histogram[label] = weight;
        }
        this->hiddenCounter = weight;
        this->frames.insert(frame);
    }

    /**
     * Sum Overload. Defines Sum operations between Voxels
     * @param v2 second addend
     * @return sum
     */
    VoxelDataMultiLabelFramesTrack operator+(const VoxelDataMultiLabelFramesTrack &v2) const
    {
        VoxelDataMultiLabelFramesTrack v1(this);

        for (L i = 0; i < LABELS_NUMBER; i++)
        {
            v1.histogram[i] += v2.histogram[i];
            v1.hiddenCounter += v2.histogram[i];
        }

        v1.frames.insert(v2.frames.begin(), v2.frames.end());
        return v1;
    }

    /**
     * Subtraction Overload. Defines Subtraction operations between Voxels
     * @param v2 second minuend
     * @return subtraction
     */
    VoxelDataMultiLabelFramesTrack operator-(const VoxelDataMultiLabelFramesTrack &v2) const
    {
        VoxelDataMultiLabelFramesTrack v1(this);
        for (L i = 0; i < LABELS_NUMBER; i++)
        {
            if (v1.histogram[i] > v2.histogram[i])
            {
                v1.histogram[i] -= v2.histogram[i];
                v1.hiddenCounter -= v2.histogram[i];
            }
            else
            {
                v1.histogram[i] = 0;
                v1.hiddenCounter = 0;
            }
        }

        typename std::set<F>::iterator it;
        for (it = v2.frames.begin(); it != v2.frames.end(); ++it)
        {
            v1.frames.erase(*it);
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
    friend std::ostream &operator<<(std::ostream &os, const VoxelDataMultiLabelFramesTrack<LABELS_NUMBER, L, W, F> &voxel)
    {
        int count = 0;
        os << std::setprecision(std::numeric_limits<double>::digits10 + 2);

        //Frames
        os << voxel.frames.size() << " ";
        typename std::set<F>::iterator it;
        for (it = voxel.frames.begin(); it != voxel.frames.end(); ++it)
        {
            os << *it << " ";
        }

        // Labels
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
    friend std::istream &operator>>(std::istream &is, VoxelDataMultiLabelFramesTrack<LABELS_NUMBER, L, W, F> &voxel)
    {
        //Frames
        int frames_size;
        is >> frames_size;
        for (int i = 0; i < frames_size; i++)
        {
            double f;
            is >> f;
            voxel.frames.insert(F(f));
        }

        //Labels
        int size;
        is >> size;
        for (L i = 0; i < LABELS_NUMBER; i++)
        {
            double w;
            is >> w;
            voxel.histogram[i] = W(w);
            voxel.hiddenCounter += W(w);
        }

        return is;
    }
};
} // namespace skimap

#endif /* VOXELDATAMULTILABEL_HPP */
