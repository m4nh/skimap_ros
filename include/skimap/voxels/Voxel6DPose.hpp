/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef VOXEL6DPOSE_HPP
#define VOXEL6DPOSE_HPP

#include <cstddef>
#include <iostream>
#include <typeinfo>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <iomanip>
#include <limits>

namespace skimap
{

/**
 * Voxel Data containing matrix-arranged values
 * D template represents datatype for cells.
 * COLUMNS template represents fixed columns size.
 */
struct Voxel6DPose
{
    std::string name;
    std::vector<cv::Vec3f> trans;
    std::vector<cv::Vec4f> rots;

    /**
     * Pointer Copy Constructor.
     * @param data source data
     */
    Voxel6DPose(Voxel6DPose *data)
    {
        name = data->name;
        rots = data->rots;
        trans = data->trans;
    }

    /**
    * Void Constructor.
    */
    Voxel6DPose()
    {
    }

    Voxel6DPose(std::string name)
    {
        this->name = name;
    }

    Voxel6DPose(std::string name, cv::Vec3f trans, cv::Vec4f rot)
    {
        this->rots.push_back(rot);
        this->trans.push_back(trans);
        this->name = name;
    }

    /**
     * Sum Overload. Defines Sum operations between Voxels
     * @param v2 second addend
     * @return sum
     */
    Voxel6DPose operator+(const Voxel6DPose &v2) const
    {
        Voxel6DPose v1 = *this;
        v1.trans.insert(v1.trans.end(), v2.trans.begin(), v2.trans.end());
        v1.rots.insert(v1.rots.end(), v2.rots.begin(), v2.rots.end());
        v1.name = v2.name;
        return v1;
    }

    /**
     * Subtraction Overload. Defines Subtraction operations between Voxels
     * @param v2 second minuend
     * @return subtraction
     */
    Voxel6DPose operator-(const Voxel6DPose &v2) const
    {
        printf("VOXEL6DOF SUBTRACTION: NOT IMPLEMENTED YET\n");
        return *this;
    }

    /**
     * Serializes object into stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const Voxel6DPose &voxel)
    {

        printf("VOXEL6DOF OSTREAM << NOT IMPLEMENTED YET \n");
        return os;
    }

    /**
     * Hydrates object from stream.
     */
    friend std::istream &operator>>(std::istream &is, Voxel6DPose &voxel)
    {

        printf("VOXEL6DOF OSTREAM >> NOT IMPLEMENTED YET \n");
        return is;
    }

    int size()
    {
        return int(trans.size());
    }

    void average(cv::Vec3f& t,cv::Vec4f& r){
        t = cv::Vec3f(.0,.0,.0);
        r = cv::Vec4f(.0,.0,.0,.0);

        for(int i = 0; i < trans.size(); i++){
            t = t + trans[i];
            r = r + rots[i];
        }
        t = t / float(trans.size());
        r = r / float(rots.size());
    }

};
}

#endif /* VOXEL6DPOSE_HPP */
