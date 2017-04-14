/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include "mapping/SkipVolume.h"

namespace slamdunk
{

SkipVolume::SkipVolume(SkipIndex min_index, SkipIndex max_index, cv::Vec3f resolution)
    : _min_index_value(min_index), _max_index_value(max_index), _resolution(resolution), _voxel_counter(0), _bytes_counter(0)
{

    _root_list = new SkipListX(min_index, max_index);
}
///////////////////////////////////////////////////////////////////////////////

SkipVolume::SkipVolume(const SkipVolume &orig)
{
}
///////////////////////////////////////////////////////////////////////////////

SkipVolume::~SkipVolume()
{
}
///////////////////////////////////////////////////////////////////////////////

bool SkipVolume::coordinatesToIndex(double x, double y, double z, SkipIndex &ix, SkipIndex &iy, SkipIndex &iz)
{
    ix = SkipIndex(x / _resolution[0]);
    iy = SkipIndex(y / _resolution[1]);
    iz = SkipIndex(z / _resolution[2]);
    bool result = true;
    result &= ix <= _max_index_value && ix >= _min_index_value;
    result &= iy <= _max_index_value && iy >= _min_index_value;
    result &= iz <= _max_index_value && iz >= _min_index_value;
    return result;
}
///////////////////////////////////////////////////////////////////////////////

bool SkipVolume::indexToCoordinates(SkipIndex ix, SkipIndex iy, SkipIndex iz, double &x, double &y, double &z)
{
    x = ix * _resolution[0];
    y = iy * _resolution[1];
    z = iz * _resolution[2];
}

///////////////////////////////////////////////////////////////////////////////

SkipVoxel *SkipVolume::findVoxel(SkipIndex &ix, SkipIndex &iy, SkipIndex &iz)
{
    //        std::cout << "Finding voxel (" << ix << "," << iy << "," << iz << ") \n";
    const SkipListX::NodeType *ylist = _root_list->find(ix);
    if (ylist != NULL && ylist->value != NULL)
    {
        const SkipListY::NodeType *zlist = ylist->value->find(iy);
        if (zlist != NULL && zlist->value != NULL)
        {
            const SkipListZ::NodeType *voxel = zlist->value->find(iz);
            if (voxel != NULL)
            {
                return voxel->value;
            }
        }
    }
    return NULL;
}

///////////////////////////////////////////////////////////////////////////////

SkipVoxel *SkipVolume::findVoxel(double x, double y, double z)
{
    SkipIndex ix, iy, iz;
    if (coordinatesToIndex(x, y, z, ix, iy, iz))
    {
        return findVoxel(ix, iy, iz);
    }
    return NULL;
}

///////////////////////////////////////////////////////////////////////////////

bool SkipVolume::integrateVoxel(SkipIndex &ix, SkipIndex &iy, SkipIndex &iz, short weight)
{
    //        std::cout << "Integrating voxel (" << ix << "," << iy << "," << iz << ") \n";
    const SkipListX::NodeType *ylist = _root_list->find(ix);
    if (ylist == NULL)
    {
        ylist = _root_list->insert(ix, new SkipListY(_min_index_value, _max_index_value));
        _bytes_counter += sizeof(SkipListX::NodeType) + sizeof(SkipListY);
    }
    const SkipListY::NodeType *zlist = ylist->value->find(iy);
    if (zlist == NULL)
    {
        zlist = ylist->value->insert(iy, new SkipListZ(_min_index_value, _max_index_value));
        _bytes_counter += sizeof(SkipListY::NodeType) + sizeof(SkipListZ);
    }
    const SkipListZ::NodeType *voxel = zlist->value->find(iz);
    if (voxel == NULL)
    {
        voxel = zlist->value->insert(iz, new SkipVoxel);
        _voxel_counter++;
        _bytes_counter += sizeof(SkipListZ::NodeType) + sizeof(SkipVoxel);
    }
    voxel->value->integrateWeight(weight);
    if (voxel->value->counter <= 0)
    {
        zlist->value->erase(iz);
        _voxel_counter--;
        _bytes_counter -= sizeof(SkipListZ::NodeType) + sizeof(SkipVoxel);
    }
    return true;
}

///////////////////////////////////////////////////////////////////////////////

bool SkipVolume::integrateVoxel(double x, double y, double z, short weight)
{
    SkipIndex ix, iy, iz;
    //            std::cout << "Integrating ("<<x<<","<<y<<","<<z<<")\n";
    if (coordinatesToIndex(x, y, z, ix, iy, iz))
    {
        return integrateVoxel(ix, iy, iz, weight);
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////

int SkipVolume::size()
{
    return _voxel_counter;
}

///////////////////////////////////////////////////////////////////////////////

int SkipVolume::sizeInBytes()
{
    return _bytes_counter;
}

///////////////////////////////////////////////////////////////////////////////

int SkipVolume::toVector(std::vector<SkipVoxelExtended> &voxels)
{
    voxels.clear();
    std::vector<SkipListX::NodeType *> xnodes;
    _root_list->retrieveNodes(xnodes);
    SkipIndex ix, iy, iz;
    double x, y, z;
    int bytes = 0;
    for (int i = 0; i < xnodes.size(); i++)
    {
        std::vector<SkipListY::NodeType *> ynodes;
        xnodes[i]->value->retrieveNodes(ynodes);
        bytes += 78;
        for (int j = 0; j < ynodes.size(); j++)
        {
            std::vector<SkipListZ::NodeType *> znodes;
            ynodes[j]->value->retrieveNodes(znodes);
            bytes += 78;
            for (int k = 0; k < znodes.size(); k++)
            {
                ix = xnodes[i]->key;
                iy = ynodes[j]->key;
                iz = znodes[k]->key;
                indexToCoordinates(ix, iy, iz, x, y, z);
                voxels.push_back(SkipVoxelExtended(*(znodes[k]->value), x, y, z));
                bytes += 36;
            }
        }
    }
    return bytes;
}
}