/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef KDSKIPLIST_HPP
#define KDSKIPLIST_HPP

#include <boost/thread.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <skimap/SkipList.hpp>
#include <skimap/voxels/GenericVoxelKD.hpp>
#include <vector>

namespace skimap
{

template <class V, class K, class D, int DEPTH = 8>
class KDSkipList
{
  public:
    typedef GenericVoxelKD<V, D> VoxelKD;

    typedef K Index;
    typedef SkipList<Index, void *, DEPTH> KNODE;
    typedef SkipList<Index, V *, DEPTH> FINAL_NODE;

    typedef std::vector<Index> Indices;
    typedef std::vector<D> Coordinates;

    /**
           */
    KDSkipList(int DIM, D resolution)
        : DIM(DIM),
          _min_index_value(std::numeric_limits<K>::min()),
          _max_index_value(std::numeric_limits<K>::max()),
          _resolution(resolution),
          _initialized(false),
          _self_concurrency_management(false)
    {
        initialize(_min_index_value, _max_index_value);
    }

    /**
           *
           */
    virtual ~KDSkipList()
    {
        for (typename std::map<K, boost::mutex *>::iterator it =
                 this->mutex_map.begin();
             it != this->mutex_map.end(); ++it)
        {
            delete it->second;
        }
    }

    /**
           */
    void initialize(K min_index, K max_index)
    {
        if (_initialized)
        {
            delete _root_list;
        }
        _root_list = new KNODE(min_index, max_index);
    }

    /////
    virtual bool isValidIndices(Indices &idx)
    {
        if (idx.size() != DIM)
            return false;
        bool valid = true;
        for (size_t i = 0; i < idx.size(); i++)
        {
            valid &= idx[i] <= _max_index_value && idx[i] >= _min_index_value;
        }
        return valid;
    }

    /**
           *
           * @param cds
           * @param idx  RESIZE IT BY YOURSELF!
           * @return
           */
    virtual bool coordinatesToIndex(Coordinates cds, Indices &idx)
    {
        //dx.resize(cds.size());
        for (size_t i = 0; i < cds.size(); i++)
        {
            idx[i] = K(floor(cds[i] / _resolution));
        }
        return isValidIndices(idx);
    }

    /**
           *
           * @param index
           * @param coordinate
           * @param resolution
           * @return
           */
    virtual bool singleIndexToCoordinate(K index, D &coordinate, D resolution)
    {
        coordinate = index * resolution + resolution * 0.5;
    }

    /**
           *
           * @param idx
           * @param cds RESIZE IT BY YOURSELF!
           * @return
           */
    virtual bool indexToCoordinates(const Indices &idx, Coordinates &cds)
    {
        for (size_t i = 0; i < idx.size(); i++)
        {
            cds[i] = idx[i] * _resolution + _resolution * 0.5;
        }
        return true;
    }

    /**
           *
           * @param ix
           * @param iy
           * @return
           */
    virtual V *find(Indices &idx)
    {
        if (!this->isValidIndices(idx))
            return NULL;

        typedef const typename KNODE::NodeType Node;

        const Node *node = _root_list->find(idx[0]);

        int pointer = 1;
        while (pointer < DIM)
        {
            K current_index = idx[pointer];
            if (node != NULL)
            {
                if (pointer == DIM - 1)
                {
                    return reinterpret_cast<V *>(reinterpret_cast<Node *>(node)->value);
                }
                else
                {
                    node = reinterpret_cast<KNODE *>(reinterpret_cast<Node *>(node)->value)->find(current_index);
                }
            }
            else
            {
                break;
            }
            pointer++;
        }
        return NULL;
    }

    /**
           *
           * @param x
           * @param y
           * @return
           */
    virtual V *find(Coordinates &cds)
    {
        Indices idx(cds.size());
        if (coordinatesToIndex(cds, idx))
        {
            return find(idx);
        }
        return NULL;
    }

    /**
      * Lock concurrency access to a X branch
      * @param key x index
      * @return
      */
    virtual void lockMap(K key)
    {
        this->mutex_map_mutex.lock();
        if (this->mutex_map.count(key) == 0)
        {
            mutex_map[key] = new boost::mutex();
        }
        this->mutex_map_mutex.unlock();
        mutex_map[key]->lock();
    }

    /**
      * UnLock concurrency access to a X branch
      * @param key x index
      * @return
      */
    virtual void unlockMap(K key)
    {
        this->mutex_map_mutex.lock();
        if (this->mutex_map.count(key) > 0)
        {
            mutex_map[key]->unlock();
        }
        this->mutex_map_mutex.unlock();
    }

    /**
           *
           * @param x
           * @param y
           * @param z
           * @param data
           * @return
           */
    virtual bool integrateVoxel(Coordinates &cds, V *data)
    {

        Indices idx(cds.size());
        if (coordinatesToIndex(cds, idx))
        {
            return integrateVoxel(idx, data);
        }
        return false;
    }

    /**
           *
           * @param ix
           * @param iy
           * @param data
           * @return
           */
    virtual bool integrateVoxel(Indices &idx, V *data)
    {

        typedef const typename KNODE::NodeType Node;

        if (isValidIndices(idx))
        {

            if (this->hasConcurrencyAccess())
                this->lockMap(idx[0]);

            KNODE *current = _root_list;
            const Node *node = current->find(idx[0]);

            int pointer = 0;
            while (pointer < DIM)
            {
                K current_index = idx[(pointer) % idx.size()];
                K next_index = idx[(pointer + 1) % idx.size()];
                if (node == NULL)
                {
                    if (pointer == DIM - 1)
                    {
                        node = current->insert(current_index, new V);
                    }
                    else
                    {
                        node = current->insert(current_index, new KNODE(_min_index_value, _max_index_value));
                    }
                }

                if (pointer < DIM - 1)
                {
                    current = reinterpret_cast<KNODE *>(node->value);
                }

                if (pointer == DIM - 1)
                {
                    *((V *)node->value) = *((V *)node->value) + *data;
                }
                else
                {
                    node = reinterpret_cast<KNODE *>(node->value)->find(next_index);
                }

                pointer++;
            }

            if (this->hasConcurrencyAccess())
                this->unlockMap(idx[0]);

            return true;
        }
        return false;
    }

    /**
           *
           * @param voxels
           */
    virtual void fetchVoxels(std::vector<VoxelKD> &voxels, Indices min_idx = Indices(0), Indices max_idx = Indices(0))
    {
        Indices idx(DIM);

        std::vector<typename KNODE::NodeType *> temp_nodes;
        if (min_idx.size() == idx.size() && max_idx.size() == idx.size())
        {
            _root_list->retrieveNodesByRange(min_idx[0], max_idx[0], temp_nodes);
        }
        else
        {
            _root_list->retrieveNodes(temp_nodes);
        }

#pragma omp parallel
        {
            std::vector<VoxelKD> voxels_private;

#pragma omp for nowait
            for (int i = 0; i < temp_nodes.size(); i++)
            {
                Indices idx(DIM);
                idx[0] = reinterpret_cast<typename KNODE::NodeType *>(temp_nodes[i])->key;
                fetchDimension(reinterpret_cast<KNODE *>(temp_nodes[i]->value), idx, voxels_private, 1, min_idx, max_idx);
            }

#pragma omp critical
            voxels.insert(voxels.end(), voxels_private.begin(), voxels_private.end());
        }
    }

    void fetchDimension(KNODE *root, Indices idx, std::vector<VoxelKD> &voxels, int current_dim, Indices min_idx = Indices(0), Indices max_idx = Indices(0))
    {
        if (current_dim < DIM - 1)
        {
            std::vector<typename KNODE::NodeType *> temp_nodes;
            if (root != NULL)
            {
                if (min_idx.size() == idx.size() && max_idx.size() == idx.size())
                {
                    root->retrieveNodesByRange(min_idx[current_dim], max_idx[current_dim], temp_nodes);
                }
                else
                {
                    root->retrieveNodes(temp_nodes);
                }
                for (int i = 0; i < temp_nodes.size(); i++)
                {
                    idx[current_dim] = reinterpret_cast<typename KNODE::NodeType *>(temp_nodes[i])->key;
                    this->fetchDimension(reinterpret_cast<KNODE *>(temp_nodes[i]->value), idx, voxels, current_dim + 1);
                }
            }
        }
        else
        {
            std::vector<typename KNODE::NodeType *> temp_nodes;
            if (min_idx.size() == idx.size() && max_idx.size() == idx.size())
            {
                root->retrieveNodesByRange(min_idx[current_dim], max_idx[current_dim], temp_nodes);
            }
            else
            {
                root->retrieveNodes(temp_nodes);
            }
            for (int i = 0; i < temp_nodes.size(); i++)
            {
                idx[idx.size() - 1] = reinterpret_cast<typename KNODE::NodeType *>(temp_nodes[i])->key;
                Coordinates cds(idx.size());
                this->indexToCoordinates(idx, cds);
                voxels.push_back(
                    VoxelKD(
                        cds, reinterpret_cast<V *>(reinterpret_cast<typename KNODE::NodeType *>(temp_nodes[i])->value)));
            }
        }
    }

    /**
           * Radius search
           * @param cx
           * @param cy
           * @param radiusx
           * @param radiusy
           * @param voxels
           * @param boxed
           */
    virtual void radiusSearch(Indices center, K radius,
                              std::vector<VoxelKD> &voxels, bool boxed = false)
    {
        Indices min_idx(DIM);
        Indices max_idx(DIM);
        for (int i = 0; i < DIM; i++)
        {
            min_idx[i] = center[i] - radius;
            max_idx[i] = center[i] + radius;
        }

        std::vector<VoxelKD> temp_voxels;
        this->fetchVoxels(temp_voxels, min_idx, max_idx);

        if (boxed)
        {
            voxels = temp_voxels;
            return;
        }

        K squaredRadius = radius * radius;
#pragma omp parallel
        {
            std::vector<VoxelKD> filtered_voxels;

#pragma omp for nowait
            for (int i = 0; i < temp_voxels.size(); i++)
            {
                Indices voxel_center(DIM);
                coordinatesToIndex(temp_voxels[i].coordinates, voxel_center);
                K d = indicesSquaredDistance(voxel_center, center);
                if (d <= squaredRadius)
                {
                    filtered_voxels.push_back(temp_voxels[i]);
                }
            }

#pragma omp critical
            voxels.insert(voxels.end(), filtered_voxels.begin(), filtered_voxels.end());
        }
    }

    /**
           *
           * @param cx
           * @param cy
           * @param cz
           * @param radiusx
           * @param radiusy
           * @param radiusz
           * @param voxels
           * @param boxed
           */
    virtual void radiusSearch(Coordinates center, D radius,
                              std::vector<VoxelKD> &voxels, bool boxed = false)
    {
        Indices icenter(DIM);
        K iradius;
        coordinatesToIndex(center, icenter);
        iradius = K(floor(radius / _resolution));
        this->radiusSearch(icenter, iradius, voxels, boxed);
    }

    /**
           *
           * @param filename
           */
    virtual void saveToFile(std::string filename)
    {
        std::vector<VoxelKD> voxels;
        fetchVoxels(voxels);

        std::ofstream f;
        f.open(filename);
        V a1;
        K a2;
        D a3;
        f << "# KDSkipList<" << typeid(a1).name() << "," << typeid(a2).name()
          << "," << typeid(a3).name() << ">" << std::endl;
        f << _min_index_value << " " << _max_index_value << " ";
        f << _resolution << " ";
        for (int i = 0; i < voxels.size(); i++)
        {
            f << voxels[i] << std::endl;
        }
        f.close();
    }

    /**
           */
    virtual void loadFromFile(std::string filename)
    {
        // std::ifstream input_file(filename.c_str());
        // if (input_file.is_open())
        // {
        // }
        // // Load Files
        // std::string line;

        // bool header_found = false;

        // while (std::getline(input_file, line))
        // {
        //     std::istringstream iss(line);
        //     if (line.length() > 0 && line.at(0) == '#')
        //         continue;

        //     if (!header_found)
        //     {
        //         K min, max;
        //         iss >> min;
        //         iss >> max;
        //         iss >> _resolution;
        //         initialize(min, max);
        //         header_found = true;
        //         continue;
        //     }

        //     if (header_found)
        //     {
        //         VoxelKD voxel;
        //         iss >> voxel;
        //         this->integrateVoxel(voxel.x, voxel.y, voxel.data);
        //     }
        // }
    }

    virtual K indicesSquaredDistance(const Indices &i1, const Indices &i2)
    {
        K distance(0);
        for (int i = 0; i < i1.size(); i++)
        {
            distance += pow(i1[i] - i2[i], 2);
        }
        return distance;
    }

    virtual void enableConcurrencyAccess(bool status = true)
    {
        this->_self_concurrency_management = status;
    }

    virtual bool hasConcurrencyAccess()
    {
        return this->_self_concurrency_management;
    }

  protected:
    Index _max_index_value;
    Index _min_index_value;
    KNODE *_root_list;
    D _resolution;
    bool _initialized;
    bool _self_concurrency_management;
    int DIM;

    // concurrency
    boost::mutex mutex_map_mutex;
    boost::mutex mutex_super_lock;
    std::map<K, boost::mutex *> mutex_map;
};
}

#endif /* KDSKIPLIST_HPP */