/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SKIPLISTGRID_HPP
#define SKIPLISTGRID_HPP

#include <boost/thread.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <skimap/SkipList.hpp>
#include <skimap/voxels/GenericVoxel3D.hpp>
#include <vector>

#define SKIPLISTGRID_MAX_DEPTH 16

namespace skimap
{

/**
     *
     * @param min_index
     * @param max_index
     */
template <class V, class K, class D, int X_DEPTH = 8, int Y_DEPTH = 8>
class SkipListGrid
{
public:
  typedef GenericVoxel3D<V, D> Voxel2D;

  /**
       * Batch integration entry
       */
  struct IntegrationEntry
  {
    K x, y;
    V *data;
    int entry_depth;

    IntegrationEntry(K x, K y, V *data, int max_depth = 3)
        : x(x), y(y), data(data), entry_depth(max_depth) {}
  };

  /**
       * Integration map for batch OMP integration
       */
  struct IntegrationMap
  {
    std::map<K, std::vector<IntegrationEntry>> map;
    std::vector<K> map_keys;

    IntegrationMap() {}

    void addEntry(K x, K y, V *data, int max_depth = 3)
    {
      IntegrationEntry entry(x, y, data, max_depth);
      if (map.find(x) == map.end())
      {
        map[x] = std::vector<IntegrationEntry>();
        map_keys.push_back(x);
      }
      map[x].push_back(entry);
    }

    void clear()
    {
      map.clear();
      map_keys.clear();
    }
  };

  typedef K Index;
  typedef SkipList<Index, V *, Y_DEPTH> Y_NODE;
  typedef SkipList<Index, Y_NODE *, X_DEPTH> X_NODE;

  /**
       *
       * @param min_index
       * @param max_index
       * @param resolution_x
       * @param resolution_y
       */
  SkipListGrid(K min_index, K max_index, D resolution_x, D resolution_y)
      : _min_index_value(min_index), _max_index_value(max_index),
        _resolution_x(resolution_x), _resolution_y(resolution_y),
        _voxel_counter(0), _xlist_counter(0), _ylist_counter(0),
        _bytes_counter(0), _batch_integration(false), _initialized(false),
        _self_concurrency_management(false)
  {
    initialize(_min_index_value, _max_index_value);
  }

  /**
       */
  SkipListGrid(D resolution)
      : _min_index_value(std::numeric_limits<K>::min()),
        _max_index_value(std::numeric_limits<K>::max()),
        _resolution_x(resolution), _resolution_y(resolution), _voxel_counter(0),
        _xlist_counter(0), _ylist_counter(0), _bytes_counter(0),
        _batch_integration(false), _initialized(false),
        _self_concurrency_management(false)
  {
    initialize(_min_index_value, _max_index_value);
  }

  /**
       */
  SkipListGrid()
      : _min_index_value(std::numeric_limits<K>::min()),
        _max_index_value(std::numeric_limits<K>::max()), _resolution_x(0.01),
        _resolution_y(0.01), _voxel_counter(0), _xlist_counter(0),
        _ylist_counter(0), _bytes_counter(0), _batch_integration(false),
        _initialized(false), _self_concurrency_management(false) {}

  /**
       *
       */
  virtual ~SkipListGrid()
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
    _root_list = new X_NODE(min_index, max_index);
    _bytes_counter += sizeof(X_NODE);
  }

  /**
       *
       * @param ix
       * @param iy
       * @return
       */
  virtual bool isValidIndex(K ix, K iy)
  {
    bool result = true;
    result &= ix <= _max_index_value && ix >= _min_index_value;
    result &= iy <= _max_index_value && iy >= _min_index_value;
    return result;
  };

  /**
       *
       * @param x
       * @param y
       * @param ix
       * @param iy
       * @return
       */
  virtual bool coordinatesToIndex(D x, D y, K &ix, K &iy)
  {
    ix = K(floor(x / _resolution_x));
    iy = K(floor(y / _resolution_y));
    return isValidIndex(ix, iy);
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
       * @param ix
       * @param iy
       * @param x
       * @param y
       * @return
       */
  virtual bool indexToCoordinates(K ix, K iy, D &x, D &y)
  {
    x = ix * _resolution_x + _resolution_x * 0.5;
    y = iy * _resolution_y + _resolution_y * 0.5;
    return true;
  }

  /**
       *
       * @param ix
       * @param iy
       * @return
       */
  virtual V *find(K ix, K iy)
  {
    const typename X_NODE::NodeType *ylist = _root_list->find(ix);
    if (ylist != NULL && ylist->value != NULL)
    {
      const typename Y_NODE::NodeType *tile = ylist->value->find(iy);
      if (tile != NULL)
      {
        return tile->value;
      }
    }
    return NULL;
  }

  /**
       *
       * @param x
       * @param y
       * @return
       */
  virtual V *find(D x, D y)
  {
    K ix, iy;
    if (coordinatesToIndex(x, y, ix, iy))
    {
      return find(ix, iy);
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
  virtual bool integrateVoxel(D x, D y, V *data)
  {

    K ix, iy;
    if (coordinatesToIndex(x, y, ix, iy))
    {
      return integrateVoxel(ix, iy, data);
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
  virtual bool integrateVoxel(K ix, K iy, V *data)
  {
    if (isValidIndex(ix, iy))
    {

      if (this->hasConcurrencyAccess())
        this->lockMap(ix);

      const typename X_NODE::NodeType *ylist = _root_list->find(ix);
      if (ylist == NULL)
      {
        ylist = _root_list->insert(
            ix, new Y_NODE(_min_index_value, _max_index_value));
      }
      const typename Y_NODE::NodeType *tile = ylist->value->find(iy);
      if (tile == NULL)
      {
        tile = ylist->value->insert(iy, new V(data));
      }
      else
      {
        *(tile->value) = *(tile->value) + *data;
      }

      if (this->hasConcurrencyAccess())
        this->unlockMap(ix);

      return true;
    }
    return false;
  }

  /**
       *
       * @return
       */
  virtual bool startBatchIntegration()
  {
    _batch_integration = true;
    _current_integration_map.clear();
    return true;
  }

  /**
       *
       * @param voxels
       */
  virtual void fetchVoxels(std::vector<Voxel2D> &voxels)
  {
    voxels.clear();
    std::vector<typename X_NODE::NodeType *> xnodes;
    _root_list->retrieveNodes(xnodes);

#pragma omp parallel
    {
      std::vector<Voxel2D> voxels_private;

#pragma omp for nowait
      for (int i = 0; i < xnodes.size(); i++)
      {
        K ix, iy;
        D x, y;
        std::vector<typename Y_NODE::NodeType *> ynodes;
        xnodes[i]->value->retrieveNodes(ynodes);
        for (int j = 0; j < ynodes.size(); j++)
        {
          ix = xnodes[i]->key;
          iy = ynodes[j]->key;
          indexToCoordinates(ix, iy, x, y);
          voxels_private.push_back(Voxel2D(x, y, ynodes[j]->value));
        }
      }

#pragma omp critical
      voxels.insert(voxels.end(), voxels_private.begin(), voxels_private.end());
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
  virtual void radiusSearch(K cx, K cy, K radiusx, K radiusy,
                            std::vector<Voxel2D> &voxels, bool boxed = false)
  {
    voxels.clear();
    std::vector<typename X_NODE::NodeType *> xnodes;

    K ix_min = cx - radiusx;
    K ix_max = cx + radiusx;
    K iy_min = cy - radiusy;
    K iy_max = cy + radiusy;

    _root_list->retrieveNodesByRange(ix_min, ix_max, xnodes);

    D rx, ry, radius;
    D centerx, centery;

    indexToCoordinates(radiusx, radiusy, rx, ry);
    indexToCoordinates(cx, cy, centerx, centery);
    radius = (rx + ry) / 2.0;

#pragma omp parallel
    {
      std::vector<Voxel2D> voxels_private;

#pragma omp for nowait
      for (int i = 0; i < xnodes.size(); i++)
      {
        K ix, iy;
        D x, y;
        D distance;
        std::vector<typename Y_NODE::NodeType *> ynodes;
        xnodes[i]->value->retrieveNodesByRange(iy_min, iy_max, ynodes);
        for (int j = 0; j < ynodes.size(); j++)
        {
          ix = xnodes[i]->key;
          iy = ynodes[j]->key;

          indexToCoordinates(ix, iy, x, y);
          if (!boxed)
          {
            distance = sqrt(pow(centerx - x, 2) + pow(centery - y, 2));
            if (distance > radius)
              continue;
          }
          voxels_private.push_back(Voxel2D(x, y, ynodes[j]->value));
        }
      }

#pragma omp critical
      voxels.insert(voxels.end(), voxels_private.begin(), voxels_private.end());
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
  virtual void radiusSearch(D cx, D cy, D radiusx, D radiusy,
                            std::vector<Voxel2D> &voxels, bool boxed = false)
  {
    K ix, iy;
    if (coordinatesToIndex(cx, cy, ix, iy))
    {
      K iradiusx, iradiusy;
      iradiusx = K(floor(radiusx / _resolution_x));
      iradiusy = K(floor(radiusy / _resolution_y));
      radiusSearch(ix, iy, iradiusx, iradiusy, voxels, boxed);
    }
  }

  /**
       *
       * @return
       */
  virtual long sizeInBytes() { return _bytes_counter; }

  /**
       *
       * @param filename
       */
  virtual void saveToFile(std::string filename)
  {
    std::vector<Voxel2D> voxels;
    fetchVoxels(voxels);

    std::ofstream f;
    f.open(filename);
    V a1;
    K a2;
    D a3;
    f << "# SkipListGrid<" << typeid(a1).name() << "," << typeid(a2).name()
      << "," << typeid(a3).name() << ">" << std::endl;
    f << _min_index_value << " " << _max_index_value << " ";
    f << _resolution_x << " ";
    f << _resolution_y << " ";
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
    std::ifstream input_file(filename.c_str());
    if (input_file.is_open())
    {
    }
    // Load Files
    std::string line;

    bool header_found = false;

    while (std::getline(input_file, line))
    {
      std::istringstream iss(line);
      if (line.length() > 0 && line.at(0) == '#')
        continue;

      if (!header_found)
      {
        K min, max;
        iss >> min;
        iss >> max;
        iss >> _resolution_x;
        iss >> _resolution_y;
        initialize(min, max);
        header_found = true;
        continue;
      }

      if (header_found)
      {
        Voxel2D voxel;
        iss >> voxel;
        this->integrateVoxel(voxel.x, voxel.y, voxel.data);
      }
    }
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
  X_NODE *_root_list;
  D _resolution_x;
  D _resolution_y;
  int _voxel_counter;
  int _xlist_counter;
  int _ylist_counter;
  long _bytes_counter;
  bool _batch_integration;
  bool _initialized;
  bool _self_concurrency_management;
  IntegrationMap _current_integration_map;

  // concurrency
  boost::mutex mutex_map_mutex;
  boost::mutex mutex_super_lock;
  std::map<K, boost::mutex *> mutex_map;
};
}

#endif /* SKIPLISTGRID_HPP */
