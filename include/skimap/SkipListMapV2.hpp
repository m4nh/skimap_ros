/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SkipListMapV2_HPP
#define SkipListMapV2_HPP

#include <boost/thread.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <omp.h>
#include <skimap/SkipList.hpp>
#include <skimap/SkipListDense.hpp>
#include <skimap/voxels/GenericVoxel3D.hpp>
#include <vector>

#define SkipListMapV2_MAX_DEPTH 16

namespace skimap {

/**
     *
     * @param min_index
     * @param max_index
     */
template <class V, class K, class D, int X_DEPTH = 8, int Y_DEPTH = 8,
          int Z_DEPTH = 8>
class SkipListMapV2 {
public:
  typedef GenericVoxel3D<V, D> Voxel3D;

  /**
       * Batch integration entry
       */
  struct IntegrationEntry {
    K x, y, z;
    V *data;
    int entry_depth;

    IntegrationEntry(K x, K y, K z, V *data, int max_depth = 3)
        : x(x), y(y), z(z), data(data), entry_depth(max_depth) {}
  };

  /**
       * Integration map for batch OMP integration
       */
  struct IntegrationMap {
    std::map<K, std::vector<IntegrationEntry>> map;
    std::vector<K> map_keys;

    IntegrationMap() {}

    void addEntry(K x, K y, K z, V *data, int max_depth = 3) {
      IntegrationEntry entry(x, y, z, data, max_depth);
      if (map.find(x) == map.end()) {
        map[x] = std::vector<IntegrationEntry>();
        map_keys.push_back(x);
      }
      map[x].push_back(entry);
    }

    void clear() {
      map.clear();
      map_keys.clear();
    }
  };

  typedef K Index;
  typedef SkipList<Index, V *, Z_DEPTH> Z_NODE;
  typedef SkipList<Index, Z_NODE *, Y_DEPTH> Y_NODE;
  typedef SkipListDense<Index, Y_NODE *, X_DEPTH> X_NODE;

  /**
       *
       * @param min_index
       * @param max_index
       * @param resolution_x
       * @param resolution_y
       * @param resolution_z
       */
  SkipListMapV2(K min_index, K max_index, D resolution_x, D resolution_y,
                D resolution_z)
      : _min_index_value(min_index), _max_index_value(max_index),
        _resolution_x(resolution_x), _resolution_y(resolution_y),
        _resolution_z(resolution_z), _voxel_counter(0), _xlist_counter(0),
        _ylist_counter(0), _bytes_counter(0), _batch_integration(false),
        _initialized(false), _self_concurrency_management(false) {
    initialize(_min_index_value, _max_index_value);
  }

  /**
       */
  SkipListMapV2(D resolution)
      : _min_index_value(std::numeric_limits<K>::min()),
        _max_index_value(std::numeric_limits<K>::max()),
        _resolution_x(resolution), _resolution_y(resolution),
        _resolution_z(resolution), _voxel_counter(0), _xlist_counter(0),
        _ylist_counter(0), _bytes_counter(0), _batch_integration(false),
        _initialized(false), _self_concurrency_management(false) {
    initialize(_min_index_value, _max_index_value);
  }

  /**
       */
  SkipListMapV2()
      : _min_index_value(std::numeric_limits<K>::min()),
        _max_index_value(std::numeric_limits<K>::max()), _resolution_x(0.01),
        _resolution_y(0.01), _resolution_z(0.1), _voxel_counter(0),
        _xlist_counter(0), _ylist_counter(0), _bytes_counter(0),
        _batch_integration(false), _initialized(false),
        _self_concurrency_management(false) {}

  /**
       *
       */
  virtual ~SkipListMapV2() {
    for (typename std::map<K, boost::mutex *>::iterator it =
             this->mutex_map.begin();
         it != this->mutex_map.end(); ++it) {
      delete it->second;
    }
  }

  /**
       */
  void initialize(K min_index, K max_index) {
    if (_initialized) {
      delete _root_list;
    }
    _root_list = new X_NODE(min_index, max_index);
    _bytes_counter += sizeof(X_NODE);
  }

  /**
       *
       * @param ix
       * @param iy
       * @param iz
       * @return
       */
  virtual bool isValidIndex(K ix, K iy, K iz) {
    bool result = true;
    result &= ix <= _max_index_value && ix >= _min_index_value;
    result &= iy <= _max_index_value && iy >= _min_index_value;
    result &= iz <= _max_index_value && iz >= _min_index_value;
    return result;
  };

  /**
       *
       * @param x
       * @param y
       * @param z
       * @param ix
       * @param iy
       * @param iz
       * @return
       */
  virtual bool coordinatesToIndex(D x, D y, D z, K &ix, K &iy, K &iz) {
    ix = K(floor(x / _resolution_x));
    iy = K(floor(y / _resolution_y));
    iz = K(floor(z / _resolution_z));
    return isValidIndex(ix, iy, iz);
  }

  /**
       *
       * @param index
       * @param coordinate
       * @param resolution
       * @return
       */
  virtual bool singleIndexToCoordinate(K index, D &coordinate, D resolution) {
    coordinate = index * resolution + resolution * 0.5;
  }

  /**
       *
       * @param ix
       * @param iy
       * @param iz
       * @param x
       * @param y
       * @param z
       * @return
       */
  virtual bool indexToCoordinates(K ix, K iy, K iz, D &x, D &y, D &z) {
    x = ix * _resolution_x + _resolution_x * 0.5;
    y = iy * _resolution_y + _resolution_y * 0.5;
    z = iz * _resolution_z + _resolution_z * 0.5;
    return true;
  }

  /**
       *
       * @param ix
       * @param iy
       * @param iz
       * @return
       */
  virtual V *find(K ix, K iy, K iz) {
    const typename X_NODE::NodeType *ylist = _root_list->find(ix);
    if (ylist != NULL && ylist->value != NULL) {
      const typename Y_NODE::NodeType *zlist = ylist->value->find(iy);
      if (zlist != NULL && zlist->value != NULL) {
        const typename Z_NODE::NodeType *voxel = zlist->value->find(iz);
        if (voxel != NULL) {
          return voxel->value;
        }
      }
    }
    return NULL;
  }

  /**
       *
       * @param x
       * @param y
       * @param z
       * @return
       */
  virtual V *find(D x, D y, D z) {
    K ix, iy, iz;
    if (coordinatesToIndex(x, y, z, ix, iy, iz)) {
      return find(ix, iy, iz);
    }
    return NULL;
  }

  /**
  * Lock concurrency access to a X branch
  * @param key x index
  * @return
  */
  virtual void lockMap(K key) {
    this->mutex_map_mutex.lock();
    if (this->mutex_map.count(key) == 0) {
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
  virtual void unlockMap(K key) {
    this->mutex_map_mutex.lock();
    if (this->mutex_map.count(key) > 0) {
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
  virtual bool integrateVoxel(D x, D y, D z, V *data) {
    K ix, iy, iz;
    if (coordinatesToIndex(x, y, z, ix, iy, iz)) {
      return integrateVoxel(ix, iy, iz, data);
    }
    return false;
  }

  /**
       *
       * @param ix
       * @param iy
       * @param iz
       * @param data
       * @return
       */
  virtual bool integrateVoxel(K ix, K iy, K iz, V *data) {
    if (isValidIndex(ix, iy, iz)) {

      if (this->hasConcurrencyAccess())
        this->_root_list->lock(ix);

      const typename X_NODE::NodeType *ylist = _root_list->find(ix);
      if (ylist == NULL) {
        ylist = _root_list->insert(
            ix, new Y_NODE(_min_index_value, _max_index_value));
        //_bytes_counter += sizeof(typename X_NODE::NodeType) + sizeof(Y_NODE);
      }

      const typename Y_NODE::NodeType *zlist = ylist->value->find(iy);
      if (zlist == NULL) {
        zlist = ylist->value->insert(
            iy, new Z_NODE(_min_index_value, _max_index_value));
        //_bytes_counter += sizeof(typename Y_NODE::NodeType) + sizeof(Z_NODE);
      }
      const typename Z_NODE::NodeType *voxel = zlist->value->find(iz);
      if (voxel == NULL) {
        voxel = zlist->value->insert(iz, new V(data));
        // _bytes_counter += sizeof(typename Y_NODE::NodeType) + sizeof(V);
      } else {
        *(voxel->value) = *(voxel->value) + *data;
      }

      if (this->hasConcurrencyAccess())
        this->_root_list->unlock(ix);
      return true;
    }
    return false;
  }

  /**
       *
       * @return
       */
  virtual bool startBatchIntegration() {
    _batch_integration = true;
    _current_integration_map.clear();
    return true;
  }

  /**
       *
       * @param voxels
       */
  virtual void fetchVoxels(std::vector<Voxel3D> &voxels) {
    voxels.clear();
    std::vector<typename X_NODE::NodeType *> xnodes;
    _root_list->retrieveNodes(xnodes);

#pragma omp parallel
    {
      std::vector<Voxel3D> voxels_private;

#pragma omp for nowait
      for (int i = 0; i < xnodes.size(); i++) {
        K ix, iy, iz;
        D x, y, z;
        std::vector<typename Y_NODE::NodeType *> ynodes;
        xnodes[i]->value->retrieveNodes(ynodes);
        for (int j = 0; j < ynodes.size(); j++) {
          std::vector<typename Z_NODE::NodeType *> znodes;
          ynodes[j]->value->retrieveNodes(znodes);

          for (int k = 0; k < znodes.size(); k++) {
            ix = xnodes[i]->key;
            iy = ynodes[j]->key;
            iz = znodes[k]->key;
            indexToCoordinates(ix, iy, iz, x, y, z);

            voxels_private.push_back(Voxel3D(x, y, z, znodes[k]->value));
          }
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
       * @param cz
       * @param radiusx
       * @param radiusy
       * @param radiusz
       * @param voxels
       * @param boxed
       */
  virtual void radiusSearch(K cx, K cy, K cz, K radiusx, K radiusy, K radiusz,
                            std::vector<Voxel3D> &voxels, bool boxed = false) {
    voxels.clear();
    std::vector<typename X_NODE::NodeType *> xnodes;

    K ix_min = cx - radiusx;
    K ix_max = cx + radiusx;
    K iy_min = cy - radiusy;
    K iy_max = cy + radiusy;
    K iz_min = cz - radiusz;
    K iz_max = cz + radiusz;

    _root_list->retrieveNodesByRange(ix_min, ix_max, xnodes);

    D rx, ry, rz, radius;
    D centerx, centery, centerz;

    indexToCoordinates(radiusx, radiusy, radiusz, rx, ry, rz);
    indexToCoordinates(cx, cy, cz, centerx, centery, centerz);
    radius = (rx + ry + rz) / 3.0;

#pragma omp parallel
    {
      std::vector<Voxel3D> voxels_private;

#pragma omp for nowait
      for (int i = 0; i < xnodes.size(); i++) {
        K ix, iy, iz;
        D x, y, z;
        D distance;
        std::vector<typename Y_NODE::NodeType *> ynodes;
        xnodes[i]->value->retrieveNodesByRange(iy_min, iy_max, ynodes);
        for (int j = 0; j < ynodes.size(); j++) {
          std::vector<typename Z_NODE::NodeType *> znodes;
          ynodes[j]->value->retrieveNodesByRange(iz_min, iz_max, znodes);
          ix = xnodes[i]->key;
          iy = ynodes[j]->key;

          for (int k = 0; k < znodes.size(); k++) {
            iz = znodes[k]->key;
            indexToCoordinates(ix, iy, iz, x, y, z);
            if (!boxed) {
              distance = sqrt(pow(centerx - x, 2) + pow(centery - y, 2) +
                              pow(centerz - z, 2));
              if (distance > radius)
                continue;
            }
            voxels_private.push_back(Voxel3D(x, y, z, znodes[k]->value));
          }
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
  virtual void radiusSearch(D cx, D cy, D cz, D radiusx, D radiusy, D radiusz,
                            std::vector<Voxel3D> &voxels, bool boxed = false) {
    K ix, iy, iz;
    if (coordinatesToIndex(cx, cy, cz, ix, iy, iz)) {
      K iradiusx, iradiusy, iradiusz;
      iradiusx = K(floor(radiusx / _resolution_x));
      iradiusy = K(floor(radiusy / _resolution_y));
      iradiusz = K(floor(radiusz / _resolution_z));
      radiusSearch(ix, iy, iz, iradiusx, iradiusy, iradiusz, voxels, boxed);
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
  virtual void saveToFile(std::string filename) {
    std::vector<Voxel3D> voxels;
    fetchVoxels(voxels);

    std::ofstream f;
    f.open(filename);
    V a1;
    K a2;
    D a3;
    f << "# SkipListMapV2<" << typeid(a1).name() << "," << typeid(a2).name()
      << "," << typeid(a3).name() << ">" << std::endl;
    f << _min_index_value << " " << _max_index_value << " ";
    f << _resolution_x << " ";
    f << _resolution_y << " ";
    f << _resolution_z << std::endl;
    for (int i = 0; i < voxels.size(); i++) {
      f << voxels[i] << std::endl;
    }
    f.close();
  }

  /**
       */
  virtual void loadFromFile(std::string filename) {
    std::ifstream input_file(filename.c_str());
    if (input_file.is_open()) {
    }
    // Load Files
    std::string line;

    bool header_found = false;

    while (std::getline(input_file, line)) {
      std::istringstream iss(line);
      if (line.length() > 0 && line.at(0) == '#')
        continue;

      if (!header_found) {
        K min, max;
        iss >> min;
        iss >> max;
        iss >> _resolution_x;
        iss >> _resolution_y;
        iss >> _resolution_z;
        initialize(min, max);
        header_found = true;
        continue;
      }

      if (header_found) {
        Voxel3D voxel;
        iss >> voxel;
        this->integrateVoxel(voxel.x, voxel.y, voxel.z, voxel.data);
      }
    }
  }

  virtual void enableConcurrencyAccess(bool status = true) {
    this->_self_concurrency_management = status;
  }

  virtual bool hasConcurrencyAccess() {
    return this->_self_concurrency_management;
  }

protected:
  /**
       *
       * @param ylist
       * @param iy
       * @param iz
       * @param data
       * @return
       */
  bool _integrateXNode(const typename X_NODE::NodeType *ylist, K &iy, K &iz,
                       V *data) {
    const typename Y_NODE::NodeType *zlist = ylist->value->find(iy);
    if (zlist == NULL) {
      zlist = ylist->value->insert(
          iy, new Z_NODE(_min_index_value, _max_index_value));
    }
    const typename Z_NODE::NodeType *voxel = zlist->value->find(iz);
    if (voxel == NULL) {
      voxel = zlist->value->insert(iz, new V(data));
    } else {
      *(voxel->value) = *(voxel->value) + *data;
    }
    return true;
  }

  Index _max_index_value;
  Index _min_index_value;
  X_NODE *_root_list;
  D _resolution_x;
  D _resolution_y;
  D _resolution_z;
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
  std::map<K, boost::mutex *> mutex_map;
};
}

#endif /* SkipListMapV2_HPP */
