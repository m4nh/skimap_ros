/*
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights
 * Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SKIMAP_HPP
#define SKIMAP_HPP

#include <fstream>
#include <iostream>
#include <map>
#include <skimap/SkipListMapV2.hpp>
#include <skimap/voxels/GenericTile2D.hpp>
#include <vector>

#define SKIPLISTMAP_MAX_DEPTH 16

namespace skimap
{

    /**
         *
         * @param min_index
         * @param max_index
         */
    template <class V, class K, class D, int X_DEPTH = 8, int Y_DEPTH = 8,
              int Z_DEPTH = 8>
    class SkiMap : public SkipListMapV2<V, K, D, X_DEPTH, Y_DEPTH, Z_DEPTH>
    {
        public:
            typedef GenericTile2D<V, D> Tiles2D;
            typedef SkipListMapV2<V, K, D, X_DEPTH, Y_DEPTH, Z_DEPTH> ParentMap;
            // typedef SkipListMapV2<V, K, D, X_DEPTH, Y_DEPTH, Z_DEPTH>::Voxel3D Voxel3D;
            typedef typename ParentMap::X_NODE X_NODE;
            typedef typename ParentMap::Y_NODE Y_NODE;
            typedef typename ParentMap::Z_NODE Z_NODE;

            /**
                 *
                 * @param min_index
                 * @param max_index
                 * @param resolution_x
                 * @param resolution_y
                 * @param resolution_z
                 */
            SkiMap(K min_index, K max_index, D resolution_x, D resolution_y,
                   D resolution_z, D zero_level = D(0.0))
                : ParentMap(min_index, max_index, resolution_x, resolution_y,
                            resolution_z)
            {
                setZeroLevel(zero_level);
            }

            /**
                 */
            SkiMap(D resolution, D zero_level = D(0.0)) : ParentMap(resolution)
            {
                setZeroLevel(D(zero_level));
            }

            /**
                 */
            SkiMap() : ParentMap()
            {
                setZeroLevel(D(0.0));
            }

            /**
                 *
                 */
            virtual ~SkiMap() {}

            /**
                 *
                 * @param x
                 * @param y
                 * @param z
                 * @param data
                 * @return
                 */
            virtual bool integrateTile(D x, D y)
            {
                K ix, iy, iz;

                if (this->coordinatesToIndex(x, y, _zero_level, ix, iy, iz))
                {
                    return integrateTile(ix, iy, iz);
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
            virtual bool integrateTile(K ix, K iy, K iz)
            {
                if (this->isValidIndex(ix, iy, iz))
                {

                    const typename X_NODE::NodeType* ylist = this->_root_list->find(ix);

                    if (ylist == NULL)
                    {
                        ylist = this->_root_list->insert(
                                    ix, new Y_NODE(this->_min_index_value, this->_max_index_value));
                        this->_bytes_counter +=
                            sizeof(typename X_NODE::NodeType) + sizeof(Y_NODE);
                    }

                    const typename Y_NODE::NodeType* zlist = ylist->value->find(iy);

                    if (zlist == NULL)
                    {
                        zlist = ylist->value->insert(
                                    iy, new Z_NODE(this->_min_index_value, this->_max_index_value));
                        this->_bytes_counter +=
                            sizeof(typename Y_NODE::NodeType) + sizeof(Z_NODE);
                    }

                    return true;
                }

                return false;
            }

            /**
                 *
                 * @param voxels
                 */
            virtual void fetchTiles(std::vector<Tiles2D>& voxels, D min_voxel_height)
            {
                voxels.clear();
                std::vector<typename X_NODE::NodeType*> xnodes;
                this->_root_list->retrieveNodes(xnodes);

                #pragma omp parallel
                {
                    std::vector<Tiles2D> voxels_private;

                    #pragma omp for nowait

                    for (int i = 0; i < xnodes.size(); i++)
                    {
                        K ix, iy, iz;
                        D x, y, z;
                        std::vector<typename Y_NODE::NodeType*> ynodes;
                        xnodes[i]->value->retrieveNodes(ynodes);

                        for (int j = 0; j < ynodes.size(); j++)
                        {
                            std::vector<typename Z_NODE::NodeType*> znodes;

                            ix = xnodes[i]->key;
                            iy = ynodes[j]->key;
                            iz = _zero_level_key;
                            this->indexToCoordinates(ix, iy, iz, x, y, z);

                            if (ynodes[j]->value->empty())
                            {
                                voxels_private.push_back(Tiles2D(x, y, z, NULL));
                            }
                            else
                            {
                                typename Z_NODE::NodeType* first_voxel =
                                    ynodes[j]->value->findNearest(_zero_level_key);

                                if (first_voxel == NULL)
                                {
                                    voxels_private.push_back(Tiles2D(x, y, z, NULL));
                                }
                                else
                                {
                                    D vh;
                                    this->singleIndexToCoordinate(first_voxel->key, vh,
                                                                  this->_resolution_z);

                                    if (vh > min_voxel_height)
                                    {
                                        voxels_private.push_back(Tiles2D(x, y, z, NULL));
                                    }
                                    else
                                    {
                                        voxels_private.push_back(Tiles2D(x, y, z, first_voxel->value));
                                    }
                                }
                            }
                        }
                    }

                    #pragma omp critical
                    voxels.insert(voxels.end(), voxels_private.begin(), voxels_private.end());
                }
            }

            /**
                 *
                 * @param zero_level
                 */
            void setZeroLevel(D zero_level)
            {
                _zero_level = zero_level;
                _zero_level_key = K(floor(_zero_level / this->_resolution_z));
            }

        protected:
            D _zero_level;
            K _zero_level_key;
    };
} // namespace skimap

#endif /* SKIMAP_HPP */
