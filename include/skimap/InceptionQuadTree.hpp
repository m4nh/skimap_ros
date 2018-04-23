#ifndef InceptionQuadtree_H
#define InceptionQuadtree_H

#include <boost/thread.hpp>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <cmath>
#include <skimap/voxels/GenericVoxel3D.hpp>

namespace skimap
{

struct Vec2;

struct Vec2
{
    union {
        struct
        {
            float x, y;
        };
        float D[2];
    };

    Vec2() {}
    Vec2(float _x, float _y)
        : x(_x), y(_y)
    {
    }

    float &operator[](unsigned int i)
    {
        return D[i];
    }

    const float &operator[](unsigned int i) const
    {
        return D[i];
    }
    Vec2 operator+(const Vec2 &r) const
    {
        return Vec2(x + r.x, y + r.y);
    }

    Vec2 operator-(const Vec2 &r) const
    {
        return Vec2(x - r.x, y - r.y);
    }

    Vec2 cmul(const Vec2 &r) const
    {
        return Vec2(x * r.x, y * r.y);
    }

    Vec2 cdiv(const Vec2 &r) const
    {
        return Vec2(x / r.x, y / r.y);
    }

    Vec2 operator*(float r) const
    {
        return Vec2(x * r, y * r);
    }

    Vec2 operator/(float r) const
    {
        return Vec2(x / r, y / r);
    }

    Vec2 &operator+=(const Vec2 &r)
    {
        x += r.x;
        y += r.y;
        return *this;
    }

    Vec2 &operator-=(const Vec2 &r)
    {
        x -= r.x;
        y -= r.y;
        return *this;
    }

    Vec2 &operator*=(float r)
    {
        x *= r;
        y *= r;
        return *this;
    }
};

/**!
	 *
	 */
template <class Voxel3D, class PointData, class D, uint16_t MAX_DEPTH>
class QuadTree
{

    // Physical position/size. This implicitly defines the bounding
    // box of this node
    Vec2 origin;        //! The physical center of this node
    Vec2 halfDimension; //! Half the width/height/depth of this node

    // The tree has up to eight children and can additionally store
    // a point, though in many applications only, the leaves will store data.
    QuadTree *children[4]; //! Pointers to child octants
    PointData *data;       //! Data point to be stored at a node
    uint16_t depth;
    /*
				Children follow a predictable pattern to make accesses simple.
				Here, - means less than 'origin' in that dimension, + means greater than.
				child:	0 1 2 3 4 5 6 7
				x:      - - - - + + + +
				y:      - - + + - - + +
				z:      - + - + - + - +
		 */

  public:
    QuadTree(D voxel_size, D g)
        : origin(Vec2(0, 0)), depth(0), data(NULL)
    {

        // Initially, there are no children
        for (int i = 0; i < 4; ++i)
            children[i] = NULL;

        for (int i = 0; i < MAX_DEPTH; i++)
        {
            voxel_size *= 2.;
        }
        halfDimension = Vec2(
            voxel_size / 2.,
            voxel_size / 2.);
    }

    QuadTree(const Vec2 &origin, const Vec2 &halfDimension, uint16_t depth)
        : origin(origin), halfDimension(halfDimension), depth(depth), data(NULL)
    {
        // Initially, there are no children
        for (int i = 0; i < 4; ++i)
            children[i] = NULL;
    }

    QuadTree(const QuadTree &copy)
        : origin(copy.origin), halfDimension(copy.halfDimension), depth(copy.depth), data(copy.data)
    {
    }

    ~QuadTree()
    {
        // Recursively destroy octants
        for (int i = 0; i < 4; ++i)
            delete children[i];
    }

    // Determine which octant of the tree would contain 'point'
    int getOctantContainingPoint(D x, D y) const
    {
        int oct = 0;
        if (x >= origin.x)
            oct |= 1;
        if (y >= origin.y)
            oct |= 2;
        return oct;
    }

    bool isLeafNode() const
    {
        return depth == MAX_DEPTH;
    }

    bool isIncomplete()
    {
        return children[0] == NULL;
    }

    void integrateVoxel(D x, D y, PointData *newdata)
    {
        // If this node doesn't have a data point yet assigned
        // and it is a leaf, then we're done!
        if (isLeafNode())
        {
            if (data == NULL)
            {
                data = new PointData();
            }

            *(data) = *(data) + *(newdata);
        }
        else
        {
            if (isIncomplete())
            {
                for (int i = 0; i < 4; ++i)
                {
                    // Compute new bounding box for this child
                    Vec2 newOrigin = origin;
                    newOrigin.x += halfDimension.x * (i & 1 ? .5f : -.5f);
                    newOrigin.y += halfDimension.y * (i & 2 ? .5f : -.5f);
                    children[i] = new QuadTree(newOrigin, halfDimension * .5f, depth + 1);
                }
            }
            children[getOctantContainingPoint(x, y)]->integrateVoxel(x, y, newdata);
        }
    }

    /**
       *
       * @param voxels
       */
    virtual void fetchVoxels(std::vector<Voxel3D> &voxels, D z)
    {
        if (isLeafNode())
        {
            if (data != NULL)
            {
                voxels.push_back(Voxel3D(origin.x, origin.y, z, data));
            }
        }
        else
        {
            if (isIncomplete())
                return;
            for (int i = 0; i < 4; ++i)
            {
                children[i]->fetchVoxels(voxels, z);
            }
        }
    }

    virtual void enableConcurrencyAccess(bool enable) {}
};

/**!
	 *
	 */
template <class PointData, class K, class D, uint16_t MAX_DEPTH, uint16_t MAX_LEVEL = 8>
class InceptionQuadTree
{
  public:
    class spinlock
    {
      private:
        typedef enum { Locked,
                       Unlocked } LockState;
        boost::atomic<LockState> state_;

      public:
        spinlock() : state_(Unlocked) {}

        void lock()
        {
            while (state_.exchange(Locked, boost::memory_order_acquire) == Locked)
            {
                /* busy-wait */
            }
        }
        void unlock()
        {
            state_.store(Unlocked, boost::memory_order_release);
        }
    };

    typedef GenericVoxel3D<PointData, D> Voxel3D;
    typedef K Index;
    typedef QuadTree<Voxel3D, PointData, D, MAX_DEPTH> QUAD_NODE;

    InceptionQuadTree(D res, D bogh)
    {
        _resolution = res;
        _min_index_value = std::numeric_limits<Index>::min();
        _max_index_value = std::numeric_limits<Index>::max();
        _concurrent_access = false;
        key_sizes = long(_max_index_value) - long(_min_index_value);
        _dense_list = new QUAD_NODE *[key_sizes];
        _mutex_array = new spinlock[this->key_sizes];
    }

    virtual bool indexToCoordinates(K iz, D &z)
    {
        z = (iz + _min_index_value) * _resolution + _resolution * 0.5;
        return true;
    }

    virtual bool coordinatesToIndex(D z, K &iz)
    {
        iz = K(floor(z / _resolution));
        iz = iz - _min_index_value;
        return isValidIndex(iz);
    }

    virtual bool isValidIndex(K iz)
    {
        bool result = true;
        result &= iz >= 0 && iz <= key_sizes;

        return result;
    };

    void integrateVoxel(D x, D y, D z, PointData *newdata)
    {
        K iz;
        if (!coordinatesToIndex(z, iz))
        {
            return;
        }

        if (this->hasConcurrencyAccess())
            this->_mutex_array[iz].lock();

        QUAD_NODE *&quadtree = _dense_list[iz];

        // XY_NODE::NodeType *quadtree = _root_list->find(iz);

        if (quadtree == NULL)
        {
            quadtree = new QUAD_NODE(_resolution, 0);
            //_bytes_counter += sizeof(typename X_NODE::NodeType) + sizeof(Y_NODE);
        }

        quadtree->integrateVoxel(x, y, newdata);

        if (this->hasConcurrencyAccess())
            this->_mutex_array[iz].unlock();
    }

    virtual void fetchVoxels(std::vector<Voxel3D> &voxels)
    {

#pragma omp parallel
        {
            std::vector<Voxel3D> voxels_private;

#pragma omp for nowait
            for (int i = 0; i < key_sizes; i++)
            {
                QUAD_NODE *quadtree = _dense_list[i];
                if (quadtree != NULL)
                {
                    D z;
                    indexToCoordinates(i, z);
                    quadtree->fetchVoxels(voxels_private, z);
                }
            }

#pragma omp critical
            voxels.insert(voxels.end(), voxels_private.begin(), voxels_private.end());
        }
    }

    virtual void enableConcurrencyAccess(bool enable) { _concurrent_access = enable; }
    virtual bool hasConcurrencyAccess()
    {
        return _concurrent_access;
    }

  private:
    Index _min_index_value;
    Index _max_index_value;
    bool _concurrent_access;
    long key_sizes;
    D _resolution;
    QUAD_NODE **_dense_list;
    spinlock *_mutex_array;
};
}
#endif