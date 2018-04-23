#ifndef Octree_H
#define Octree_H

#include <boost/thread.hpp>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <cmath>
#include <skimap/voxels/GenericVoxel3D.hpp>

namespace skimap
{

struct Vec3;
Vec3 operator*(float r, const Vec3 &v);

struct Vec3
{
    union {
        struct
        {
            float x, y, z;
        };
        float D[3];
    };

    Vec3() {}
    Vec3(float _x, float _y, float _z)
        : x(_x), y(_y), z(_z)
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

    float maxComponent() const
    {
        float r = x;
        if (y > r)
            r = y;
        if (z > r)
            r = z;
        return r;
    }

    float minComponent() const
    {
        float r = x;
        if (y < r)
            r = y;
        if (z < r)
            r = z;
        return r;
    }

    Vec3 operator+(const Vec3 &r) const
    {
        return Vec3(x + r.x, y + r.y, z + r.z);
    }

    Vec3 operator-(const Vec3 &r) const
    {
        return Vec3(x - r.x, y - r.y, z - r.z);
    }

    Vec3 cmul(const Vec3 &r) const
    {
        return Vec3(x * r.x, y * r.y, z * r.z);
    }

    Vec3 cdiv(const Vec3 &r) const
    {
        return Vec3(x / r.x, y / r.y, z / r.z);
    }

    Vec3 operator*(float r) const
    {
        return Vec3(x * r, y * r, z * r);
    }

    Vec3 operator/(float r) const
    {
        return Vec3(x / r, y / r, z / r);
    }

    Vec3 &operator+=(const Vec3 &r)
    {
        x += r.x;
        y += r.y;
        z += r.z;
        return *this;
    }

    Vec3 &operator-=(const Vec3 &r)
    {
        x -= r.x;
        y -= r.y;
        z -= r.z;
        return *this;
    }

    Vec3 &operator*=(float r)
    {
        x *= r;
        y *= r;
        z *= r;
        return *this;
    }

    // Inner/dot product
    float operator*(const Vec3 &r) const
    {
        return x * r.x + y * r.y + z * r.z;
    }

    float norm() const
    {
        return sqrtf(x * x + y * y + z * z);
    }

    float normSquared() const
    {
        return x * x + y * y + z * z;
    }

    // Cross product
    Vec3 operator^(const Vec3 &r) const
    {
        return Vec3(
            y * r.z - z * r.y,
            z * r.x - x * r.z,
            x * r.y - y * r.x);
    }

    Vec3 normalized() const
    {
        return *this / norm();
    }
};

inline Vec3 operator*(float r, const Vec3 &v)
{
    return Vec3(v.x * r, v.y * r, v.z * r);
}

/**!
	 *
	 */
template <class PointData, class D, uint16_t MAX_DEPTH>
class Octree
{

    // Physical position/size. This implicitly defines the bounding
    // box of this node
    Vec3 origin;        //! The physical center of this node
    Vec3 halfDimension; //! Half the width/height/depth of this node

    // The tree has up to eight children and can additionally store
    // a point, though in many applications only, the leaves will store data.
    Octree *children[8]; //! Pointers to child octants
    PointData *data;     //! Data point to be stored at a node
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
    typedef GenericVoxel3D<PointData, D> Voxel3D;
    Octree(float voxel_size, float g)
        : origin(Vec3(0, 0, 0)), depth(0), data(NULL)
    {

        // Initially, there are no children
        for (int i = 0; i < 8; ++i)
            children[i] = NULL;

        for (int i = 0; i < MAX_DEPTH; i++)
        {
            voxel_size *= 2.;
        }
        halfDimension = Vec3(
            voxel_size / 2.,
            voxel_size / 2.,
            voxel_size / 2.);

        printf("OCTREE SIZE: %f\n", voxel_size);
    }

    Octree(const Vec3 &origin, const Vec3 &halfDimension, uint16_t depth = 0)
        : origin(origin), halfDimension(halfDimension), depth(depth), data(NULL)
    {
        // Initially, there are no children
        for (int i = 0; i < 8; ++i)
            children[i] = NULL;
    }

    Octree(const Octree &copy)
        : origin(copy.origin), halfDimension(copy.halfDimension), depth(copy.depth), data(copy.data)
    {
    }

    ~Octree()
    {
        // Recursively destroy octants
        for (int i = 0; i < 8; ++i)
            delete children[i];
    }

    // Determine which octant of the tree would contain 'point'
    int getOctantContainingPoint(D x, D y, D z) const
    {
        int oct = 0;
        if (x >= origin.x)
            oct |= 4;
        if (y >= origin.y)
            oct |= 2;
        if (z >= origin.z)
            oct |= 1;
        return oct;
    }

    bool isLeafNode() const
    {
        // This is correct, but overkill. See below.
        /*
				 for(int i=0; i<8; ++i)
				 if(children[i] != NULL) 
				 return false;
				 return true;
			 */

        // We are a leaf iff we have no children. Since we either have none, or
        // all eight, it is sufficient to just check the first.
        return depth == MAX_DEPTH;
    }

    bool isIncomplete()
    {
        return children[0] == NULL;
    }

    void integrateVoxel(D x, D y, D z, PointData *newdata)
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
                for (int i = 0; i < 8; ++i)
                {
                    // Compute new bounding box for this child
                    Vec3 newOrigin = origin;
                    newOrigin.x += halfDimension.x * (i & 4 ? .5f : -.5f);
                    newOrigin.y += halfDimension.y * (i & 2 ? .5f : -.5f);
                    newOrigin.z += halfDimension.z * (i & 1 ? .5f : -.5f);
                    children[i] = new Octree(newOrigin, halfDimension * .5f, depth + 1);
                }
            }
            children[getOctantContainingPoint(x, y, z)]->integrateVoxel(x, y, z, newdata);
        }
    }

    /**
       *
       * @param voxels
       */
    virtual void fetchVoxels(std::vector<Voxel3D> &voxels)
    {
        if (isLeafNode())
        {
            if (data != NULL)
                //printf("%f,%f,%f = %d \n", origin.x, origin.y, origin.z, data);
                voxels.push_back(Voxel3D(origin.x, origin.y, origin.z, data));
        }
        else
        {
            if (isIncomplete())
                return;
            for (int i = 0; i < 8; ++i)
            {
                children[i]->fetchVoxels(voxels);
            }
        }
    }

    virtual void enableConcurrencyAccess(bool enable) {}
    // // This is a really simple routine for querying the tree for points
    // // within a bounding box defined by min/max points (bmin, bmax)
    // // All results are pushed into 'results'
    // void
    // getPointsInsideBox(const Vec3 &bmin, const Vec3 &bmax, std::vector<PointData *> &results)
    // {
    //     // If we're at a leaf node, just see if the current data point is inside
    //     // the query bounding box
    //     if (isLeafNode())
    //     {
    //         if (data != NULL)
    //         {
    //             const Vec3 &p = data->getPosition();
    //             if (p.x > bmax.x || p.y > bmax.y || p.z > bmax.z)
    //                 return;
    //             if (p.x < bmin.x || p.y < bmin.y || p.z < bmin.z)
    //                 return;
    //             results.push_back(data);
    //         }
    //     }
    //     else
    //     {
    //         // We're at an interior node of the tree. We will check to see if
    //         // the query bounding box lies outside the octants of this node.
    //         for (int i = 0; i < 8; ++i)
    //         {
    //             // Compute the min/max corners of this child octant
    //             Vec3 cmax = children[i]->origin + children[i]->halfDimension;
    //             Vec3 cmin = children[i]->origin - children[i]->halfDimension;

    //             // If the query rectangle is outside the child's bounding box,
    //             // then continue
    //             if (cmax.x < bmin.x || cmax.y < bmin.y || cmax.z < bmin.z)
    //                 continue;
    //             if (cmin.x > bmax.x || cmin.y > bmax.y || cmin.z > bmax.z)
    //                 continue;

    //             // At this point, we've determined that this child is intersecting
    //             // the query bounding box
    //             children[i]->getPointsInsideBox(bmin, bmax, results);
    //         }
    //     }
    // }
};
}
#endif