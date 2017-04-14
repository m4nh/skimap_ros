/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SLAM_DUNK_QUADTREE_H
#define SLAM_DUNK_QUADTREE_H

#include "slamdunk/slamdunk_defines.h"

#include <list>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace slamdunk
{

template <class Tp>
struct SLAM_DUNK_API QuadTreeElement
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Tp DataType;

  Eigen::Vector2d m_coords;
  DataType m_data;
};

template <class DataTp, class ElementRefTp>
class SLAM_DUNK_API QuadTreeNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DataTp DataType;
  typedef ElementRefTp ElementRef;
  typedef QuadTreeNode<DataType, ElementRef> Node;
  typedef QuadTreeElement<DataType> Element;
  typedef std::list<Element, Eigen::aligned_allocator<Element>> ElementList;

  virtual ~QuadTreeNode(){};

  virtual ElementRef insert(Element &e) = 0;
  virtual bool isEmpty() const = 0;
  virtual void query(const Eigen::Vector2d &offset, double length,
                     const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, ElementList &data) const = 0;
  virtual unsigned char getLevel() const = 0;
};

template <class DataTp, class LeafTp>
class SLAM_DUNK_API QuadTreeBranch : public QuadTreeNode<DataTp, typename LeafTp::ElementRefTp>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DataTp DataType;
  typedef LeafTp LeafType;
  typedef typename LeafType::ElementRefTp ElementRef;
  typedef QuadTreeNode<DataType, ElementRef> Node;
  typedef QuadTreeElement<DataType> Element;
  typedef std::list<Element, Eigen::aligned_allocator<Element>> ElementList;

  QuadTreeBranch(unsigned char level) : m_level(level) { m_children[0] = m_children[1] = m_children[2] = m_children[3] = NULL; };
  virtual ~QuadTreeBranch()
  {
    delete m_children[0];
    delete m_children[1];
    delete m_children[2];
    delete m_children[3];
  }

  virtual inline ElementRef insert(Element &e)
  {
    e.m_coords *= 2.0;
    assert(e.m_coords.x() >= 0 && e.m_coords.x() < 2.0);
    assert(e.m_coords.y() >= 0 && e.m_coords.y() < 2.0);

    const int index = ((int)e.m_coords.x()) + 2 * ((int)e.m_coords.y());
    if (e.m_coords.x() >= 1.0)
      e.m_coords.x() -= 1.0;
    if (e.m_coords.y() >= 1.0)
      e.m_coords.y() -= 1.0;

    if (m_children[index] == NULL)
    {
      if (m_level == 0)
        m_children[index] = new LeafType();
      else
        m_children[index] = new QuadTreeBranch(m_level - 1);
    }
    return m_children[index]->insert(e);
  }

  virtual inline bool isEmpty() const { return !(m_children[0] || m_children[1] || m_children[2] || m_children[3]); }

  virtual inline void query(const Eigen::Vector2d &offset, double length,
                            const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, ElementList &data) const
  {
    length /= 2.0;
    if (m_children[0] && p0.x() < 0.5 && p0.y() < 0.5)
      m_children[0]->query(offset, length, p0 * 2, p1 * 2, data);
    if (m_children[1] && p0.y() < 0.5 && p1.x() > 0.5)
      m_children[1]->query(Eigen::Vector2d(offset.x() + length, offset.y()), length,
                           Eigen::Vector2d(2 * p0.x() - 1, 2 * p0.y()), Eigen::Vector2d(2 * p1.x() - 1, 2 * p1.y()), data);
    if (m_children[2] && p0.x() < 0.5 && p1.y() > 0.5)
      m_children[2]->query(Eigen::Vector2d(offset.x(), offset.y() + length), length,
                           Eigen::Vector2d(2 * p0.x(), 2 * p0.y() - 1), Eigen::Vector2d(2 * p1.x(), 2 * p1.y() - 1), data);
    if (m_children[3] && p1.x() > 0.5 && p1.y() > 0.5)
      m_children[3]->query(Eigen::Vector2d(offset.x() + length, offset.y() + length), length,
                           Eigen::Vector2d(2 * p0.x() - 1, 2 * p0.y() - 1), Eigen::Vector2d(2 * p1.x() - 1, 2 * p1.y() - 1), data);
  }

  virtual inline unsigned char getLevel() const { return m_level; }

private:
  unsigned char m_level;
  Node *m_children[4]; // { (0,0), (1,0), (0,1), (1,1) }
};

namespace internal
{
template <class Tp>
struct SLAM_DUNK_API type_wrapper
{
  typedef Tp type;
};
}

template <class LeafTp>
class SLAM_DUNK_API QuadTreeLeafElementRef
{
public:
  typedef LeafTp Leaf;
  typedef typename Leaf::Element Element;
  typedef typename std::list<Element, Eigen::aligned_allocator<Element>>::iterator ElementIterator;

  bool erase()
  {
    if (!nodePtr || elemIt == nodePtr->m_elements.end())
      return false;
    nodePtr->m_elements.erase(elemIt);
    elemIt = nodePtr->m_elements.end();
    return true;
  }

private:
  friend typename internal::type_wrapper<Leaf>::type;

  ElementIterator elemIt;
  Leaf *nodePtr;
};

template <class DataTp>
class SLAM_DUNK_API QuadTreeLeaf : public QuadTreeNode<DataTp, QuadTreeLeafElementRef<QuadTreeLeaf<DataTp>>>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DataTp DataType;
  typedef QuadTreeLeafElementRef<QuadTreeLeaf<DataTp>> ElementRefTp;
  typedef ElementRefTp ElementRef;
  typedef QuadTreeNode<DataType, ElementRef> Node;
  typedef QuadTreeElement<DataType> Element;
  typedef std::list<Element, Eigen::aligned_allocator<Element>> ElementList;

  virtual inline ElementRef insert(Element &e)
  {
    m_elements.push_back(e);
    ElementRef er;
    er.nodePtr = this;
    er.elemIt = --(m_elements.end());
    return er;
  }

  virtual inline bool isEmpty() const { return m_elements.empty(); }

  virtual inline void query(const Eigen::Vector2d &offset, double length,
                            const Eigen::Vector2d &p0, const Eigen::Vector2d &p1, ElementList &data) const
  {
    for (typename ElementList::const_iterator it = m_elements.begin(); it != m_elements.end(); ++it)
      if (it->m_coords.x() >= p0.x() && it->m_coords.x() < p1.x() &&
          it->m_coords.y() >= p0.y() && it->m_coords.y() < p1.y())
      {
        data.push_back(*it);
        data.back().m_coords = data.back().m_coords * length + offset;
      }
  }

  virtual inline unsigned char getLevel() const { return 0; }

private:
  friend class QuadTreeLeafElementRef<QuadTreeLeaf<DataTp>>;
  ElementList m_elements;
};

template <class DataTp>
struct SLAM_DUNK_API DummyDataID
{
  typedef DataTp DataType;
  int operator()(const DataType &data) const { return (int)data; }
};

template <class DataTp, class DataIDTp = DummyDataID<DataTp>, class BranchTp = QuadTreeBranch<DataTp, QuadTreeLeaf<DataTp>>>
class SLAM_DUNK_API QuadTree
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DataTp DataType;
  typedef DataIDTp DataIDType;
  typedef BranchTp BranchType;
  typedef typename BranchType::ElementRef ElementRef;
  typedef QuadTreeNode<DataType, ElementRef> Node;
  typedef QuadTreeElement<DataType> Element;
  typedef std::list<Element, Eigen::aligned_allocator<Element>> ElementList;
  typedef std::map<int, ElementRef> ElementMap;

  QuadTree(double resolution, unsigned char max_depth, const DataIDType &dataId = DataIDType())
      : m_resolution(resolution), m_length(resolution * (1u << (max_depth + 1))),
        m_root(new BranchType(max_depth + 1)), m_dataid(dataId) {}
  virtual ~QuadTree() { delete m_root; }

  inline bool insert(const DataType &data, const Eigen::Vector2d &coords)
  {
    if (m_root->isEmpty())
      m_offset.noalias() = coords - Eigen::Vector2d(m_length / 2., m_length / 2.);
    else if (coords.x() < m_offset.x() || coords.x() >= m_offset.x() + m_length ||
             coords.y() < m_offset.y() || coords.y() >= m_offset.y() + m_length)
      return false;

    Element e;
    e.m_coords.noalias() = (coords - m_offset) / m_length;
    e.m_data = data;
    m_element_map[m_dataid(data)] = m_root->insert(e);
    return true;
  }

  inline bool remove(const DataType &data)
  {
    int id = m_dataid(data);
    typename ElementMap::iterator it = m_element_map.find(id);
    if (it == m_element_map.end())
      return false;
    bool erase_res = it->second.erase();
    m_element_map.erase(it);
    return erase_res;
  }

  // remove and insert again -- more efficient than {{{ this->remove(data); this->insert(data, coords); }}}
  inline bool update(const DataType &data, const Eigen::Vector2d &coords)
  {
    int id = m_dataid(data);
    typename ElementMap::iterator it = m_element_map.find(id);
    if (it != m_element_map.end())
      it->second.erase();

    if (m_root->isEmpty())
      m_offset.noalias() = coords - Eigen::Vector2d(m_length / 2., m_length / 2.);
    else if (coords.x() < m_offset.x() || coords.x() >= m_offset.x() + m_length ||
             coords.y() < m_offset.y() || coords.y() >= m_offset.y() + m_length)
    {
      if (it != m_element_map.end())
        m_element_map.erase(it);
      return false;
    }

    Element e;
    e.m_coords.noalias() = (coords - m_offset) / m_length;
    e.m_data = data;
    if (it != m_element_map.end())
      it->second = m_root->insert(e);
    else
      m_element_map[id] = m_root->insert(e);
    return true;
  }

  inline void query(double x0, double y0, double width, double height, ElementList &data) const
  {
    if (width < 0)
    {
      x0 += width;
      width *= -1;
    }
    if (height < 0)
    {
      y0 += height;
      height *= -1;
    }
    if (x0 + width < m_offset.x() || x0 >= m_offset.x() + m_length ||
        y0 + height < m_offset.y() || y0 >= m_offset.y() + m_length)
      return;

    Eigen::Vector2d p0 = (Eigen::Vector2d(x0, y0) - m_offset) / m_length;
    Eigen::Vector2d p1 = (Eigen::Vector2d(x0 + width, y0 + height) - m_offset) / m_length;
    m_root->query(m_offset, m_length, p0, p1, data);
  }

  inline void query(ElementList &data) const
  {
    query(m_offset.x(), m_offset.y(), m_offset.x() + m_length, m_offset.y() + m_length, data);
  }

  inline double getResolution() const { return m_resolution; }
  inline double getLength() const { return m_length; }
  inline const Eigen::Vector2d &getOffset() const { return m_offset; }

  inline void clear()
  {
    m_element_map.clear();
    if (!(m_root->isEmpty()))
    {
      unsigned char d = m_root->getLevel();
      delete m_root;
      m_root = new BranchType(d);
    }
  }

private:
  double m_resolution, m_length;
  Node *m_root;
  Eigen::Vector2d m_offset;
  DataIDType m_dataid;
  ElementMap m_element_map;
};

/////// DRAWING FUNCTION
template <class QuadTreeT>
cv::Mat drawQuadTree(const QuadTreeT &tree, unsigned imgsize)
{
  cv::Mat_<cv::Vec3b> img(imgsize, imgsize, cv::Vec3b(255, 255, 255));
  typename QuadTreeT::ElementList data;
  tree.query(data);
  for (typename QuadTreeT::ElementList::const_iterator it = data.begin(); it != data.end(); ++it)
    cv::circle(img, cv::Point((it->m_coords.x() - tree.getOffset().x()) * imgsize / tree.getLength(), (it->m_coords.y() - tree.getOffset().y()) * imgsize / tree.getLength()),
               tree.getResolution() * imgsize / tree.getLength(), cv::Scalar(255, 0, 0), -1);

  return img;
}
}

#endif // SLAM_DUNK_QUADTREE_H
