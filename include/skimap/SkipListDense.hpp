/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SKIPLISTDENSE_HPP
#define SKIPLISTDENSE_HPP

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

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

typedef spinlock Lock;

namespace skimap
{

/**
 * SkipListDenseNode represents a single node in a SkipList. 
 * K template represents datatype for Keys. 
 * V template represents datatype for Values.
 * MAXLEVEL template represent max depth of the SkipListDenseNode.
 */
template <class K, class V, int MAXLEVEL>
class SkipListDenseNode
{
  public:
    /**
     * Void Constructor.
     */
    SkipListDenseNode()
    {
    }

    /**
     * Constructs a node with target Key.
     * @param searchKey target Key.
     */
    SkipListDenseNode(K searchKey) : key(searchKey)
    {
    }

    /**
     * Constructs a node with target Key,Value.
     * @param searchKey target Key
     * @param val target Value
     */
    SkipListDenseNode(K searchKey, V val) : key(searchKey), value(val)
    {
    }

    /**
     * Void Destructor.
     */
    virtual ~SkipListDenseNode()
    {
    }

    K key;
    V value;
};

/**
 * SkipListDense class. 
 * K template represents datatype for Keys. 
 * V template represents datatype for Values.
 * MAXLEVEL template represent max depth of the SkipListDense.
 */
template <class K, class V, int MAXLEVEL = 16>
class SkipListDense
{
  public:
    typedef K KeyType;
    typedef V ValueType;
    typedef SkipListDenseNode<K, V, MAXLEVEL> NodeType;

    /**
     * Constructor with MIN/MAX values for keys.
     * @param min_key min Key value.
     * @param max_key max Key value.
     */
    SkipListDense(K min_key, K max_key, bool prepare_locks = true) : header_node_(NULL), tail_node_(NULL),
                                                                     max_current_level_(1), max_level(MAXLEVEL),
                                                                     min_key_(min_key), max_value_(max_key), size_(0), last_(0)
    {
        this->key_sizes = long(max_key) - long(min_key);
        //printf("Create Dense List: %d,%d =  %ld\n", min_key, max_key, this->key_sizes);
        this->_dense_nodes = new NodeType *[this->key_sizes];
        if (prepare_locks)
        {
            this->_mutex_array = new Lock[this->key_sizes];
        }
        this->empty();
        // header_node_ = new NodeType(min_key_);
        // tail_node_ = new NodeType(max_value_);
        // for (int i = 1; i <= MAXLEVEL; i++)
        // {
        //     header_node_->forwards[i] = tail_node_;
        // }
    }

    /**
     * Destructor.
     */
    virtual ~SkipListDense()
    {
        for (int i = 0; i < this->key_sizes; i++)
        {
            delete this->_dense_nodes[i];
        }

        // NodeType *curr_node = header_node_->forwards[1];
        // while (curr_node != tail_node_)
        // {
        //     NodeType *tempNode = curr_node;
        //     curr_node = curr_node->forwards[1];
        //     delete tempNode;
        // }
        // delete header_node_;
        // delete tail_node_;
    }

    long _convertKey(K key)
    {
        return key - this->min_key_;
    }

    bool checkInnerKey(long key)
    {
        return key >= 0 && key < this->key_sizes;
    }

    void lock(K key)
    {
        long inner_key = _convertKey(key);
        if (checkInnerKey(inner_key))
        {
            this->_mutex_array[inner_key].lock();
        }
    }

    void unlock(K key)
    {
        long inner_key = _convertKey(key);
        if (checkInnerKey(inner_key))
        {
            this->_mutex_array[inner_key].unlock();
        }
    }
    /**
     * Inserts new KEY,VALUE in the SkipListDense.
     * @param search_key searching Key for insertion.
     * @param new_value insertion Value.
     * @return new Node inserted, or previous one.
     */
    NodeType *insert(K search_key, V new_value)
    {
        long inner_key = _convertKey(search_key);
        if (!checkInnerKey(inner_key))
            return NULL;

        if (_dense_nodes[inner_key] == NULL)
        {
            _dense_nodes[inner_key] = new NodeType(search_key, new_value);
        }
        else
        {
            _dense_nodes[inner_key]->value = new_value;
        }
        return _dense_nodes[inner_key];
    }

    /**
     * Removes node with target Key.
     * @param search_key target Key
     */
    void erase(K search_key)
    {
        long inner_key = _convertKey(search_key);
        if (!checkInnerKey(inner_key))
            return;
        if (_dense_nodes[inner_key] != NULL)
        {
            delete _dense_nodes[inner_key];
        }
    }

    /**
     * Search by Key.
     * @param search_key target Key
     * @return 
     */
    const NodeType *find(K search_key)
    {
        long inner_key = _convertKey(search_key);
        if (!checkInnerKey(inner_key))
        {
            return NULL;
        }
        return _dense_nodes[inner_key];
    }

    /**
     * Search for node with nearest Key.
     * @param search_key target Key
     * @param previous TRUE if previous node (with respect to SkipListDense oreder) 
     * is required, FALSE otherwise.
     * @return 
     */
    NodeType *findNearest(K search_key, bool previous = false)
    {
        long inner_key = _convertKey(search_key);
        int step = previous ? -1 : 1;
        inner_key = inner_key + step;
        if (!checkInnerKey(inner_key))
            return NULL;
        NodeType *node = _dense_nodes[inner_key];
        while (node == NULL && checkInnerKey(inner_key))
        {
            inner_key = inner_key + step;
            if (!checkInnerKey(inner_key))
                return NULL;
            node = _dense_nodes[inner_key];
        }
        return node;
    }

    /**
     * @return TRUE if list is empty.
     */
    bool empty() const
    {
        for (int i = 0; i < this->key_sizes; i++)
        {
            this->_dense_nodes[i] = NULL;
        }
    }

    /**
     * @return String representation of SkipListDense
     */
    std::string toString()
    {
        /*std::stringstream sstr;
        NodeType *curr_node = header_node_->forwards[1];
        while (curr_node != tail_node_)
        {
            sstr << "(" << curr_node->key << "," << curr_node->value << ")" << std::endl;
            curr_node = curr_node->forwards[1];
        }
        return sstr.str();*/
        return "";
    }

    /**
     * Iterates list and return an ordered Vector of Nodes
     * @param nodes OUTPUT vector of Nodes
     */
    void retrieveNodes(std::vector<NodeType *> &nodes)
    {

        nodes.clear();
        for (int i = 0; i < this->key_sizes; i++)
        {
            if (this->_dense_nodes[i] != NULL)
            {
                nodes.push_back(this->_dense_nodes[i]);
            }
        }
    }

    /**
     * Iterates list and return an ordered Vector of Nodes. Search is bounded.
     * @param nodes OUTPUT vector of Nodes
     * @param start start node
     * @param end_key end target Key
     */
    void retrieveNodes(std::vector<NodeType *> &nodes, NodeType *start, K end_key)
    {
        printf("NOt implemented!!\n");
    }

    /**
     * Iterates list between two Keys and return an ordered Vector of Nodes
     * @param min_key min Key
     * @param max_key max Key
     * @param nodes OUTPUT vector of Nodes
     */
    void retrieveNodesByRange(K min_key, K max_key, std::vector<NodeType *> &nodes)
    {
        nodes.clear();
        long start = _convertKey(min_key);
        long end = _convertKey(max_key);

        for (int i = start; i <= end; i++)
        {
            if (checkInnerKey(i))
            {
                if (this->_dense_nodes[i] != NULL)
                {
                    nodes.push_back(this->_dense_nodes[i]);
                }
            }
        }
    }

    /**
     * @return  First Node of the list.
     */
    const NodeType *first()
    {
        for (int i = 0; i < this->key_sizes; i++)
        {
            if (this->_dense_nodes[i] != NULL)
            {
                return this->_dense_nodes[i];
            }
        }
        return NULL;
    }

    /**
     * Returns last node. Requires a search O(nlog(n))
     * @return Last Node of the list.
     */
    const NodeType *last()
    {
        for (int i = this->key_sizes - 1; i >= 0; i--)
        {
            if (this->_dense_nodes[i] != NULL)
            {
                return this->_dense_nodes[i];
            }
        }
        return NULL;
    }

    /**
     * @return Last Node value with low reliability. Requires O(1).
     */
    K getProbableLastKey()
    {
        return last_;
    }

    /**
     * @return List size.
     */
    int getSize()
    {
        return size_;
    }

    const int max_level;

  protected:
    /**
     * 
     * @return uniform random value
     */
    double uniformRandom()
    {
        return rand() / double(RAND_MAX);
    }

    /**
     * @return random SkipListDense level
     */
    int randomLevel()
    {
        int level = 1;
        double p = 0.5;
        while (uniformRandom() < p && level < MAXLEVEL)
        {
            level++;
        }
        return level;
    }

    K min_key_;
    K max_value_;
    long key_sizes;
    K last_;
    int max_current_level_;
    int size_;
    SkipListDenseNode<K, V, MAXLEVEL> *header_node_;
    SkipListDenseNode<K, V, MAXLEVEL> *tail_node_;
    NodeType **_dense_nodes;
    Lock *_mutex_array;
};
}

#endif /* SKIPLISTDENSE_HPP */
