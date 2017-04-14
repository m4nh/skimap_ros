/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SKIPLIST_HPP
#define SKIPLIST_HPP

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <vector>

namespace skimap
{

/**
 * SkipListNode represents a single node in a SkipList. 
 * K template represents datatype for Keys. 
 * V template represents datatype for Values.
 * MAXLEVEL template represent max depth of the SkipListNode.
 */
template <class K, class V, int MAXLEVEL>
class SkipListNode
{
  public:
    /**
     * Void Constructor.
     */
    SkipListNode()
    {
        for (int i = 1; i <= MAXLEVEL; i++)
        {
            forwards[i] = NULL;
        }
    }

    /**
     * Constructs a node with target Key.
     * @param searchKey target Key.
     */
    SkipListNode(K searchKey) : key(searchKey)
    {
        for (int i = 1; i <= MAXLEVEL; i++)
        {
            forwards[i] = NULL;
        }
    }

    /**
     * Constructs a node with target Key,Value.
     * @param searchKey target Key
     * @param val target Value
     */
    SkipListNode(K searchKey, V val) : key(searchKey), value(val)
    {
        for (int i = 1; i <= MAXLEVEL; i++)
        {
            forwards[i] = NULL;
        }
    }

    /**
     * Void Destructor.
     */
    virtual ~SkipListNode()
    {
    }

    K key;
    V value;
    SkipListNode<K, V, MAXLEVEL> *forwards[MAXLEVEL + 1];
};

/**
 * SkipList class. 
 * K template represents datatype for Keys. 
 * V template represents datatype for Values.
 * MAXLEVEL template represent max depth of the SkipList.
 */
template <class K, class V, int MAXLEVEL = 16>
class SkipList
{
  public:
    typedef K KeyType;
    typedef V ValueType;
    typedef SkipListNode<K, V, MAXLEVEL> NodeType;

    /**
     * Constructor with MIN/MAX values for keys.
     * @param min_key min Key value.
     * @param max_key max Key value.
     */
    SkipList(K min_key, K max_key) : header_node_(NULL), tail_node_(NULL),
                                     max_current_level_(1), max_level(MAXLEVEL),
                                     min_key_(min_key), max_value_(max_key), size_(0), last_(0)
    {
        header_node_ = new NodeType(min_key_);
        tail_node_ = new NodeType(max_value_);
        for (int i = 1; i <= MAXLEVEL; i++)
        {
            header_node_->forwards[i] = tail_node_;
        }
    }

    /**
     * Destructor.
     */
    virtual ~SkipList()
    {
        NodeType *curr_node = header_node_->forwards[1];
        while (curr_node != tail_node_)
        {
            NodeType *tempNode = curr_node;
            curr_node = curr_node->forwards[1];
            delete tempNode;
        }
        delete header_node_;
        delete tail_node_;
    }

    /**
     * Inserts new KEY,VALUE in the SkipList.
     * @param search_key searching Key for insertion.
     * @param new_value insertion Value.
     * @return new Node inserted, or previous one.
     */
    NodeType *insert(K search_key, V new_value)
    {
        SkipListNode<K, V, MAXLEVEL> *update[MAXLEVEL];
        NodeType *curr_node = header_node_;
        for (int level = max_current_level_; level >= 1; level--)
        {
            while (curr_node->forwards[level]->key < search_key)
            {
                curr_node = curr_node->forwards[level];
            }
            update[level] = curr_node;
        }
        curr_node = curr_node->forwards[1];
        if (curr_node->key == search_key)
        {
            curr_node->value = new_value;
        }
        else
        {
            int new_level = randomLevel();
            if (new_level > max_current_level_)
            {
                for (int level = max_current_level_ + 1; level <= new_level; level++)
                {
                    update[level] = header_node_;
                }
                max_current_level_ = new_level;
            }
            curr_node = new NodeType(search_key, new_value);
            size_++;
            for (int lv = 1; lv <= new_level; lv++)
            {
                curr_node->forwards[lv] = update[lv]->forwards[lv];
                update[lv]->forwards[lv] = curr_node;
            }
            if (getSize() <= 1)
            {
                last_ = search_key;
            }
            else
            {
                if (search_key > last_)
                {
                    last_ = search_key;
                }
            }
        }
        return curr_node;
    }

    /**
     * Removes node with target Key.
     * @param search_key target Key
     */
    void erase(K search_key)
    {
        SkipListNode<K, V, MAXLEVEL> *update[MAXLEVEL];
        NodeType *curr_node = header_node_;
        for (int level = max_current_level_; level >= 1; level--)
        {
            while (curr_node->forwards[level]->key < search_key)
            {
                curr_node = curr_node->forwards[level];
            }
            update[level] = curr_node;
        }
        curr_node = curr_node->forwards[1];
        if (curr_node->key == search_key)
        {
            for (int lv = 1; lv <= max_current_level_; lv++)
            {
                if (update[lv]->forwards[lv] != curr_node)
                {
                    break;
                }
                update[lv]->forwards[lv] = curr_node->forwards[lv];
            }
            delete curr_node;
            size_--;
            // update the max level
            while (max_current_level_ > 1 && header_node_->forwards[max_current_level_] == NULL)
            {
                max_current_level_--;
            }
        }
    }

    /**
     * Search by Key.
     * @param search_key target Key
     * @return 
     */
    const NodeType *find(K search_key)
    {
        NodeType *curr_node = header_node_;
        for (int level = max_current_level_; level >= 1; level--)
        {
            while (curr_node->forwards[level]->key < search_key)
            {
                curr_node = curr_node->forwards[level];
            }
        }
        curr_node = curr_node->forwards[1];
        if (curr_node->key == search_key)
        {
            return curr_node;
        }
        else
        {
            return NULL;
        }
    }

    /**
     * Search for node with nearest Key.
     * @param search_key target Key
     * @param previous TRUE if previous node (with respect to SkipList oreder) 
     * is required, FALSE otherwise.
     * @return 
     */
    NodeType *findNearest(K search_key, bool previous = false)
    {
        NodeType *curr_node = header_node_;
        for (int level = max_current_level_; level >= 1; level--)
        {
            while (curr_node->forwards[level]->key < search_key)
            {
                curr_node = curr_node->forwards[level];
            }
        }
        if (previous)
        {
            return curr_node;
        }
        else
        {
            curr_node = curr_node->forwards[1];
            return curr_node;
        }
    }

    /**
     * @return TRUE if list is empty.
     */
    bool empty() const
    {
        return (header_node_->forwards[1] == tail_node_);
    }

    /**
     * @return String representation of SkipList
     */
    std::string toString()
    {
        std::stringstream sstr;
        NodeType *curr_node = header_node_->forwards[1];
        while (curr_node != tail_node_)
        {
            sstr << "(" << curr_node->key << "," << curr_node->value << ")" << std::endl;
            curr_node = curr_node->forwards[1];
        }
        return sstr.str();
    }

    /**
     * Iterates list and return an ordered Vector of Nodes
     * @param nodes OUTPUT vector of Nodes
     */
    void retrieveNodes(std::vector<NodeType *> &nodes)
    {
        nodes.clear();
        NodeType *curr_node = header_node_->forwards[1];
        while (curr_node != tail_node_)
        {
            nodes.push_back(curr_node);
            curr_node = curr_node->forwards[1];
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
        nodes.clear();
        NodeType *curr_node = start;
        while (curr_node != tail_node_)
        {
            nodes.push_back(curr_node);
            if (curr_node->key >= end_key)
                return;
            curr_node = curr_node->forwards[1];
        }
    }

    /**
     * Iterates list between two Keys and return an ordered Vector of Nodes
     * @param min_key min Key
     * @param max_key max Key
     * @param nodes OUTPUT vector of Nodes
     */
    void retrieveNodesByRange(K min_key, K max_key, std::vector<NodeType *> &nodes)
    {
        NodeType *start = findNearest(min_key, true);
        if (start != NULL && start->key == min_key_)
        {
            start = start->forwards[1];
        }
        retrieveNodes(nodes, start, max_key);
    }

    /**
     * @return  First Node of the list.
     */
    const NodeType *first()
    {
        return header_node_->forwards[1];
    }

    /**
     * Returns last node. Requires a search O(nlog(n))
     * @return Last Node of the list.
     */
    const NodeType *last()
    {
        NodeType *curr_node = header_node_;
        for (int level = max_current_level_; level >= 1; level--)
        {
            while (curr_node->forwards[level]->key < max_value_)
            {
                curr_node = curr_node->forwards[level];
            }
        }
        return curr_node;
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
     * @return random SkipList level
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
    K last_;
    int max_current_level_;
    int size_;
    SkipListNode<K, V, MAXLEVEL> *header_node_;
    SkipListNode<K, V, MAXLEVEL> *tail_node_;
};
}

#endif /* SKIPLIST_HPP */
