/*********************************************************************************************************************
 * @file     
 * @brief    
 * @version  1.0.0
 * @date	 03/08/2023
 *
 * @cond
 *********************************************************************************************************************
 * Copyright (c) 2023, EmbeddedCrab (Hemant Sharma) - All Rights Reserved
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************************************************
 *
 * *************************** Change history ********************************
 *
 * @endcond
 */
/******************************************************************************
* Notes:
*
* Change History
* --------------
*
*******************************************************************************/

/** @file:	
 *  @brief:	This file contains.
 */
#ifndef _MONOTONIC_BUFFER_RESOURCE_HPP_
#define _MONOTONIC_BUFFER_RESOURCE_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include "polymorphic_allocator.hpp"


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations
******************************************************************************/

//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright Ion Gaztanaga 2015-2015. Distributed under the Boost
// Software License, Version 1.0. (See accompanying file
// LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/container for documentation.
//
//////////////////////////////////////////////////////////////////////////////

/// @brief Local to this file only
struct custom_new_t
{};

// avoid including <new>
inline void *operator new(std::size_t, void *p, custom_new_t)
{
  return p;
}

inline void operator delete(void *, void *, custom_new_t)
{}

// namespace pmr
namespace pmr
{

template <class NodeTraits>
class common_slist_algorithms
{
  public:
    typedef typename NodeTraits::node node;
    typedef typename NodeTraits::node_ptr node_ptr;
    typedef typename NodeTraits::const_node_ptr const_node_ptr;
    typedef NodeTraits node_traits;

    static node_ptr get_previous_node(node_ptr p, const node_ptr &this_node)
    {
      for (node_ptr p_next; this_node != (p_next = NodeTraits::get_next(p)); p = p_next)
      {
        // Logic error: possible use of linear lists with
        // operations only permitted with circular lists
        assert(p);
      }
      return p;
    }

    inline static void init(const node_ptr &this_node)
    {
      NodeTraits::set_next(this_node, node_ptr());
    }

    inline static bool unique(const const_node_ptr &this_node)
    {
      node_ptr next = NodeTraits::get_next(this_node);
      return !next || next == this_node;
    }

    inline static bool inited(const const_node_ptr &this_node)
    {
      return !NodeTraits::get_next(this_node);
    }

    inline static void unlink_after(const node_ptr &prev_node)
    {
      const_node_ptr this_node(NodeTraits::get_next(prev_node));
      NodeTraits::set_next(prev_node, NodeTraits::get_next(this_node));
    }

    inline static void unlink_after(const node_ptr &prev_node, const node_ptr &last_node)
    {
      NodeTraits::set_next(prev_node, last_node);
    }

    inline static void link_after(const node_ptr &prev_node, const node_ptr &this_node)
    {
      NodeTraits::set_next(this_node, NodeTraits::get_next(prev_node));
      NodeTraits::set_next(prev_node, this_node);
    }

    inline static void incorporate_after(const node_ptr &bp, const node_ptr &b, const node_ptr &be)
    {
      node_ptr p(NodeTraits::get_next(bp));
      NodeTraits::set_next(bp, b);
      NodeTraits::set_next(be, p);
    }

    static void transfer_after(const node_ptr &bp, const node_ptr &bb, const node_ptr &be)
    {
      if (bp != bb && bp != be && bb != be)
      {
        node_ptr next_b = NodeTraits::get_next(bb);
        node_ptr next_e = NodeTraits::get_next(be);
        node_ptr next_p = NodeTraits::get_next(bp);
        NodeTraits::set_next(bb, next_e);
        NodeTraits::set_next(be, next_p);
        NodeTraits::set_next(bp, next_b);
      }
    }

    struct stable_partition_info
    {
      std::size_t num_1st_partition;
      std::size_t num_2nd_partition;
      node_ptr beg_2st_partition;
      node_ptr new_last_node;
    };

    template <class Pred>
    static void stable_partition(node_ptr before_beg, const node_ptr &end, Pred pred, stable_partition_info &info)
    {
      node_ptr bcur = before_beg;
      node_ptr cur = node_traits::get_next(bcur);
      node_ptr new_f = end;

      std::size_t num1 = 0, num2 = 0;
      while (cur != end)
      {
        if (pred(cur))
        {
          ++num1;
          bcur = cur;
          cur = node_traits::get_next(cur);
        }
        else
        {
          ++num2;
          node_ptr last_to_remove = bcur;
          new_f = cur;
          bcur = cur;
          cur = node_traits::get_next(cur);
          try
          {
            // Main loop
            while (cur != end)
            {
              if (pred(cur))
              { // Might throw
                ++num1;
                // Process current node
                node_traits::set_next(last_to_remove, cur);
                last_to_remove = cur;
                node_ptr nxt = node_traits::get_next(cur);
                node_traits::set_next(bcur, nxt);
                cur = nxt;
              }
              else
              {
                ++num2;
                bcur = cur;
                cur = node_traits::get_next(cur);
              }
            }
          }
          catch (...)
          {
            node_traits::set_next(last_to_remove, new_f);
            throw;
          }
          node_traits::set_next(last_to_remove, new_f);
          break;
        }
      }
      info.num_1st_partition = num1;
      info.num_2nd_partition = num2;
      info.beg_2st_partition = new_f;
      info.new_last_node = bcur;
    }

    static std::size_t distance(const const_node_ptr &f, const const_node_ptr &l)
    {
      const_node_ptr i(f);
      std::size_t result = 0;
      while (i != l)
      {
        i = NodeTraits::get_next(i);
        ++result;
      }
      return result;
    }
};


template <class NodeTraits>
class linear_slist_algorithms : public common_slist_algorithms<NodeTraits>
{
  typedef common_slist_algorithms<NodeTraits> base_t;

  public:
    typedef typename NodeTraits::node node;
    typedef typename NodeTraits::node_ptr node_ptr;
    typedef typename NodeTraits::const_node_ptr const_node_ptr;
    typedef NodeTraits node_traits;

    inline static void init_header(const node_ptr &this_node)
    {
      NodeTraits::set_next(this_node, node_ptr());
    }

    inline static node_ptr get_previous_node(const node_ptr &prev_init_node, const node_ptr &this_node)
    {
      return base_t::get_previous_node(prev_init_node, this_node);
    }

    static std::size_t count(const const_node_ptr &this_node)
    {
      std::size_t result = 0;
      const_node_ptr p = this_node;
      do
      {
        p = NodeTraits::get_next(p);
        ++result;
      } while (p);
      return result;
    }

    static void swap_trailing_nodes(const node_ptr &this_node, const node_ptr &other_node)
    {
      node_ptr this_nxt = NodeTraits::get_next(this_node);
      node_ptr other_nxt = NodeTraits::get_next(other_node);
      NodeTraits::set_next(this_node, other_nxt);
      NodeTraits::set_next(other_node, this_nxt);
    }

    static node_ptr reverse(const node_ptr &p)
    {
      if (!p)
        return node_ptr();
      node_ptr i = NodeTraits::get_next(p);
      node_ptr first(p);
      while (i)
      {
        node_ptr nxti(NodeTraits::get_next(i));
        base_t::unlink_after(p);
        NodeTraits::set_next(i, first);
        first = i;
        i = nxti;
      }
      return first;
    }

    static std::pair<node_ptr, node_ptr> move_first_n_backwards(const node_ptr &p, std::size_t n)
    {
      std::pair<node_ptr, node_ptr> ret;
      // Null shift, or count() == 0 or 1, nothing to do
      if (!n || !p || !NodeTraits::get_next(p))
      {
        return ret;
      }

      node_ptr first = p;
      bool end_found = false;
      node_ptr new_last = node_ptr();
      node_ptr old_last = node_ptr();

      // Now find the new last node according to the shift count.
      // If we find 0 before finding the new last node
      // unlink p, shortcut the search now that we know the size of the list
      // and continue.
      for (std::size_t i = 1; i <= n; ++i)
      {
        new_last = first;
        first = NodeTraits::get_next(first);
        if (first == node_ptr())
        {
          // Shortcut the shift with the modulo of the size of the list
          n %= i;
          if (!n)
            return ret;
          old_last = new_last;
          i = 0;
          // Unlink p and continue the new first node search
          first = p;
          // unlink_after(new_last);
          end_found = true;
        }
      }

      // If the p has not been found in the previous loop, find it
      // starting in the new first node and unlink it
      if (!end_found)
      {
        old_last = base_t::get_previous_node(first, node_ptr());
      }

      // Now link p after the new last node
      NodeTraits::set_next(old_last, p);
      NodeTraits::set_next(new_last, node_ptr());
      ret.first = first;
      ret.second = new_last;
      return ret;
    }

    static std::pair<node_ptr, node_ptr> move_first_n_forward(const node_ptr &p, std::size_t n)
    {
      std::pair<node_ptr, node_ptr> ret;
      // Null shift, or count() == 0 or 1, nothing to do
      if (!n || !p || !NodeTraits::get_next(p))
        return ret;

      node_ptr first = p;

      // Iterate until p is found to know where the current last node is.
      // If the shift count is less than the size of the list, we can also obtain
      // the position of the new last node after the shift.
      node_ptr old_last(first), next_to_it, new_last(p);
      std::size_t distance = 1;
      while (!!(next_to_it = node_traits::get_next(old_last)))
      {
        if (distance++ > n)
          new_last = node_traits::get_next(new_last);
        old_last = next_to_it;
      }
      // If the shift was bigger or equal than the size, obtain the equivalent
      // forward shifts and find the new last node.
      if (distance <= n)
      {
        // Now find the equivalent forward shifts.
        // Shortcut the shift with the modulo of the size of the list
        std::size_t new_before_last_pos = (distance - (n % distance)) % distance;
        // If the shift is a multiple of the size there is nothing to do
        if (!new_before_last_pos)
          return ret;

        for (new_last = p; --new_before_last_pos; new_last = node_traits::get_next(new_last))
        {
          // empty
        }
      }

      // Get the first new node
      node_ptr new_first(node_traits::get_next(new_last));
      // Now put the old beginning after the old end
      NodeTraits::set_next(old_last, p);
      NodeTraits::set_next(new_last, node_ptr());
      ret.first = new_first;
      ret.second = new_last;
      return ret;
    }
};

struct slist_node
{
  slist_node *next;
};

struct slist_node_traits
{
  typedef slist_node node;
  typedef slist_node *node_ptr;
  typedef const slist_node *const_node_ptr;

  static node_ptr get_next(const_node_ptr n)
  {
    return n->next;
  }

  static void set_next(const node_ptr &n, const node_ptr &next)
  {
    n->next = next;
  }
};

struct block_slist_header : public slist_node
{
  std::size_t size;
};

typedef linear_slist_algorithms<slist_node_traits> slist_algo;

template <class DerivedFromBlockSlistHeader = block_slist_header>
class block_slist_base
{
  slist_node m_slist;

  static const std::size_t MaxAlignMinus1 = memory_resource::max_align - 1u;

  public:
    static const std::size_t header_size = std::size_t(sizeof(DerivedFromBlockSlistHeader) + MaxAlignMinus1) & std::size_t(~MaxAlignMinus1);

    explicit block_slist_base()
    {
      slist_algo::init_header(&m_slist);
    }

    block_slist_base(const block_slist_base &) = delete;
    block_slist_base operator=(const block_slist_base &) = delete;

    ~block_slist_base()
    {
    }

    void *allocate(std::size_t size, memory_resource &mr)
    {
      if ((size_t(-1) - header_size) < size)
        throw std::bad_alloc();
      void *p = mr.allocate(size + header_size);
      block_slist_header &mb = *::new ((void *)p, custom_new_t()) DerivedFromBlockSlistHeader;
      mb.size = size + header_size;
      slist_algo::link_after(&m_slist, &mb);
      return (char *)p + header_size;
    }

    void release(memory_resource &mr) noexcept
    {
      slist_node *n = slist_algo::node_traits::get_next(&m_slist);
      while (n)
      {
        DerivedFromBlockSlistHeader &d = static_cast<DerivedFromBlockSlistHeader &>(*n);
        n = slist_algo::node_traits::get_next(n);
        std::size_t size = d.block_slist_header::size;
        d.~DerivedFromBlockSlistHeader();
        mr.deallocate(reinterpret_cast<char *>(&d), size, memory_resource::max_align);
      }
      slist_algo::init_header(&m_slist);
    }
};

class block_slist : public block_slist_base<>
{
  memory_resource &m_upstream_rsrc;

  public:
    explicit block_slist(memory_resource &upstream_rsrc)
        : block_slist_base<>(), m_upstream_rsrc(upstream_rsrc)
    {
    }

    block_slist(const block_slist &) = delete;
    block_slist operator=(const block_slist &) = delete;

    ~block_slist()
    {
      this->release();
    }

    void *allocate(std::size_t size)
    {
      return this->block_slist_base<>::allocate(size, m_upstream_rsrc);
    }

    void release() noexcept
    {
      return this->block_slist_base<>::release(m_upstream_rsrc);
    }

    memory_resource &upstream_resource() const noexcept
    {
      return m_upstream_rsrc;
    }
};

//! A monotonic_buffer_resource is a special-purpose memory resource intended for
//! very fast memory allocations in situations where memory is used to build up a
//! few objects and then is released all at once when the memory resource object
//! is destroyed. It has the following qualities:
//!
//! - A call to deallocate has no effect, thus the amount of memory consumed
//!   increases monotonically until the resource is destroyed.
//!
//! - The program can supply an initial buffer, which the allocator uses to satisfy
//!   memory requests.
//!
//! - When the initial buffer (if any) is exhausted, it obtains additional buffers
//!   from an upstream memory resource supplied at construction. Each additional
//!   buffer is larger than the previous one, following a geometric progression.
//!
//! - It is intended for access from one thread of control at a time. Specifically,
//!   calls to allocate and deallocate do not synchronize with one another.
//!
//! - It owns the allocated memory and frees it on destruction, even if deallocate has
//!   not been called for some of the allocated blocks.

class monotonic_buffer_resource : public memory_resource
{
  public:
    static const std::size_t initial_next_buffer_size = 32u * sizeof(void *);

    explicit monotonic_buffer_resource(memory_resource *upstream = 0) noexcept;

    explicit monotonic_buffer_resource(std::size_t initial_size, memory_resource *upstream = 0) noexcept;

    // Default Constructor
    monotonic_buffer_resource() = delete;

    monotonic_buffer_resource(void *buffer, std::size_t buffer_size, memory_resource *upstream = 0) noexcept;

    monotonic_buffer_resource(const monotonic_buffer_resource &) = delete;
    monotonic_buffer_resource operator=(const monotonic_buffer_resource &) = delete;

    virtual ~monotonic_buffer_resource() noexcept;

    void release() noexcept;

    memory_resource *upstream_resource() const noexcept;

    std::size_t remaining_storage(std::size_t alignment, std::size_t &wasted_due_to_alignment) const noexcept;

    std::size_t remaining_storage(std::size_t alignment = 1u) const noexcept;

    const void *current_buffer() const noexcept;

    std::size_t next_buffer_size() const noexcept;

  private:
    block_slist m_memory_blocks;
    void *m_current_buffer;
    std::size_t m_current_buffer_size;
    std::size_t m_next_buffer_size;
    void *const m_initial_buffer;
    std::size_t const m_initial_buffer_size;

    void increase_next_buffer();
    void increase_next_buffer_at_least_to(std::size_t minimum_size);
    void *allocate_from_current(std::size_t aligner, std::size_t bytes);

  protected:
    virtual void *do_allocate(std::size_t bytes, std::size_t alignment) override;
    virtual void do_deallocate(void *p, std::size_t bytes, std::size_t alignment) noexcept override;
    virtual bool do_is_equal(const memory_resource &other) const noexcept override;
};

} // namespace pmr

#endif  // monotonic_buffer_resource.hpp

/********************************** End of File *******************************/