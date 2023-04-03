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


/******************************************************************************
* Includes
******************************************************************************/
#include <cstddef>
#include <climits>

#include <algorithm>

#include "monotonic_buffer_resource.hpp"


/******************************************************************************
* Constant Declarations
******************************************************************************/


/******************************************************************************
* Class Function Definitions
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

namespace {

typedef std::size_t        uintptr_type;
static const std::size_t minimum_buffer_size = 2*sizeof(void*);

}  //namespace {


namespace pmr {

/// @brief Some Math Utility functions, not separated from pmr but should be!

////////////////////////////
// DeBruijn method
////////////////////////////

//Taken from:
//http://stackoverflow.com/questions/11376288/fast-computing-of-log2-for-64-bit-integers
//Thanks to Desmond Hume

inline std::size_t floor_log2(std::size_t v, std::integral_constant<std::size_t, 32>)
{
  static const int MultiplyDeBruijnBitPosition[32] =
      {
          0, 9, 1, 10, 13, 21, 2, 29, 11, 14, 16, 18, 22, 25, 3, 30,
          8, 12, 20, 28, 15, 17, 24, 7, 19, 27, 23, 6, 26, 5, 4, 31};

  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;

  return MultiplyDeBruijnBitPosition[(std::size_t)(v * 0x07C4ACDDU) >> 27];
}

inline std::size_t floor_log2(std::size_t v, std::integral_constant<std::size_t, 64>)
{
  static const std::size_t MultiplyDeBruijnBitPosition[64] = {
      63, 0, 58, 1, 59, 47, 53, 2,
      60, 39, 48, 27, 54, 33, 42, 3,
      61, 51, 37, 40, 49, 18, 28, 20,
      55, 30, 34, 11, 43, 14, 22, 4,
      62, 57, 46, 52, 38, 26, 32, 41,
      50, 36, 17, 19, 29, 10, 13, 21,
      56, 45, 25, 31, 35, 16, 9, 12,
      44, 24, 15, 8, 23, 7, 6, 5};

  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v |= v >> 32;
  return MultiplyDeBruijnBitPosition[((std::size_t)((v - (v >> 1)) * 0x07EDD5E59A4E28C2ULL)) >> 58];
}

inline std::size_t floor_log2(std::size_t x)
{
  const std::size_t Bits = sizeof(std::size_t) * CHAR_BIT;
  return floor_log2(x, std::integral_constant<std::size_t, Bits>());
}

inline bool is_pow2(std::size_t x)
{
  return (x & (x - 1)) == 0;
}

template <std::size_t N>
struct static_is_pow2
{
  static const bool value = (N & (N - 1)) == 0;
};

inline std::size_t ceil_log2(std::size_t x)
{
  return static_cast<std::size_t>(!(is_pow2)(x)) + floor_log2(x);
}

inline std::size_t ceil_pow2(std::size_t x)
{
  return std::size_t(1u) << (ceil_log2)(x);
}

inline std::size_t previous_or_equal_pow2(std::size_t x)
{
   return std::size_t(1u) << floor_log2(x);
}


void monotonic_buffer_resource::increase_next_buffer()
{
   m_next_buffer_size = (std::size_t(-1)/2 < m_next_buffer_size) ? std::size_t(-1) : m_next_buffer_size*2;
}

void monotonic_buffer_resource::increase_next_buffer_at_least_to(std::size_t minimum_size)
{
   if(m_next_buffer_size < minimum_size){
      if(is_pow2(minimum_size)){
         m_next_buffer_size = minimum_size;
      }
      else if(std::size_t(-1)/2 < minimum_size){
         m_next_buffer_size = minimum_size;
      }
      else{
         m_next_buffer_size = ceil_pow2(minimum_size);
      }
   }
}

monotonic_buffer_resource::monotonic_buffer_resource(memory_resource* upstream) noexcept
   : m_memory_blocks(upstream ? *upstream : *get_default_resource())
   , m_current_buffer(0)
   , m_current_buffer_size(0u)
   , m_next_buffer_size(initial_next_buffer_size)
   , m_initial_buffer(0)
   , m_initial_buffer_size(0u)
{}

monotonic_buffer_resource::monotonic_buffer_resource(std::size_t initial_size, memory_resource* upstream) noexcept
   : m_memory_blocks(upstream ? *upstream : *get_default_resource())
   , m_current_buffer(0)
   , m_current_buffer_size(0u)
   , m_next_buffer_size(minimum_buffer_size)
   , m_initial_buffer(0)
   , m_initial_buffer_size(0u)
{                                         //In case initial_size is zero
   this->increase_next_buffer_at_least_to(initial_size + !initial_size);
}

monotonic_buffer_resource::monotonic_buffer_resource(void* buffer, std::size_t buffer_size, memory_resource* upstream) noexcept
   : m_memory_blocks(upstream ? *upstream : *get_default_resource())
   , m_current_buffer(buffer)
   , m_current_buffer_size(buffer_size)
   , m_next_buffer_size
      (previous_or_equal_pow2
         (std::max(buffer_size, std::size_t(initial_next_buffer_size))))
   , m_initial_buffer(buffer)
   , m_initial_buffer_size(buffer_size)
{  this->increase_next_buffer(); }

monotonic_buffer_resource::~monotonic_buffer_resource()
{  this->release();  }

void monotonic_buffer_resource::release() noexcept
{
   m_memory_blocks.release();
   m_current_buffer = m_initial_buffer;
   m_current_buffer_size = m_initial_buffer_size;
   m_next_buffer_size = initial_next_buffer_size;
}

memory_resource* monotonic_buffer_resource::upstream_resource() const noexcept
{  return &m_memory_blocks.upstream_resource();   }

std::size_t monotonic_buffer_resource::remaining_storage(std::size_t alignment, std::size_t &wasted_due_to_alignment) const noexcept
{
   const uintptr_type up_alignment_minus1 = alignment - 1u;
   const uintptr_type up_alignment_mask = ~up_alignment_minus1;
   const uintptr_type up_addr = uintptr_type(m_current_buffer);
   const uintptr_type up_aligned_addr = (up_addr + up_alignment_minus1) & up_alignment_mask;
   wasted_due_to_alignment = std::size_t(up_aligned_addr - up_addr);
   return m_current_buffer_size <= wasted_due_to_alignment ? 0u : m_current_buffer_size - wasted_due_to_alignment;
}

std::size_t monotonic_buffer_resource::remaining_storage(std::size_t alignment) const noexcept
{
   std::size_t ignore_this;
   return this->remaining_storage(alignment, ignore_this);
}

const void *monotonic_buffer_resource::current_buffer() const noexcept
{  return m_current_buffer;  }

std::size_t monotonic_buffer_resource::next_buffer_size() const noexcept
{  return m_next_buffer_size;  }

void *monotonic_buffer_resource::allocate_from_current(std::size_t aligner, std::size_t bytes)
{
   char * p = (char*)m_current_buffer + aligner;
   m_current_buffer = p + bytes;
   m_current_buffer_size -= aligner + bytes;
   return p;
}

void* monotonic_buffer_resource::do_allocate(std::size_t bytes, std::size_t alignment)
{
   if(alignment > memory_resource::max_align){
     (void)bytes; (void)alignment;
      throw std::bad_alloc();
   }

   //See if there is room in current buffer
   std::size_t aligner = 0u;
   if(this->remaining_storage(alignment, aligner) < bytes){
      //The new buffer will be aligned to the strictest alignment so reset
      //the aligner, which was needed for the old buffer.
      aligner = 0u;
      //Update next_buffer_size to at least bytes
      this->increase_next_buffer_at_least_to(bytes);
      //Now allocate and update internal data
      m_current_buffer = (char*)m_memory_blocks.allocate(m_next_buffer_size);
      m_current_buffer_size = m_next_buffer_size;
      this->increase_next_buffer();
   }
   //Enough internal storage, extract from it
   return this->allocate_from_current(aligner, bytes);
}

void monotonic_buffer_resource::do_deallocate(void* p, std::size_t bytes, std::size_t alignment) noexcept
{  (void)p; (void)bytes;  (void)alignment;  }

bool monotonic_buffer_resource::do_is_equal(const memory_resource& other) const noexcept
{  return this == &other;  }


} //namespace pmr {


/********************************** End of File *******************************/