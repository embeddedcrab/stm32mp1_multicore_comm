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
#ifndef _SHARED_MEMORY_INTERFACE_HPP_
#define _SHARED_MEMORY_INTERFACE_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <cstdint>

#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <unistd.h>
#include <sys/mman.h>

#include "pmr_headers.hpp"


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations
******************************************************************************/

namespace utils
{


/**
 * Implementation of Shared Memory Interface
 */
class SharedMemoryI
{
    public:

        /**
         * Enum for Flags available for Memory Mapping
         * 
        */
        enum class ShmMapFlags: int32_t{
            SHM_MAP_SHARED = MAP_SHARED,
            SHM_MAP_PRIVATE = MAP_PRIVATE,
            SHM_MAP_SHARED_VALIDATE = MAP_SHARED_VALIDATE
        };

        /**
         * Enum for Protocol available for Memory Mapping
         * 
        */
        enum class ShmMapProto: int32_t{
            SHM_MAP_PROT_NONE = PROT_NONE,
            SHM_MAP_PROT_READ = PROT_READ,
            SHM_MAP_PROT_WRITE = PROT_WRITE,
            SHM_MAP_PROT_RW = PROT_READ | PROT_WRITE,
            SHM_MAP_PROT_EXEC = PROT_EXEC
        };

        /**
         * Enum for Flags available for Shared Memory
         * 
        */
        enum class ShmFlags: int32_t{
            SHM_CREATE = O_CREAT,
            SHM_RDONLY = O_RDONLY,
            SHM_RW = O_RDWR,
            SHM_EXCL = O_EXCL,
            SHM_TRUNC = O_TRUNC,
            SHM_CREATE_RDWR = O_CREAT | O_RDWR
        };

        /**
         * Destructor of SharedMemoryI
         *
        */
        virtual ~SharedMemoryI() = default;

        /**
         * Basic Functions of SemaphoreI
         * 
        */
        // Functions to memory map and unmap shared memory to virtual address space of process
        virtual int32_t map( const unsigned long offset ) noexcept = 0;
        virtual int32_t unmap() noexcept = 0;

        // Function to get statistics of shared memory
        virtual struct stat get_stats() noexcept = 0;

        // Function to get shared memory pointer for data transfer
        virtual void * get_shm_ptr() const noexcept = 0;

    protected:

};


}   // namespace utils

#endif  // shared_memory_interface.hpp

/********************************** End of File *******************************/