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
#ifndef _SHARED_MEMORY_HPP_
#define _SHARED_MEMORY_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <cstring>
#include <functional>
#include <vector>

#include "shared_memory_interface.hpp"
#include "semaphore.hpp"


#include <iostream>


/******************************************************************************
* Configuration Constants
******************************************************************************/

constexpr uint32_t DefaultSharedMemorySize{4096U};


/******************************************************************************
* Class Declarations
******************************************************************************/

namespace utils
{


/**
 * Implementation of Shared Memory class
 */
template <typename T>
class SharedMemory : public SharedMemoryI
{
    public:

        /**
         * Constructor of Shared Memory
         *
         * @param[in]   name            name of shared memory segment
         * @param[in]   seminterface&   semaphore interface reference
         * @param[in]   lentgth         length of shared memory segment
         * @param[in]   flags           flag options for shared memory
         *          
         * @throws  system_error for creation failure of Shared Memory
         *
        */
        explicit SharedMemory( std::string name, uint32_t length, Semaphore& semInterface,
            SharedMemoryI::ShmFlags flags = SharedMemoryI::ShmFlags::SHM_RW)
        :   psharedMemory_{NULL}, size_{length}, flagsOpts_{flags}, 
            stats_{}, shmName_{name}, semaphoreInterface_(semInterface)
        {
            // Initialize Shared Memory
            fd_ = shm_open( shmName_.c_str(), static_cast<int32_t>(flagsOpts_), 0644 );

            // Set length of Shared Memory
            int32_t ret = ftruncate(fd_, static_cast<off_t>(size_));
            if( (fd_ < 0) || (-1 == ret) ){
                perror("Could not Initialize Shared Memory!! Exiting with Exception");
                if( fd_ > 0 ){
                    // Destroy SHM
                    ret = shm_unlink(shmName_.c_str());
                }
                // Exit with Exception in Constructor
                throw std::system_error(errno, std::system_category(), "SHM Init");
            }
        }

        /**
         * Default Constructor of SharedMemory
         *
        */
        SharedMemory() = delete;

        /**
         * Copy Constructor of SharedMemory
         *
         * @param   other   Object of other Shared Memory
         * @returns Object reference of moved object 
         *
        */
        SharedMemory( SharedMemory& other ) = delete;

        /**
         * Move Constructor of SharedMemory
         *
         * @param   other   Object of Shared Memory to be moved
         * @returns Object of moved instance
         *
        */
        SharedMemory( SharedMemory&& other ) noexcept = default;

        /**
         * Copy Assignement of SharedMemory
         *
         * @param   other   Object of other Shared Memory
         * @returns Reference of copied object
         *
        */
        SharedMemory& operator=( SharedMemory& other ) = delete;

        /**
         * Move Assignment of SharedMemory
         *
         * @param   other   Object tpo be moved
         * @returns Reference of moved object
         *
        */
        SharedMemory& operator=( SharedMemory&& other ) noexcept = default;

        /**
         * Destructor of SharedMemory
         *
        */
        virtual ~SharedMemory()
        {
            std::cout << "~SharedMemory() Start" << std::endl;
            // Unmap memory if not done already
            if( (psharedMemory_ != NULL) && (psharedMemory_ != MAP_FAILED) )
            {
                std::cout << "UnMapping Shared Memory" << std::endl;
                static_cast<void>(munmap(psharedMemory_, size_));
            }

            // Close Shared Memory
            static_cast<void>(close(fd_));

            // Check flags and destroy SHM, checking for O_CREAT
            if( static_cast<int32_t>(flagsOpts_) & 0x40 ){
                static_cast<void>(shm_unlink(shmName_.c_str()));
                std::cout << "~Destructor() ShmUnlink()" << std::endl;
            }
            std::cout << "~Destructor() Shared Memory Finished" << std::endl;
        }

        /**
         * Map SHM in virtual address space
         *
         * @param [in]  offset  offset in file
         * @returns 0   success
         *          1   failed
         *
        */
        virtual int32_t map( const unsigned long offset = 0 ) noexcept override
        {
            off_t of{0};

            // Set offset value iff, it is in multiple of PAGE SIZE
            if( offset ){
                if( 0 == offset % sysconf(_SC_PAGE_SIZE) ){
                    of = offset;
                }
            }

            // Map memory
            psharedMemory_ = (void *)mmap( NULL, size_, 
                                static_cast<int32_t>(SharedMemoryI::ShmMapProto::SHM_MAP_PROT_RW), 
                                static_cast<int32_t>(SharedMemoryI::ShmMapFlags::SHM_MAP_SHARED), 
                                fd_, of );
            
            // return status
            return !( psharedMemory_ != MAP_FAILED );
        }

        /**
         * Unmap SHM in virtual address space
         *
         * @param   NA
         * @returns 0   success
         *          1   failed
         *
        */
        virtual int32_t unmap() noexcept override
        {
            int32_t ret{0};

            // Check memory status and process accordingly
            if( psharedMemory_ != MAP_FAILED )
            {
                ret = munmap(psharedMemory_, size_);
            }

            return (!ret ? 0 : 1);
        }

        /**
         * Get Stats of shared memory
         *
         * @param   NA
         * @returns stat    structure containing corresponding data
         *
        */
        virtual struct stat get_stats() noexcept override
        {
            int ret = fstat(fd_, &stats_);
            return stats_;
        }

        /**
         * Get shared memory pointer
         *
         * @param   NA
         * @returns void *  pointer to shared memory
         *
        */
        virtual void * get_shm_ptr() const noexcept override
        {
            return psharedMemory_;
        }

        /**
         * Write data into shared memory
         *
         * @param[in]   in          input data to be written into SHM
         * @param[in]   sizeOfData  data size to be written
         * @returns 0   success
         *          1   failed
         *
        */
        int32_t write( const T& in, size_t sizeOfData ) noexcept
        {
            void * ret{nullptr};

            if( sizeOfData > size_ ){
                return 1; 
            } else{
                ret = (void *) memcpy(psharedMemory_, static_cast<const void *>(&in), sizeOfData);
            }
            
            return (ret != NULL) ? 0 : 1;
        }

        /**
         * Write data into shared memory
         *
         * @param[in]   in  input data to be written into SHM
         * @param[in]   addrOffset  Offset address from base address
         *                          should be less than size of memory after calculation,
         *                          users responsibility to check validity
         * @param[in]   sizeOfData  data size to be written
         * @returns 0   success
         *          1   failed
         *
        */
        int32_t write( const T& in, uint32_t addrOffset, size_t sizeOfData ) noexcept
        {
            void * ret{nullptr};

            if( sizeOfData > size_ ){
                return 1; 
            } else{
                // Offset is being calculated by using uint8_t * i.e., byte addressing
                ret = (void *) memcpy( static_cast<void *>(static_cast<uint8_t *>(psharedMemory_) + addrOffset), 
                                        static_cast<const void *>(&in), sizeOfData);
            }
            
            return (ret != NULL) ? 0 : 1;
        }

        /**
         * Read data from shared memory
         *
         * @param[inout]    out         output data to be read from SHM
         * @param[in]       sizeOfData  size of data to be read
         * @returns 0       success
         *          1       failed
         *
        */
        int32_t read( T& out, size_t sizeOfData ) noexcept
        {
            void * ret = (void *) memcpy(static_cast<void *>(&out), psharedMemory_, sizeOfData);
            return (ret != NULL) ? 0 : 1;
        }

        /**
         * Read data from shared memory
         *
         * @param[inout]    out         output data to be read from SHM
         * @param[in]       addrOffset   offset address from base address
         * @param[in]       sizeOfData  size of data to be read
         * @returns 0       success
         *          1       failed
         *
        */
        int32_t read( T& out, uint32_t addrOffset, size_t sizeOfData ) noexcept
        {
            // Offset is being calculated by using uint8_t * i.e., byte addressing
            void * ret = (void *) memcpy(static_cast<void *>(&out), 
                            static_cast<void *>(static_cast<uint8_t *>(psharedMemory_) + addrOffset ),
                            sizeOfData);
            return (ret != NULL) ? 0 : 1;
        }

        /**
         * Get semaphore associated with this shared memory
         *
         * @param   NA
         * @returns semaphore object reference
         *
        */
        Semaphore * getSemaphore() const
        {
            return const_cast<Semaphore *>(&(semaphoreInterface_.get()));
        }

    private:
        // Pointer to shared memory
        void * psharedMemory_;

        // Length of shared memory
        uint32_t size_;

        // File Descriptor of Shared Memory
        int32_t fd_;

        // Flags for shared memory segment
        SharedMemoryI::ShmFlags flagsOpts_;

        // Shared Memory statistics
        struct stat stats_;

        // Name of Shared Memory
        std::string shmName_;

        // Reference of Semaphore
        std::reference_wrapper<Semaphore> semaphoreInterface_;

        // Local class variable to read std::string data from SHM
        [[maybe_unused]] std::vector<char> buffer_;
};


// Template specilization for Read and Write operations
template <>
int32_t SharedMemory<std::string>::write( const std::string& in, size_t sizeOfData ) noexcept
{
    void * ret{nullptr};

    if( sizeOfData > size_ ){
        return 1; 
    } else{
        ret = (void *) memcpy(psharedMemory_, static_cast<const void *>(in.c_str()), sizeOfData);
    }
 
    return (ret != NULL) ? 0 : 1;
}

template <>
int32_t SharedMemory<std::string>::write( const std::string& in, uint32_t addrOffset, size_t sizeOfData ) noexcept
{
    void * ret{nullptr};

    if( sizeOfData > size_ ){
        return 1; 
    } else{
        // Offset is being calculated by using uint8_t * i.e., byte addressing
        ret = (void *) memcpy( static_cast<void *>(static_cast<uint8_t *>(psharedMemory_) + addrOffset), 
                                static_cast<const void *>(in.c_str()), sizeOfData);
    }
            
    return (ret != NULL) ? 0 : 1;
}

template <>
int32_t SharedMemory<std::string>::read( std::string& out, size_t sizeOfData ) noexcept
{
    // Read data into buffer and then transfer to string inout variable
    buffer_.resize(sizeOfData);

    void * ret = (void *) memcpy(static_cast<void *>(&buffer_[0]), psharedMemory_, sizeOfData);
    if( ret != NULL ){
        // Copy data into string
        out.assign(&buffer_[0], buffer_.size()) ;
        buffer_.clear();
    }
    return (ret != NULL) ? 0 : 1;
}

template <>
int32_t SharedMemory<std::string>::read( std::string& out, uint32_t addrOffset, size_t sizeOfData ) noexcept
{
    // Read data into buffer and then transfer to string inout variable
    buffer_.resize(sizeOfData);

    // Offset is being calculated by using uint8_t * i.e., byte addressing
    void * ret = (void *) memcpy(static_cast<void *>(&buffer_[0]), 
                            static_cast<void *>(static_cast<uint8_t *>(psharedMemory_) + addrOffset ),
                            sizeOfData);
    if( ret != NULL ){
        out.assign(&buffer_[0], buffer_.size());
    }
    return (ret != NULL) ? 0 : 1;
}


}   // namespace utils

#endif  // shared_memory.hpp

/********************************** End of File *******************************/