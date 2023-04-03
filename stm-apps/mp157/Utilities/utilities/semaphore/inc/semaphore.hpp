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
#ifndef _SEMAPHORE_HPP_
#define _SEMAPHORE_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <system_error>
#include <string>

#include "semaphore_interface.hpp"


#include <iostream>


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations
******************************************************************************/

namespace utils
{


/**
 * Implementation of Semaphore class
 */
class Semaphore : public SemaphoreI
{
    public:
        /**
         * Constructor of Semaphore
         *
         * @param   name    name of semaphore
         * @param   flags   flags for semaphore creation
         * @param   mode    mode of access
         * @param   value   initial value of semaphore
         *          
         * @throws  system_error for creation failure of semaphore
         *
        */
        explicit Semaphore( std::string name, 
            SemaphoreI::SemaphoreFlags flags = SemaphoreI::SemaphoreFlags::SEM_NO_FLAGS, 
            unsigned int value = 0 )
        :   flags_(flags), semName_(name)
        {
            // Initialize Semaphore according to params provided
            if( !semName_.empty() ){
                // Create Named Semaphore
                sem_ = sem_open( semName_.c_str(), static_cast<int32_t>(flags_), 0644, value );
                // Check creation status of semaphore
                if( SEM_FAILED == sem_ )
                {
                    perror("Could not Initialize Semaphore!! Exiting with Exception");
                    // Exit with Exception in Constructor
                    throw std::system_error(errno, std::system_category(), "Named Sem Init");
                }
            } else{
                // Create UnNamed Semaphore to be used within threads
                int32_t ret = sem_init( sem_, 0, value );
                // Check status
                if( -1 == ret )
                {
                    perror("Could not Initialize Semaphore!! Exiting with Exception");
                    // Exit with Exception in Constructor
                    throw std::system_error(errno, std::system_category(), "UnNamed Sem Init");
                }
            }

            // Semaphore is created successfully
        }

        /**
         * Default Constructor of Semaphore
         * 
        */
        Semaphore() = delete;

        /**
         * Copy Constructor of Semaphore
         *
         * @param   other   object of other Semaphore
         * @returns Copy of input object
         *
        */
        Semaphore( Semaphore& other ) = delete;

        /**
         * Move Constructor of Semaphore
         * 
         * @param   other   Object of other Semaphore
         * @returns Object after Move operation
         *
        */
        Semaphore( Semaphore&& other ) noexcept = default;

        /**
         * Copy Assignement of Semaphore
         * 
         * @param   other   Object of other Semaphore
         * @returns Reference of copied object
         *
        */
        Semaphore& operator=( Semaphore& other ) = delete;

        /**
         * Move Assignment of Semaphore
         * 
         * @param   other   Object of other Semaphore
         * @returns Object reference of moved object
         *
        */
        Semaphore& operator=( Semaphore&& other ) noexcept = default;

        /**
         * Destructor of Semaphore
         *
        */
        virtual ~Semaphore();


        /**
         * Increment semaphore value
         *
         * @param
         * @returns int32_t return status of operation  
         *
        */
        virtual int32_t post() noexcept override
        {
            return sem_post( sem_ );
        }

        /**
         * Wait for semaphore
         *
         * @param
         * @returns int32_t return status of operation  
         *
        */
        virtual int32_t wait() noexcept override
        {
            return sem_wait( sem_ );
        }

        /**
         * Try Wait for semaphore, if not available then return
         *
         * @param
         * @returns int32_t return status of operation  
         *
        */
        virtual int32_t try_wait() noexcept override
        {
            return sem_trywait( sem_ );
        }

        /**
         * Timed Wait for semaphore
         *
         * @param [in]   ms  time in milli seconds
         * @returns int32_t return status of operation  
         *
        */
        virtual int32_t timed_wait( uint32_t const ms ) noexcept override
        {
            // Initialize time structure
            const struct timespec time = {.tv_sec = 0, .tv_nsec = ms/1000000};
            return sem_timedwait( sem_, &time );
        }

        /**
         * Timed Wait for semaphore
         *
         * @param [inout]   sval pointer to value
         * @returns int32_t return status of operation
         *
        */
        virtual int32_t get_value( int *sval ) const noexcept override
        {
            return sem_getvalue( sem_, sval );
        }

    private:
        // Semaphore variable
        sem_t * sem_;

        // Flags used to create semaphore
        SemaphoreI::SemaphoreFlags flags_;

        // Name of Named Semaphore
        // Note: Could use std::optional iff std > c++14
        std::string semName_;
};


}   // namespace utils

#endif  // semaphore.hpp

/********************************** End of File *******************************/