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
#ifndef _THREAD_POOL_HPP_
#define _THREAD_POOL_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include "thread_pool_interface.hpp"

#include <iostream>


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations
******************************************************************************/

namespace utils{


static const std::string ThreadName{"ThreadPoolDefault"};


/**
 * Implementation of Thread Pool class
 */
class ThreadPool : public ThreadPoolI
{
    public:
        /**
         * Constructor of ThreadPool
         *
         * @param[in]   numberOfThreads Thread Numbers for Thread Pool
         * @param[in]   name            Name of Thread
         * @return      NA
         *
        */
        explicit ThreadPool( uint32_t numberOfThreads, std::string name = ThreadName )
        :   shutdownRequestedFlag_{false}, threadCount_{numberOfThreads}, 
            syncMutex_{}, syncCondVar_{}, 
            threadName_{name}, functionQueue_{}, 
            threadSpace_(std::make_unique<std::thread[]>(threadCount_))
        {}

        /**
         * Default Constructor of ThreadPool
         *
        */
        ThreadPool() = default;

        /**
         * Copy Constructor of ThreadPool
         *
        */
        ThreadPool( const ThreadPool& other ) = delete;

        /**
         * Move Constructor of ThreadPool
         *
        */
        ThreadPool( const ThreadPool&& other ) noexcept;

        /**
         * Destructor of ThreadPool
         *
        */
        virtual ~ThreadPool();

        /**
         * Copy Assignement of ThreadPool
         *
        */
        ThreadPool& operator=( const ThreadPool& other ) = delete;

        /**
         * Move Assignment of ThreadPool
         *
        */
        ThreadPool& operator=( const ThreadPool&& other ) noexcept;

        /**
         * Initialize Thread Pool
         *
         * @param   NA
         * @return  void
         *
        */
        virtual void init() noexcept override;

        /**
         * Destroy Thread Pool
         *
         * @param   NA
         * @return  void
         *
        */
        virtual void destroy() noexcept override;

        /**
         * Add jobs in queue of Thread Pool
         *
         * @param[in]   f       function to be executed
         * @param[in]   args    args for function
         * @param[in]   prior   to add function at front or back
         * @return      void
         *
        */
        template<typename Function, typename... Args>
        void postJob( Function&& f, Args&&... args, bool prior = false )
        {
            // Take function and insert it into queue
            std::function<void()> task = std::bind(std::forward<Function>(f), std::forward<Args>(args)... );
            {
                // Take Lock
                const std::unique_lock<std::mutex> lock{syncMutex_};
                // Check prior and take decision
                if( true == prior ){
                    functionQueue_.emplace_front(task);
                } else{
                    functionQueue_.emplace_back(task);
                }
            }

            // Signal condition variable
            syncCondVar_.notify_one();
        }

        /**
         * Add jobs in queue of Thread Pool
         *
         * @param[in]   f       function to be executed with arguments
         * @return      void
         *
        */
        void postJob(std::function<void()> f)
        {
            {
                // Take Lock
                const std::unique_lock<std::mutex> lock{syncMutex_};
                // Check prior and take decision
                functionQueue_.emplace_back(f);
            }

            // Signal condition variable
            syncCondVar_.notify_one();
        }

    private:
        // Atomic flag for opeartion
        std::atomic_bool shutdownRequestedFlag_;

        // Thread Count variable
        uint32_t threadCount_;

        // Mutex for synchronization between tasks w.r.t. accessing queue
        std::mutex syncMutex_;

        // Condition Variable to sync for processing
        std::condition_variable syncCondVar_;

        // Threads Name
        std::string threadName_;

        // Deque to store function to be executed by thread pool
        std::deque<std::function<void()>> functionQueue_;

        // Vector of Threads to create specific number of threads
        // std::vector<std::thread> threadSpace_;
        std::unique_ptr<std::thread[]> threadSpace_;

        /**
         * Re-Entrant Execute function for Thread Pool
         *
         * @param   this    internally "this" being passed
         * @return  void
         *
        */
        virtual void execute() noexcept override;
};


}   // namespace utils

#endif  // thread_pool.hpp

/********************************** End of File *******************************/