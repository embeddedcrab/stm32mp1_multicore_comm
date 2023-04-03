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
#include "thread_pool.hpp"


/******************************************************************************
* Constant Declarations
******************************************************************************/


/******************************************************************************
* Class Function Definitions
******************************************************************************/

namespace utils{


ThreadPool::~ThreadPool()
{
  if(false == shutdownRequestedFlag_){
    destroy();
  }
}


void ThreadPool::init() noexcept
{
  shutdownRequestedFlag_ = false;

  // Initialize Thread and their names
  for( uint32_t count{0}; count < threadCount_; ++count )
  {
    threadSpace_[count] = std::thread(&ThreadPool::execute, this);
    pthread_setname_np(threadSpace_[count].native_handle(), (threadName_ + std::to_string(count)).c_str());
  }
}


void ThreadPool::destroy() noexcept
{
  // Reset RunninFlag to stop opeartion
  shutdownRequestedFlag_ = true;

  // Notify All
  syncCondVar_.notify_all();

  std::cout << "ThreadPool " << threadName_ << " joining Threads....." << std::endl;

  // Let Threads finish their task and end
  // for (auto &thread : threadSpace_)
  for( uint32_t count{0}; count < threadCount_; ++count )
  {
    if (threadSpace_[count].joinable()){
      threadSpace_[count].join();
    }
  }

  std::cout << "ThreadPool " << threadName_ << " Threds joining finished and destroyed" << std::endl;
}


void ThreadPool::execute() noexcept
{
  // Run forever loop
  for (;;)
  {
    std::function<void()> func;
    {
      // Take lock and wait for any activity on it
      std::unique_lock<std::mutex> lock{syncMutex_};
      syncCondVar_.wait(lock, [this]
                        { return !functionQueue_.empty() || shutdownRequestedFlag_; });

      // Check for running flag
      if( (shutdownRequestedFlag_) && functionQueue_.empty() )
      {
        return;
      }

      // Take job from queue
      func = functionQueue_.front();
      functionQueue_.pop_front();
    }

    // Start Job execution
    func();
  }
}


}   // namespace utils

/********************************** End of File *******************************/