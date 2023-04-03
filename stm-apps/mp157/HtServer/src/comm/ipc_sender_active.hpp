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
#ifndef _IPC_SENDER_ACTIVE_HPP_
#define _IPC_SENDER_ACTIVE_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <thread>
#include <atomic>
#include <queue>
#include <functional>

#include <iostream>

#include "ipc_sender.hpp"


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations/Definition
******************************************************************************/

// communication namespace
namespace comm
{


/**
 * Implementation of IPC Sender Active class
 * 
 * @brief Processes IPC Sender requests one by one
 */
template <typename T = CommShmData>
class IpcSenderActive
{
  public:
    /**
     * Default Constructor of IpcSenderActive
     *
     */
    explicit IpcSenderActive( IpcSenderInterface<T> &ipc )
    : runningFlag_{true},
      requestQueue_{},
      activeThread_{}, 
      syncMutex_{},
      syncCondVar_{},
      ipcInterface_(ipc)
    {}

    /**
     * Default Destructor of 
     *
     */
    virtual ~IpcSenderActive()
    {
      std::cout << "[IpcSenderActive] Joining Threads....." << std::endl;
      // Join Thread
      if( activeThread_.joinable() )
      {
        activeThread_.join();
      }
    }


    /**
     * Constructors and Assignment operators
     *
     */
    IpcSenderActive() = delete;
    IpcSenderActive(const IpcSenderActive &other) = delete;
    IpcSenderActive(const IpcSenderActive &&other);
    IpcSenderActive &operator=(const IpcSenderActive &other) = delete;
    IpcSenderActive &operator=(const IpcSenderActive &&other);


    /**
     * Init()
     * @brief   Iniitalize Thread as Active Object to process requests
     * @param   NA
     * @return  void
     *
    */
    void Init()
    {
      // Start Thread to process requests
      activeThread_ = std::thread(&IpcSenderActive::RequestHandler, this);
      pthread_setname_np(activeThread_.native_handle(), IPC_SENDER_ACTIVE_THREAD_NAME.c_str());
    }

    /**
     * Shutdown()
     * @brief   Reset variable responsible for thread functioning to stop processing
     * @param   NA
     * @return  void
     *
    */
    void Shutdown()
    {
      runningFlag_ = false;
      syncCondVar_.notify_all();
    }

    /**
     * AddRequest()
     * @brief     Add Requests into queue
     * @param[in] message message to be transmitted
     * @return  void
     *
    */
    void AddRequest( T const & request_message )
    {
      // Insert message into queue
      std::lock_guard<std::mutex> lock{syncMutex_};
      requestQueue_.emplace(request_message);
      std::cout << __FUNCTION__ << " Request Added into queue fo processing" << std::endl;

      syncCondVar_.notify_one();
    }

  private:

    // Shutdown variable to stop thread
    std::atomic_bool runningFlag_;

    /// Queue to store requests
    std::queue<T> requestQueue_;

    // Thread to handle requests
    std::thread activeThread_;

    // Mutex for synchronization to access queue
    std::mutex syncMutex_;

    // Condition Variable to sync for data processing
    std::condition_variable syncCondVar_;

    // Reference of Ipc Sender function to be used internally
    std::reference_wrapper<IpcSenderInterface<T>> ipcInterface_;

    /**
     * RequestHandler, Thread function to process requests
     *
     * @param[in] this  class reference internally passed while thread creation
     * @return  void
     *
    */
    void RequestHandler()
    {
      for( ; ; )
      {
        T data_to_send{};

        {
        // Take lock and wait for some event on lock or any data presence in queue
        std::unique_lock<std::mutex> lock{syncMutex_};
        syncCondVar_.wait(lock, [this]
                          { return !requestQueue_.empty() || !runningFlag_; });

        // Check for running flag
        if( (!runningFlag_) && requestQueue_.empty() )
        {
          return;
        }

        // Pop request from queue and process
        data_to_send = requestQueue_.front();
        requestQueue_.pop();
        }

        // Send data to IPC
        static_cast<void>(ipcInterface_.get().Send(data_to_send));
        std::cout << __FUNCTION__ << " - Sent data to IPC Interface" <<std::endl;
      }
    }
};


} // namespace comm

#endif  // ipc_sender_active.hpp

/********************************** End of File *******************************/