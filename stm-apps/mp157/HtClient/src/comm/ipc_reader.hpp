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
#ifndef _IPC_READER_HPP_
#define _IPC_READER_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <string>

#include <iostream>

#include <thread_pool.hpp>


#include "ipc_read.hpp"


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations
******************************************************************************/


namespace comm
{


/**
 * Implementation of IPC Reader class
 */
class IpcReader
{
  public:
    /**
     * Constructor of IpcReader
     *
     */
    explicit IpcReader( std::string shm_name, std::string sem_name = "" )
    : shutdownRequested_{false},
      threadPool_(IPC_SERVER_THREAD_POOL_THREADS, IPC_SERVER_THREAD_POOL_NAME),
      msgReceiverThread_{},
      ipcRead_(shm_name, sem_name)
    {}

    /**
     * Default Constructor of 
     *
     */
    IpcReader() : IpcReader(SharedMemoryName, SemaphoreName)
    {}

    /**
     * Default Destructor of 
     *
     */
    virtual ~IpcReader()
    {
      // Join Thread running to receive data over pipe
      if( msgReceiverThread_.joinable() )
      {
        msgReceiverThread_.join();
      }
    }


    /**
     * Constructors and Assignment operators
     *
     */

    IpcReader(const IpcReader &other) = delete;
    IpcReader(const IpcReader &&other);
    IpcReader &operator=(const IpcReader &other) = delete;
    IpcReader &operator=(const IpcReader &&other);


    /**
     * Init
     *
     * @param   NA
     * @return  void
     *
    */
    void Init()
    {
      std::cout << "[IpcReader] " << __FUNCTION__ << std::endl;
      // Initialize Thread Pool
      threadPool_.init();
      // Iniitalize SHM Mapping
      ipcRead_.Init();
    }

    /**
     * Run()
     * @brief   Iniitalize Receiver Thread to receive data from SHM
     * @param   NA
     * @return  void
     *
    */
    void Run()
    {
      std::cout << "[IpcReader] " << __FUNCTION__ << std::endl;
      
      // Initialize thread to receive data from named pipe
      msgReceiverThread_ = std::thread( &IpcReader::ReceiveMessages, this );
      pthread_setname_np(msgReceiverThread_.native_handle(), IPC_READER_THREAD_NAME.c_str());
    }

    /**
     * Shutdown()
     * @brief   Reset variables responsible for processing
     * @param   NA
     * @return  void
     *
    */
    void Shutdown()
    {
      std::cout << "[IpcReader] " << __FUNCTION__ << std::endl;
      shutdownRequested_ = true;
    }
  
  private:
    // Variable to shutdown functioning of class
    std::atomic<bool> shutdownRequested_;

    // Thread to handle 
    utils::ThreadPool threadPool_;

    // Thread variable
    std::thread msgReceiverThread_;

    // IPC Reader
    IpcRead<CommShmStrData> ipcRead_;

    /**
     * ProcessMessage()
     * @brief   Thread to process received data
     * @param   NA
     * @return  void
     *
    */
    void ProcessMessage( CommShmStrData const message )
    {
      std::cout << "Data Received from SHM is [length = " << message.length 
                << ", id = " << message.id << ", name = " << message.name << "]" << std::endl;
    }

    /**
     * ReceiveMessages()
     * @brief   Thread to read data from SHM
     * @param   NA
     * @return  void
     *
    */
    void ReceiveMessages()
    {
      int32_t ret{};
      // Local variable to thread function
      CommShmStrData message{};

      while( !shutdownRequested_ )
      {
        // std::cout << "Waiting for data in Client" << std::endl;
        // Read Data from SHM whenever available
        ret = ipcRead_.Read(message, 5000, sizeof(CommShmStrData));
        // Check status of read
        if( !ret ){
          // Pass it to Thread Pool's Thread Function to process data
          threadPool_.postJob(std::bind(&IpcReader::ProcessMessage, this, message));
        }
      }
    }
};


} // namespace comm

#endif  // ipc_reader.hpp

/********************************** End of File *******************************/