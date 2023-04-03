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
#ifndef _IPC_SENDER_HPP_
#define _IPC_SENDER_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <iostream>

#include <shared_memory.hpp>
#include <reset_handler.hpp>

#include "comm_configs.hpp"

#include "ipc_sender_interface.hpp"


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
 * Implementation of IPC Sender class, uses shared memory
 */
template <typename T = CommShmData, uint32_t SIZE = 4096U>
class IpcSender : public IpcSenderInterface<T>
{
  public:
    /**
     * Default Constructor of IpcSender
     *
     * Works as a creator of Shared Memory and Semaphore
     */
    explicit IpcSender( std::string shm_name, std::string semaphore_name )
    : semaphoreInterface_(semaphore_name, utils::SemaphoreI::SemaphoreFlags::SEM_CREATE, 0), 
      shmInterface_(shm_name, SIZE, semaphoreInterface_, utils::SharedMemoryI::ShmFlags::SHM_CREATE_RDWR)
    {}

    /**
     * Default Destructor of 
     *
     */
    virtual ~IpcSender()
    {
      if( shmInterface_.unmap() ){
        std::cout << "SHM UnMapping Failed" << std::endl;
      }
    }


    /**
     * Constructors and Assignment operators
     *
     */
    IpcSender(const IpcSender &other) = delete;
    IpcSender(const IpcSender &&other);
    IpcSender &operator=(const IpcSender &other) = delete;
    IpcSender &operator=(const IpcSender &&other);


    /**
     * Init()
     * @brief   Iniitalize Communication interface base
     * @param   NA
     * @return  void
     *
    */
    virtual void Init() noexcept override
    {
      std::cout << "[IpcSender] " << __FUNCTION__ << std::endl;

      // Initialize Shared Memory
      int32_t status = shmInterface_.map(0);
      if( status ){
        std::cout << "Shared Memory Mapping Failed, Exiting\n";
        sysutils::SystemResetHandler::Handler();
      }
    }

    /**
     * Send()
     * @brief       Send message to shared memory
     * @param[in]   msg message to be transferred
     * @return  void
     *
    */
    virtual int32_t Send( T const &message ) noexcept override
    {
      // Initialize Data for SHM
      int32_t ret{0};

      std::cout << "Data Size to be written is = " << sizeof(message) << std::endl;

      if( 1 == shmInterface_.write(message, sizeof(message)) ){
        std::cout << __FUNCTION__ << " - writing data to SHM [FAILED.....]" << std::endl;
        ret = 1;
      }

      // Post signal for Reader
      if( -1 == shmInterface_.getSemaphore()->post() ){
        std::cout << __FUNCTION__ << " - writing data to SHM [SEMAPHORE FAILED.....]" << std::endl;
        ret = 1;
      }

      return ret;
    }
  
  private:
    // Create Named Semaphore
    utils::Semaphore semaphoreInterface_{};

    // Open Shared Memory as Writer (Creator)
    utils::SharedMemory<T> shmInterface_{};
};


} // namespace comm

#endif  // ipc_sender.hpp

/********************************** End of File *******************************/