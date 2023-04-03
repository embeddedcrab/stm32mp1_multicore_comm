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
#ifndef _SERVER_CORE_HPP_
#define _SERVER_CORE_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <atomic>
#include <chrono>
#include <thread>

#include <iostream>

#include "server_core_interface.hpp"
#include "ipc_receiver.hpp"


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations/Definition
******************************************************************************/

// namespace core
namespace core
{

/**
 * Implementation of Server Core class
 */
class ServerCore final
{
  public:

    /**
     * Default Constructor of
     *
     */
    ServerCore()
    : ipcServer_()
    {}

    /**
     * Default Destructor of 
     *
     */
    virtual ~ServerCore() = default;


    /**
     * Constructors and Assignment operators
     *
     */
    ServerCore(const ServerCore &other) = delete;
    ServerCore(const ServerCore &&other) = delete;
    ServerCore &operator=(const ServerCore &other) = delete;
    ServerCore &operator=(const ServerCore &&other) = delete;


    /**
     * Init()
     * @brief   Initialize Server Core
     * @param   NA
     * @return  void
     *
    */
    void Init()
    {
      std::cout << "[ServerCore] " << __FUNCTION__ << std::endl;
      ipcServer_.Init();
    }

    /**
     * Run()
     * @brief   Start Core Server processing and sleep until any signal comes
     * @param   NA
     * @return  void
     *
    */
    void Run()
    {
      std::cout << "[ServerCore] " << __FUNCTION__ << std::endl;

      // Start IPC Server
      ipcServer_.Run();

      std::chrono::system_clock::time_point next{std::chrono::system_clock::now() + std::chrono::milliseconds(500)};

      // Wait till shutdown is not requested
      while( !shutdownRequested_ ){
        // Sleep and again check for variable
        std::this_thread::sleep_until(next);
        next = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
      }
      std::cout << "[ServerCore] " << __FUNCTION__ << " - Exiting from here <><><><><>" << std::endl;
    }

    /**
     * Shutdown()
     * @brief   Stops Core Server processing
     * @param   NA
     * @return  void
     *
    */
    void Shutdown()
    {
      std::cout << "[ServerCore] " << __FUNCTION__ << std::endl;

      ipcServer_.Shutdown();
    }

    /**
     * ResetHandler()
     * @brief   Works as a Reset Handler of Application, nay critical/unreceoverable error
     *          will call this function to stop application
     * @param   NA
     * @return  void
     *
    */
    static void ResetHandler(){
      std::cout << "[ServerCore] " << __FUNCTION__ << std::endl;
      shutdownRequested_ = true;
    }

  private:
    // Variable to shutdown functioning of class
    static std::atomic<bool> shutdownRequested_;

    // Ipc Server Channel
    comm::IpcReceiver ipcServer_;
};

std::atomic<bool> ServerCore::shutdownRequested_{false};


} // namespace core

#endif  // server_core.hpp

/********************************** End of File *******************************/