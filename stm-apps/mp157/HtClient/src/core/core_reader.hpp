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
#ifndef _CORE_READER_HPP_
#define _CORE_READER_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <atomic>
#include <thread>

#include <iostream>


#include "ipc_reader.hpp"


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations
******************************************************************************/


namespace core
{


/**
 * Implementation of Core Reader class
 */
class CoreReader final
{
  public:

    /**
     * Default Constructor of
     *
     */
    CoreReader()
    : ipcReaderObj_()
    {}

    /**
     * Default Destructor of 
     *
     */
    virtual ~CoreReader() = default;


    /**
     * Constructors and Assignment operators
     *
     */
    CoreReader(const CoreReader &other) = delete;
    CoreReader(const CoreReader &&other) = delete;
    CoreReader &operator=(const CoreReader &other) = delete;
    CoreReader &operator=(const CoreReader &&other) = delete;


    /**
     * Init()
     * @brief   Initialize Server Core
     * @param   NA
     * @return  void
     *
    */
    void Init()
    {
      std::cout << "[CoreReader] " << __FUNCTION__ << std::endl;
      ipcReaderObj_.Init();
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
      std::cout << "[CoreReader] " << __FUNCTION__ << std::endl;
      ipcReaderObj_.Run();

      std::chrono::system_clock::time_point next{std::chrono::system_clock::now() + std::chrono::milliseconds(500)};

      // Wait till shutdown is not requested
      while( !shutdownRequested_ ){
        // Sleep and again check for variable
        std::this_thread::sleep_until(next);
        next = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
      }
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
      std::cout << "[CoreReader] " << __FUNCTION__ << std::endl;
      ipcReaderObj_.Shutdown();
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
      std::cout << "[CoreReader] " << __FUNCTION__ << std::endl;
      shutdownRequested_ = true;
    }

  private:
    // Variable to shutdown functioning of class
    static std::atomic<bool> shutdownRequested_;

    // IPC Reader
    comm::IpcReader ipcReaderObj_;
};

std::atomic<bool> CoreReader::shutdownRequested_{false};


} // namespace core

#endif  // core_reader.hpp

/********************************** End of File *******************************/