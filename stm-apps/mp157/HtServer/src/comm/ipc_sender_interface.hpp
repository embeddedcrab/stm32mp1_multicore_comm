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
#ifndef _IPC_SENDER_INTERFACE_HPP_
#define _IPC_SENDER_INTERFACE_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <atomic>
#include <chrono>
#include <thread>

#include <iostream>

#include "comm_configs.hpp"

/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations/Definition
******************************************************************************/


// namespace core for core functionalities
namespace comm
{

/**
 * Implementation of IPC Sender Interface class
 */
template <typename T>
class IpcSenderInterface
{
  public:

    /**
     * Default Costructor of 
     *
     */
    IpcSenderInterface() = default;

    /**
     * Default Destructor of 
     *
     */
    virtual ~IpcSenderInterface() = default;

    /**
     * Constructors and Assignment operators
     *
     */
    IpcSenderInterface(const IpcSenderInterface &other) = delete;
    IpcSenderInterface(const IpcSenderInterface &&other);
    IpcSenderInterface &operator=(const IpcSenderInterface &other) = delete;
    IpcSenderInterface &operator=(const IpcSenderInterface &&other);


    /**
     * Basic functions for IPC Sender
     *
    */
    virtual void Init() noexcept = 0;
    virtual int32_t Send( T const &message ) noexcept = 0;
};


}

#endif  // ipc_sender_interface.hpp

/********************************** End of File *******************************/