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
#ifndef _COMM_CONFIGS_HPP_
#define _COMM_CONFIGS_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <string>


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations/Definition
******************************************************************************/

// communication namespace
namespace comm
{

// Constants for Message transmission
constexpr uint8_t Delimiter{','};
constexpr uint32_t MessageSize{4096U};
constexpr uint32_t SizeOfName{32U};

// Constants for IPC Thread Pool
constexpr uint32_t IPC_SERVER_THREAD_POOL_THREADS{2U};
const std::string IPC_SERVER_THREAD_POOL_NAME{"IPC_ThreadPool"};

// IPC Server Thread Name
const std::string IPC_SERVER_MESSAGE_RECEIVER_THREAD_NAME{"IPC_RECV_Thread"};
const std::string IPC_SENDER_ACTIVE_THREAD_NAME{"IPC_ACTIVE_THREAD"};

#ifdef __linux

// Note: Need to explicitly create pipe, SW is not creating any
//        It can also be tested on linux machine
const std::string ReadNamedPipe{"/dev/ttyRPMSG0"};
const std::string WriteNamedPipe{"/dev/ttyRPMSG1"};

const std::string SharedMemoryName{"/sharedMemoryHT"};
const std::string SemaphoreName{"/semaphoreHT"};

#else

// Named Pipes for communication with Co-Processor
const std::string ReadNamedPipe{"/dev/ttyRPMSG0"};
const std::string WriteNamedPipe{"/dev/ttyRPMSG0"};

const std::string SharedMemoryName{"/sharedMemoryHT"};
const std::string SemaphoreName{"/semaphoreHT"};

#endif

/**
 * Structure definition for Shared Memory Data
 */
struct CommShmData
{
  uint32_t length;
  uint32_t id;
};


struct CommShmStrData
{
  uint32_t length{0};
  uint32_t id{0};
  char name[SizeOfName]{'\0'};
};

} // namespace comm

#endif  // comm_configs.hpp

/********************************** End of File *******************************/