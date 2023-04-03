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
#ifndef _IPC_RECEIVER_HPP_
#define _IPC_RECEIVER_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <atomic>
#include <thread>
#include <string>
#include <cstring>

#include <fcntl.h>
#include <termios.h>

#include <iostream>

#include <thread_pool.hpp>

#include "message_conversion.hpp"
#include "ipc_sender_active.hpp"


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
 * Implementation of IPC Receiver class
 */
class IpcReceiver
{
  public:
    /**
     * Constructor of Ipc receiver
     *
     */
    explicit IpcReceiver( std::string const pipe_name, std::string shm_name, std::string sem_name = "" )
    : shutdownRequested_{false},
      pipeFd_{-1},
      pipeName_{pipe_name}, 
      threadPool_(IPC_SERVER_THREAD_POOL_THREADS, IPC_SERVER_THREAD_POOL_NAME),
      msgReceiverThread_{},
      ipcTransmit_(shm_name, sem_name),
      ipcTransmitActive_(ipcTransmit_)
    {}

    /**
     * Default Constructor of 
     *
     */
    IpcReceiver() : IpcReceiver(ReadNamedPipe, SharedMemoryName, SemaphoreName)
    {}


    /**
     * Default Destructor of 
     *
     */
    virtual ~IpcReceiver()
    {
      std::cout << "[IpcReceiver] Joining Threads....." << std::endl;
      // Join Thread running to receive data over pipe
      if( msgReceiverThread_.joinable() )
      {
        msgReceiverThread_.join();
      }

      // Close Pipe when thread goes off
      static_cast<void>(close(pipeFd_));

      std::cout << "[IpcReceiver] " << __FUNCTION__ << " Threads finished" << std::endl;
    }


    /**
     * Constructors and Assignment operators
     *
     */

    IpcReceiver(const IpcReceiver &other) = delete;
    IpcReceiver(const IpcReceiver &&other);
    IpcReceiver &operator=(const IpcReceiver &other) = delete;
    IpcReceiver &operator=(const IpcReceiver &&other);


    /**
     * Init
     *
     * @param   NA
     * @return  void
     *
    */
    void Init()
    {
      std::cout << "[IpcReceiver] " << __FUNCTION__ << std::endl;
      // Initialize Thread Pool
      threadPool_.init();
      ipcTransmit_.Init();
      // Start Active Thread Processing
      ipcTransmitActive_.Init(); 
    }

    /**
     * Run()
     * @brief   Iniitalize Receiver Thread to receive data on server
     * @param   NA
     * @return  void
     *
    */
    void Run()
    {
      std::cout << "[IpcReceiver] " << __FUNCTION__ << std::endl;
      
      // Initialize thread to receive data from named pipe
      msgReceiverThread_ = std::thread( &IpcReceiver::ReceiveMessages, this );
      pthread_setname_np(msgReceiverThread_.native_handle(), 
                          IPC_SERVER_MESSAGE_RECEIVER_THREAD_NAME.c_str());
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
      std::cout << "[IpcReceiver] " << __FUNCTION__ << std::endl;
      shutdownRequested_ = true;
      ipcTransmitActive_.Shutdown();
    }
  
  private:
    // Variable to shutdown functioning of class
    std::atomic<bool> shutdownRequested_;

    // File Descriptor for Named Pipe
    int pipeFd_;

    // Name of Named Pipe
    std::string pipeName_;

    // Create Client Thread to write data to SHM
    utils::ThreadPool threadPool_;

    // Thread variable
    std::thread msgReceiverThread_;

    // IPC Transmitter
    IpcSender<CommShmStrData> ipcTransmit_;

    // IPC Transmitter Active Object variable to send data
    IpcSenderActive<CommShmStrData> ipcTransmitActive_;

    /**
     * ProcessRequest()
     * @brief Reentrant function to process request of server
     * @param   NA
     * @return  void
     *
    */
    void ProcessRequest( const uint32_t len, const uint32_t i, std::string const &str ) noexcept
    {
      std::cout << "[IpcReceiver::ProcessRequest] - Received MSG is [id - " << i << " with length - " 
                                << len << " and Name = " << str <<  "]" << std::endl;
      // Create Data structure for request and pass ti to active object
      CommShmStrData request{.length = len, .id = i};
      ::strncpy(request.name, str.c_str(), SizeOfName);
      ipcTransmitActive_.AddRequest(request);
    }

    /**
     * ReceiveMessages()
     * @brief   Thread to receive data from Named Pipe
     * @param   NA
     * @return  void
     *
    */
    void ReceiveMessages()
    {
      // Local variable to thread
      char message[MessageSize]{};
      std::string msg_data{};
      uint32_t msg_id{};
      uint32_t msg_length{0};
      int status{0};

      // Block signals
      sigset_t sigmask;
      {
          sigset_t blocked;
          ::sigemptyset(&blocked);
          ::sigaddset(&blocked, SIGINT);
          ::sigaddset(&blocked, SIGKILL);
          ::sigaddset(&blocked, SIGTERM);
          int ret = ::sigprocmask(SIG_BLOCK, &blocked, &sigmask);
          if( -1 == ret ){
            // Pipe could not be opened, stop processing and exit
            sysutils::SystemResetHandler::Handler();
          }
      }
      // Empty signal mask sued for pselect
      ::sigemptyset(&sigmask);

      // Define pselect specific variables
      fd_set fdsRead{};
      struct timespec timeout{.tv_sec = 2, .tv_nsec = 0};
      struct termios tty;

      // Open Pipe for operation, if failed then trigger shutdown
      pipeFd_ = ::open( pipeName_.c_str(), O_RDWR | O_NOCTTY );
      // Check fd
      if( -1 == pipeFd_ ){
        // Pipe could not be opened, stop processing and exit
        sysutils::SystemResetHandler::Handler();
      }

      // Send a dummy message to M4 side on same pipe, needed for stm32mp1
      ::write(pipeFd_, "Hello World", strlen("HelloWorld") );

      ::tcflush(pipeFd_, TCIOFLUSH);
      ::tcgetattr( pipeFd_, &tty );
      ::cfmakeraw( &tty );
      ::tcsetattr( pipeFd_, TCSANOW, &tty );

      // Set Pipe FD Set for pselect
      FD_ZERO(&fdsRead);
      FD_SET(pipeFd_, &fdsRead);

      std::cout << "[IpcReceiver::ReceiveMessages] Starting Reading data on Server Pipe" << std::endl;

      // Forever processing of requests
      while( !shutdownRequested_ )
      {
        // Read file descriptor using pselect (signal environment)
        // TODO: Check functioning of pselect to watch descriptor with timeout
        status = ::pselect( (pipeFd_ + 1), &fdsRead, NULL, NULL, NULL, &sigmask );

        // Check pselect status
        if( -1 == status ){
          std::cout << "[IpcReceiver::ReceiveMessages] pselect failed!! and continue....." << std::endl;
          continue;
        } else if( 0 == status ){
          continue;
        }

        // Check FD Status for Read Pipe
        if (FD_ISSET(pipeFd_, &fdsRead))
        {
          // Read data from pipe
          status = read( pipeFd_, message, MessageSize );
          // Check return status of read from pipe
          if( status ){
            std::cout << "[IpcReceiver::ReceiveMessages] Received data is " << message << std::endl;
            // Extract request data from message
            DeserializeCommDataCreate(message, msg_length, msg_id, msg_data);

            // Call function to add request into queue using active object
            threadPool_.postJob(std::bind(&IpcReceiver::ProcessRequest, this, msg_length, msg_id, msg_data));

            // Reset message for next read
            memset(static_cast<void *>(message), '\0', MessageSize);
          }
        }
      }

      std::cout << "[IpcReceiver::ReceiveMessages] Exiting pselect w/Thread from " << __FUNCTION__ << std::endl;
    }
};


} // namespace comm

#endif  // ipc_receiver.hpp

/********************************** End of File *******************************/