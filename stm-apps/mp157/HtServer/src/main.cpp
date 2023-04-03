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

#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <thread>

// shared memory
#include <shared_memory.hpp>
// thread pool
#include <thread_pool.hpp>

// Application Initialization
#include "init.hpp"

// Server Class declaration
#include "server_core.hpp"


// Some Constructors and Destructors for Init and De-Init tasks
extern "C"{
void InitB4Main( void ) __attribute__ ((constructor));
void DeInitAfterMain( void ) __attribute__ ((destructor));
}


#if 0
class Demo{
  public:
    Demo(){}
    ~Demo() = default;

    void Init(){
      std::cout << "Init Application" << std::endl;
    }

    void Run(){
      std::cout << "Running Application" << std::endl;
      std::chrono::system_clock::time_point next{std::chrono::system_clock::now() + std::chrono::milliseconds(500)};

      // Wait till shutdown is not requested
      while( !shutdownRequested ){
        // Sleep and again check for variable
        std::this_thread::sleep_until(next);
        next = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
        std::cout << "Application Running......." << std::endl;
      }

      std::cout << "Exiting Main Application" << std::endl;
    }

    void Shutdown(){
      std::cout << "Shutdown Application" << std::endl;
    }

    static void ResetHandler(){
      std::cout << "Reset handler of Application" << std::endl;
      shutdownRequested = true;
    }

    // std::atomic is safe, as long as it is lock-free
    static std::atomic<bool> shutdownRequested;
};

std::atomic<bool> Demo::shutdownRequested{false};
#endif


int main( int argc, char* argv[] )
{
  sysutils::SystemResetHandler::Init();

  try{
    // Create Object of Application
    application::InitApplication<core::ServerCore>::Init( argc, argv);
  } catch( const std::exception& e ){
    std::cout << "Application Could not be initialized, Exiting....." << std::endl;
    exit(-1);
  }

  return 0;
}



extern "C"{
void InitB4Main( void )
{
  printf("%s\n", "InitB4Main Constructor");
}

void DeInitAfterMain( void )
{
  printf("%s\n", "DeInitAfterMain Destructor");
}
}