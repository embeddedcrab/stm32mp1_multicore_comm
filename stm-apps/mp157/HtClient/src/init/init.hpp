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
#ifndef _INIT_HPP_
#define _INIT_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <thread>

#include <iostream>

#include <signal_handler.hpp>
#include <reset_handler.hpp>


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations
******************************************************************************/

namespace application
{


/**
 * Implementation of Initializer class
 */
template< class Application >
class InitApplication final
{
  public:

    /**
     * Init()
     * @brief     Initialize Application and start thread to run it
     * @param[in] argc  number of arguments
     * @param[in] argv  value of arguments
     * @return  void
     *
    */
    static void Init( int argc, char* argv[] )
    {
      static_cast<void>(argc);
      static_cast<void>(argv);

      // Create application Object and set Signal Handlers
      Application app;

      // Set System Reset Handler
      sysutils::SystemResetHandler::SetResetHandler( app.ResetHandler );
      // Wait for Signals or any error
      sysutils::SystemSignalHandler::Init();

      // Initialize Application
      app.Init();

      // Start Application in separate Thread
      std::thread AppThread( InitApplication::Start, std::ref(app) );

      // Wait for Thread to finish
      AppThread.join();

      // Shutdown Application
      app.Shutdown();

      std::cout << "[InitApplication] Application Shutdown....." << std::endl;
    }

    /**
     * Start()
     * @brief   Function to handle thread functioning
     * @param   NA
     * @return  void
     *
    */
    static void Start( Application& app )
    {
      // Start Application infinite loop
      app.Run();
      std::cout << "[InitApplication] Joining main Thread....." << std::endl;
    }
};


} // namespace application

#endif  // init.hpp

/********************************** End of File *******************************/