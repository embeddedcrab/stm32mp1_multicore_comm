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
#ifndef _SIGNAL_HANDLER_HPP_
#define _SIGNAL_HANDLER_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <csignal>
#include <cstring>
#include <iostream>

#include "reset_handler.hpp"


/******************************************************************************
* Configuration Constants
******************************************************************************/

#define USING_SIGACTION
// #define USING_SIGNALSET


/******************************************************************************
* Class Declarations/Definition
******************************************************************************/

// namespace sysutils
namespace sysutils{


/**
 * Implementation of Signal Handler class
 */
class SystemSignalHandler final
{
  public:

    /**
     * Constructors and Destructors of class
     *
    */
    SystemSignalHandler() = delete;
    ~SystemSignalHandler() = default;


    /**
     * Init()
     * @brief   Initialize Signal Handler
     * @param   NA
     * @return  void
     *
    */
    static void Init()
    {
      #if defined(USING_SIGACTION)
      memset(&actions_, 0, sizeof(actions_));
      actions_.sa_handler = Handler;

      sigaction(SIGINT,  &actions_, NULL);
      sigaction(SIGKILL,  &actions_, NULL);
      sigaction(SIGTERM, &actions_, NULL);

      #elif defined(USING_SIGNALSET)
      // Initialize sigset with required signals for sysutils
      int32_t ret = sigemptyset(&signalSet_);
      if( !ret ){
        // Add Signals for monitoring
        ret = sigaddset(&signalSet_, SIGINT);
        if( !ret ){
          ret = sigaddset(&signalSet_, SIGKILL);
        }
      }

      // Check for status
      if( ret != 0 ){
        std::cout << "[SystemSignalHandler] " << __FUNCTION__ << " Signal Handler SIGSET failed, Shutting Down System with Error" << std::endl;
        reset();
      }
      #endif
    }

    #if defined(USING_SIGNALSET)
    static void SigWait()
    {
      int signum = 0;
      // Wait for Signal
      int ret = ::sigwait(&signalSet_, &signum);
      if( !ret ){
        // Call signal Handler
        Handler( signum );
      }
    }
    #endif

    /**
     * Handler()
     * @brief     Signal Handler function to handle signals
     * @param[in] signum  signal number received
     * @return  void
     *
    */
    static void Handler( int signum )
    {
      signalStatus_ = signum;

      // Check for type of Signal
      switch( signum )
      {
        case SIGINT:
          std::cout << "__SIGNAL_HANDLER__:: SIGINT, system Shutting Down" << std::endl;
        break;

        case SIGKILL:
          std::cout << "__SIGNAL_HANDLER__:: SIGKILL, system Shutting Down" << std::endl;
        break;

        case SIGTERM:
          std::cout << "__SIGNAL_HANDLER__:: SIGTERM, system Shutting Down" << std::endl;
        break;

        default:
          std::cout << "__SIGNAL_HANDLER__:: Unused Signal Occurred, system Shutting Down" << std::endl;
      }

      // TODO: Could add Reset System for specific signals only
      Reset();
    }

    // Signal Status
    static volatile std::sig_atomic_t signalStatus_;

  private:
    #if defined(USING_SIGACTION)
      // Signal Actions
      static struct sigaction actions_;
    #elif defined(USING_SIGNALSET)
      // Signal Set
      static sigset_t signalSet_;
    #endif

    /**
     * Reset()
     * @brief   Fucntion to reset signals and call appropriate handler
     * @param   NA
     * @return  void
     *
    */
    static void Reset()
    {
      #if defined(USING_SIGACTION)
      // Reset Signals
      struct sigaction old_actions;
      memset(&old_actions, 0, sizeof(old_actions));

      sigaction(SIGINT, NULL, &old_actions);
      sigaction(SIGKILL, NULL, &old_actions);
      sigaction(SIGTERM, NULL, &old_actions);

      #elif defined(USING_SIGNALSET)

      #endif

      // Call System reset handler
      SystemResetHandler::Handler();
    }
};


#if defined(USING_SIGACTION)
// Signal Actions
struct sigaction SystemSignalHandler::actions_{};
#elif defined(USING_SIGNALSET)
// Signal Set
sigset_t SystemSignalHandler::signalSet_{};
#endif

// Signal Status
volatile std::sig_atomic_t SystemSignalHandler::signalStatus_{0};


} // namespace sysutils

#endif  // signal_handler.hpp

/********************************** End of File *******************************/