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
#ifndef _RESET_HANDLER_HPP_
#define _RESET_HANDLER_HPP_


/******************************************************************************
* Includes
******************************************************************************/
#include <cstdlib>

#include <iostream>
#include <functional>
#include <exception>


/******************************************************************************
* Configuration Constants
******************************************************************************/


/******************************************************************************
* Class Declarations/Definition
******************************************************************************/

namespace sysutils
{


/**
 * Implementation of Reset Handler class
 */
class SystemResetHandler final
{
  public:
    /**
     * Default Constructor of System Handler
     *
     */
    SystemResetHandler() = delete;
    ~SystemResetHandler() = default;

    /**
     * Init()
     * @brief   Init function to initialize Reset handler
     * @param   NA
     * @return  void
     *
    */
    static void Init()
    {
      // Set Default sysutils reset handler
      SetResetHandler(&Handler);
    }

    /**
     * SetResetHandler()
     * @brief     Function to set system reset handler
     * @param[in] handler function pointer to received function
     * @return  void
     *
    */
    static void SetResetHandler( void (*handler)() )
    {
      if( handler != nullptr ){
        // Set Terminate Handle
        std::terminate_handler handlerFnc = std::set_terminate(handler);
        // Handler was set successfully
        resetHandler_ = handler;
      }
    }

    /**
     * Handler()
     * @brief   Works as main default system reset handler
     * @param   NA
     * @return  void
     *
    */
    static void Handler()
    {
      if( resetHandler_ )
      {
        std::cout << "[SystemResetHandler] " << __FUNCTION__ << " - Calling Application System Reset Handler" << std::endl;
        resetHandler_();
      } else{
        // User Defined Action for System Failure
        std::cout << "[SystemResetHandler] " << __FUNCTION__ << " - System Abnormally Shutting Down" << std::endl;
        exit(-1);
      }
    }

  private:
    // System Reset handler function
    static std::terminate_handler resetHandler_;
};


// Initializing static variable
std::terminate_handler SystemResetHandler::resetHandler_{nullptr};

} // namespace sysutils


#endif  // reset_handler.hpp

/********************************** End of File *******************************/